
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <ceres/ceres.h>

#include "generic_sensor.hpp"
#include "error_models.hpp"    // for Euclidean3DError
#include "linearTF_solver.hpp" // for LinearTfSolver

#include "background_filter.hpp"
#include "morph_operator.hpp"
#include "blob_extractor.hpp"
#include "sphere_fitter.hpp"

/***************************************
           class RangeSensor
***************************************/
class RangeSensorOptions
{
public:
  const double focal_length;
  const int width, height;
  const float bg_threshold, near_cutoff, far_cutoff, morph_radius;
  const double target_radius, min_volumn, max_volumn;
  const int    min_count, max_count;

  RangeSensorOptions(const double fl, const int w, const int h,
		     const float bgt, const float nc, const float fc, const float mr,
		     const double tr, const double mv, const double Mv, const int mc, const int Mc)
    :focal_length(fl), width(w), height(h), 
     bg_threshold(bgt), near_cutoff(nc), far_cutoff(fc), morph_radius(mr), target_radius(tr),
     min_volumn(mv), max_volumn(Mv), min_count(mc), max_count(Mc){}
};

class RangeSensor : public Sensor
{
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

public:
  RangeSensor(std::string name="rs", std::string type="rs")
 :Sensor(name, type){}
  
  static boost::shared_ptr<Sensor> Create(std::string& name, std::string& type)
  {
    return boost::shared_ptr<Sensor>(new RangeSensor(name, type));
  }
  /*********************************************************
  **********************************************************
        Functions related to Solving the rigid transform
   *********************************************************
   ********************************************************/
  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk)
  {
    LinearTfSolver lts(_measure.size());
    std::map<int, Eigen::Vector3d>::iterator it = _measure.begin();
    while( it != _measure.end() )
      {
	lts.AddPointPair(ldmk[it->first], it->second);
	++it;
      }
  
    lts.EstimateTfSVD(&(_origin[0]));
  }

  virtual void InitCeresProblem(ceres::Problem& prob,
				std::vector<Eigen::Vector3d>& ldmk)
  {
    std::map<int, Eigen::Vector3d>::iterator it = _measure.begin();
    while( it != _measure.end() )
      {
	Eigen::Vector3d& v = it->second;
	//	ceres::CostFunction* cf = Euclidean3DError::Create(v(0),v(1),v(2), false);
	ceres::CostFunction* cf = Euclidean3DError::AngleReprojError::Create(v(0), v(1), v(2));
	ceres::LossFunction* lf = NULL;
	prob.AddResidualBlock(cf, lf, &(_origin[3]), &(_origin[0]), ldmk[it->first].data());
	++it;
      }
    prob.SetParameterization(&(_origin[3]), new ceres::QuaternionParameterization);
  }

  /*********************************************************
  **********************************************************
          Functions related to dectecting targets
   *********************************************************
   ********************************************************/
  virtual bool RunPipeline(void* query, void* train, void* options, 
			   std::vector<int>& inlier_idx, Eigen::Vector3d& result)
  {
    CloudType::Ptr fg = *static_cast<CloudType::Ptr *>(query);
    CloudType::Ptr bg = *static_cast<CloudType::Ptr *>(train);

    RangeSensorOptions* opt = static_cast<RangeSensorOptions *>(options);


    std::vector<char> mask;
    ImageFilter<PointType> img_filter(opt->bg_threshold, opt->near_cutoff, opt->far_cutoff);
    img_filter.AddBackgroundCloud(bg);

    img_filter.GetForegroundMask(fg, mask);

    img_filter.WriteMaskToFile("mask.dump", mask);

    // Morphological operations:
    Morphology::Erode2_5D<PointType> (fg, mask, opt->focal_length, opt->morph_radius);

    img_filter.WriteMaskToFile("erode.dump", mask);

    Morphology::Dilate2_5D<PointType>(fg, mask, opt->focal_length, opt->morph_radius);

    img_filter.WriteMaskToFile("final.dump", mask);

    // Extract clusters from binaryImage
    std::vector<pcl::PointIndices> cluster_list;

    FastBlobExtractor<PointType> fbe(opt->width, opt->height, 
				     opt->min_count,  opt->max_count, 
				     opt->min_volumn, opt->max_volumn);

    fbe.setInputCloud(fg);
    fbe.extract(cluster_list, mask);

    // Not cluster could meet the requirements
    if(cluster_list.empty())
      return false;

    // TODO: make the following a function/class: LinearFitterSelector
    // Find the cluster that is most likely to be a ball
    double cost, best_cost = 1e5;
    int best_idx = -1;
    std::vector<Eigen::Vector3d> centers(cluster_list.size());
    for(int t=0; t< cluster_list.size();++t)
      {
	// Some cluster may get trimmed in blob extractor
	if(cluster_list[t].indices.empty())
	  continue;

	LinearFitter<PointType> lsf(opt->target_radius);
	lsf.SetInputCloud(fg, cluster_list[t].indices);
	cost = lsf.ComputeFitCost(centers[t]);
	std::cout << "Center at "<< centers[t].transpose() <<" with cost = "<< cost<<std::endl;
	if (cost < best_cost)
	  {
	    best_cost = cost;
	    best_idx = t;
	  }
      }

    // Nonlinear refinement
    NonlinearFitter<PointType> nlsf(opt->target_radius);
    nlsf.SetInputCloud(fg, cluster_list[best_idx].indices);
    // Notice this is a different kind of cost, not comparable to linear fit cost
    cost = nlsf.ComputeFitCost(centers[best_idx]);

    double fl = opt->focal_length;
    /*    if (_type == "sr")
      fl = -opt->focal_length;
    else
      fl = opt->focal_length;
    */
    remask_inliers(fg, centers[best_idx], inlier_idx, opt->width, opt->height,fl,opt->target_radius);
    
    nlsf.Clear();//reset the cost function
    nlsf.SetInputCloud(fg, inlier_idx);
    cost = nlsf.ComputeFitCost(centers[best_idx]);

    result = centers[best_idx];
    return true;
  }

private:
  void remask_inliers(CloudType::Ptr cloudp, Eigen::Vector3d& ctr, std::vector<int>& inliers,
		      const int width, const int height, const double fl, const double target_radius)
  {
    int u0 = width /2 + static_cast<int>(ctr(0) / ctr(2) * fl);
    int v0 = height/2 + static_cast<int>(ctr(1) / ctr(2) * fl);
    int ws = abs(2*target_radius / ctr(2) * fl);
 
    printf("Search window = %d, centered at <%d, %d>\n", ws, u0, v0);

    //  std::vector<int> inlier_list;
    for(int u=u0-ws; u<=u0+ws; ++u)
      {
	for(int v=v0-ws; v<=v0+ws; ++v)
	  {
	    if (u<0 || u>=width || v<0 || v>=height)
	      continue;

	    // This condition ensures a circular shape
	    //	  if (ws*ws < ((u-u0)*(u-u0)+(v-v0)*(v-v0)))
	    //	    continue;

	    int linear_idx = v*width+u;
	    PointType& p = cloudp->points[linear_idx];

	    double d = sqrt((p.x-ctr(0))*(p.x-ctr(0)) +
			    (p.y-ctr(1))*(p.y-ctr(1)) +
			    (p.z-ctr(2))*(p.z-ctr(2)));

	    double diff = std::abs(d-target_radius);
#define RATIO 0.5
	    //	    if (diff < (RATIO*target_radius) )
	      {
		inliers.push_back(linear_idx);
		//		printf("d=%lf, diff=%lf\n", d, diff);
	      }
	  }
      }

    //    printf("# of inliers=%d\n", inliers.size());
  }
};








