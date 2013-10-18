/*
  Assume we already know where the balls are located and where the 
  cameras are placed, project the balls back to camera frame and
  do a refit
 */

#include "error_models.hpp"

// For nonlinear fitting
#include <ceres/ceres.h>
#include <ceres/types.h>

// For loading config files
#include <yaml-cpp/yaml.h>

// PCL related stuff
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <fstream>
#include <iomanip>

#include <vector>
#include <array>

#define SHOW_INLIER   1
//#define PERCENT_RADIUS 0.8 // take points within 80% of radius
#define MAX_JACOBIAN_THREADS 4

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

// Extract inlier points given a detection
void filterInliersFromDetection(CloudType::Ptr cloudp, Eigen::Vector3d& ctr,
				const int width, const int height, const double fl, const double radius,
				pcl::PointIndices& inlier_list);
// Write result to a file
void write2file(std::string file, std::vector<std::string>& names, std::vector<std::string>& types,
		std::vector<std::array<double,7> >& cams,std::vector<Eigen::Vector3d>& balls);
// Visualize Inlier
void showInliers(CloudType::ConstPtr cloud, std::vector<int>& inlier_list);
// Output the results to console
void print_results(std::vector<std::array<double,7> >& cams,std::vector<Eigen::Vector3d>& balls);


int main(int argc, char** argv)
{
  if (argc < 3)
    {
      PCL_ERROR("Usage: %s <config.yaml> <solution.yaml>\n", argv[0]);
      exit(1);
    }

  YAML::Node config = YAML::LoadFile(argv[1]);
  YAML::Node solved = YAML::LoadFile(argv[2]);

  const double target_radius    = config["target_ball_radius"].as<double>();
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();
  const int num_bg              = config["NumBackground"].as<int>();

  const YAML::Node& sensor_nd = solved["Sensors"];
  const YAML::Node& landmk_nd = solved["Landmarks"];

  // Set the static variable
  Euclidean3DError::RayAutoError::R = target_radius;

  // Ceres variables
  ceres::Problem prob;
  ceres::Solver::Options opt;  
  ceres::Solver::Summary summary;

  // Save all landmarks into a vector of Vector3d
  std::vector<Eigen::Vector3d> landmk_vc(landmk_nd.size());
  for (int j=0; j < landmk_vc.size(); j++)
    {
      landmk_vc[j] << static_cast<double>(landmk_nd[j][1].as<double>()),
	              static_cast<double>(landmk_nd[j][2].as<double>()),
	              static_cast<double>(landmk_nd[j][3].as<double>());
      std::cout << "V["<<j<<"]= " << landmk_vc[j].transpose()<<std::endl;
    }

  // Project the landmarks to each sensor frame
  std::vector<std::array<double, 7> > extrinsics(sensor_nd.size());
  std::vector<std::string> names(sensor_nd.size());
  std::vector<std::string> types(sensor_nd.size());
  for (int i=0; i < sensor_nd.size(); i++) // Outer loop goes through all sensors
    {
      types[i] = sensor_nd[i]["Type"].as<std::string>();
      names[i] = sensor_nd[i]["Name"].as<std::string>();

      std::cout<<"Sensor: " << names[i] <<" of type: "<< types[i] <<std::endl;

      const YAML::Node& origin_nd = sensor_nd[i]["Origin"];
      const YAML::Node& observ_nd = sensor_nd[i]["Observations"];
      // convert to t and q
      Eigen::Vector3d t;
      Eigen::Quaterniond q;

      std::array<double, 7>& ext = extrinsics[i];
      ext[0] = origin_nd[0].as<double>();
      ext[1] = origin_nd[1].as<double>();
      ext[2] = origin_nd[2].as<double>();
      ext[3] = origin_nd[3].as<double>();
      ext[4] = origin_nd[4].as<double>();
      ext[5] = origin_nd[5].as<double>();
      ext[6] = origin_nd[6].as<double>();

      t<< ext[0], ext[1], ext[2];
      q.w() = ext[3]; q.x() = ext[4]; q.y() = ext[5]; q.z() = ext[6];

      const double fl     = config["Sensors"][i]["focal_length"].as<double>();
      const int width     = config["Sensors"][i]["Width"].as<int>();
      const int height    = config["Sensors"][i]["Height"].as<int>();

      // Inner loop goes over all corresponding observations
      for (int j=0; j < landmk_vc.size(); j++)
	{
	  CloudType::Ptr cloudp(new CloudType);

	  // Load the PCD files
	  std::string filename = config["Sensors"][i]["PCDs"][j+num_bg].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *cloudp) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }
	  else
	    PCL_WARN("Loaded PCD %s\n", full_path.c_str());

	  // The detected sphere is at:
	  Eigen::Vector3d detect_sphere;
	  detect_sphere[0] = observ_nd[j][1].as<double>();
	  detect_sphere[1] = observ_nd[j][2].as<double>();
	  detect_sphere[2] = observ_nd[j][3].as<double>();

	  pcl::PointIndices inliers;

	  float f = types[i]=="sr" ? -fl : fl;
	  filterInliersFromDetection(cloudp, detect_sphere, width, height, f, target_radius, inliers);
	
	  // For convenience, use this instead
	  std::vector<int>& inlier_list = inliers.indices;
#if SHOW_INLIER
	  showInliers(cloudp, inlier_list);
#endif
	  
	  // Add the point to the optimization problem
	  
	  for (int k=0; k < inlier_list.size(); ++k)
	    {
	      PointType& p = cloudp->points[inlier_list[k]];
	      ceres::CostFunction* cf = new ceres::AutoDiffCostFunction<Euclidean3DError::RayAutoError,1,4,3,3>(new Euclidean3DError::RayAutoError(p.x, p.y, p.z));
	      //	      ceres::CostFunction* cf = new Euclidean3DError::RayCostError(p.x, p.y, p.z);
	      ceres::LossFunction* lf = new ceres::HuberLoss(1.0);
	      prob.AddResidualBlock(cf, lf, &(ext[3]), &(ext[0]), landmk_vc[j].data());
	    }
	}
      prob.SetParameterization(&(ext[3]), new ceres::QuaternionParameterization);
    }

  opt.max_num_iterations = 100;
  opt.function_tolerance = 1e-10;
  opt.num_threads = MAX_JACOBIAN_THREADS;
  opt.minimizer_progress_to_stdout = true;
  opt.linear_solver_type = ceres::DENSE_SCHUR;

  print_results(extrinsics, landmk_vc);
  ceres::Solve(opt, &prob, &summary);
  std::cout << summary.FullReport() << std::endl;

  print_results(extrinsics, landmk_vc);
  write2file("raysln.yaml", names, types, extrinsics, landmk_vc);
  return 0;
}

void filterInliersFromDetection(CloudType::Ptr cloudp, Eigen::Vector3d& ctr,
				const int width, const int height, const double fl, const double radius,
				pcl::PointIndices& inlier_list)
{
  std::vector<int>& inliers = inlier_list.indices;

  int u0 = width /2 + static_cast<int>(ctr(0) / ctr(2) * fl);
  int v0 = height/2 + static_cast<int>(ctr(1) / ctr(2) * fl);
  int ws = abs(2*radius / ctr(2) * fl);
 
  printf("Search window = %d, centered at <%d, %d>\n", ws, u0, v0);

  //  std::vector<int> inlier_list;
  for(int u=u0-ws; u<=u0+ws; ++u)
    {
      for(int v=v0-ws; v<=v0+ws; ++v)
	{
	  if (u<0 || u>=width || v<0 || v>=height)
	    continue;

	  // This condition ensures a circular shape
	  if (ws*ws < ((u-u0)*(u-u0)+(v-v0)*(v-v0)))
	    continue;

	  int linear_idx = v*width+u;
	  //	  printf("idx = %d (%d, %d)\n", linear_idx, u, v);
	  PointType& p = cloudp->points[linear_idx];
	  
	  double d = ((p.x-ctr(0))*(p.x-ctr(0)) +
		      (p.y-ctr(1))*(p.y-ctr(1)) +
		      (p.z-ctr(2))*(p.z-ctr(2)))/(radius*radius);
#define INLIER_THRESHOLD 0.9		  
	  if ( d < (1+INLIER_THRESHOLD) && d > (1-INLIER_THRESHOLD) )
	    inliers.push_back(linear_idx);
	}
    }
}
/*
void filterInliersFromProjection()
{
  // Where is the ball supposed to be at in the sensor frame?
  Eigen::Vector3d new_center = q.inverse()*(landmk_vc[j] - t);
  double ox = sensor_nd[i]["Observations"][j][1].as<double>();
  double oy = sensor_nd[i]["Observations"][j][2].as<double>();
  double oz = sensor_nd[i]["Observations"][j][3].as<double>();

  std::cout << "Detected Center at ["<< ox <<", "<< oy<<", "<< oz <<"]"<<std::endl;
  std::cout << "Projected Center at ["<< new_center.transpose() <<"]"<<std::endl;

  // TODO: Create a mask(inlier list) to select the new ball
  int u0 = width /2 + static_cast<int>(new_center(0) / new_center(2) * fl);
  int v0 = height/2 + static_cast<int>(new_center(1) / new_center(2) * fl);
  double ws = PERCENT_RADIUS*target_radius / new_center(2) * fl;
	  
  std::cout << "Search window = "<< ws << std::endl;

  std::vector<int> inlier_list;
  for(int u=u0-ws; u<=u0+ws; ++u)
    {
      for(int v=v0-ws; v<=v0+ws; ++v)
	{
	  if (u<0 || u>=width || v<0 || v>=height)
	    continue;

	  // This condition ensures a circular shape
	  if (ws*ws < ((u-u0)*(u-u0)+(v-v0)*(v-v0)))
	    continue;

	  int linear_idx = v*width+u;
	  PointType& p = cloudp->points[linear_idx];
		  
	  double d = ((p.x-new_center(0))*(p.x-new_center(0)) +
		      (p.y-new_center(1))*(p.y-new_center(1)) +
		      (p.z-new_center(2))*(p.z-new_center(2)))/(target_radius*target_radius);
		  
	  if ( d < (1+INLIER_THRESHOLD) && d > (1-INLIER_THRESHOLD) )
	    inlier_list.push_back(linear_idx);
	}
    }

  std::cout << inlier_list.size() << " inliers found with threshold of "<< INLIER_THRESHOLD <<std::endl;

  // The inlier number has to be greater than 0
  assert(inlier_list.size() > 0);
}
*/
void showInliers(CloudType::ConstPtr cloud, std::vector<int>& inlier_list)
{
  static pcl::visualization::PCLVisualizer viewer("3D Viewer");
  static bool first_time = true;

  CloudType::Ptr inlier_cloud(new CloudType), outlier_cloud(new CloudType);

  std::sort(inlier_list.begin(), inlier_list.end());

  // Filter points:
  inlier_cloud->points.reserve(inlier_list.size());
  outlier_cloud->points.reserve(cloud->size() - inlier_list.size());
  for(int i=0, j=0; i < cloud->points.size(); ++i)
    {
      if (i == inlier_list[j])
	{
	  inlier_cloud->points.push_back(cloud->points[i]);
	  ++j;
	}
      else
	outlier_cloud->points.push_back(cloud->points[i]);
    }

  // Set up a custom color handler
  pcl::visualization::PointCloudColorHandlerCustom<PointType> outlier_handler(outlier_cloud,   0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType>  inlier_handler( inlier_cloud, 255,   0, 0);
  
  if(first_time)
    {
      viewer.addPointCloud<PointType> (inlier_cloud, inlier_handler, "inlier_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inlier_cloud");
      viewer.addPointCloud<PointType> (outlier_cloud, outlier_handler, "outlier_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier_cloud");

      viewer.setBackgroundColor (0, 0, 0);

      viewer.addCoordinateSystem (0.05);
      viewer.initCameraParameters ();
      viewer.setCameraPose(0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);

      first_time = false;
    }
  else
    {
      viewer.updatePointCloud<pcl::PointXYZ>(inlier_cloud, inlier_handler, "inlier_cloud");
      viewer.updatePointCloud<pcl::PointXYZ>(outlier_cloud, outlier_handler, "outlier_cloud");
    }

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  viewer.resetStoppedFlag();
}

void print_results(std::vector<std::array<double,7> >& cams,
		   std::vector<Eigen::Vector3d>& balls)
{
  for(int i=0; i< cams.size(); i++)
    {
      std::array<double,7>& c = cams[i];
      PCL_WARN("Cam[%d] = [%7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf]\n",
	       c[0], c[1], c[2], c[3], c[4], c[5], c[6]);
    }

  for(int j=0; j< balls.size(); ++j)
    {
      Eigen::Vector3d& b = balls[j];
      PCL_WARN("Ball[%d] = [%7.4lf %7.4lf %7.4lf]\n", b(0), b(1), b(2));
    }
}

void write2file(std::string file, std::vector<std::string>& names, std::vector<std::string>& types,
		std::vector<std::array<double,7> >& cams,std::vector<Eigen::Vector3d>& balls)
{
  std::ofstream os(file);
  if (!os.is_open())
    {
      PCL_ERROR("Couldn't open file %s\n", file.c_str());
      return;
    }
  
  // First write all sensor information
  os << "Sensors:"<<std::endl << std::setprecision(9);
  for (int i=0; i < cams.size(); ++i)
    {
      os << "  - Type: " << types[i] << std::endl;
      os << "    Name: " << names[i] << std::endl;

      std::array<double,7>& c = cams[i];
      os << "    Origin: [" <<c[0]<<", "<<c[1]<<", "<<c[2]<<", "<<c[3]<<", "<<c[4]<<", "<<c[5]<<", "<<c[6]<<"]"<<std::endl;
      os << "    Observations:"<<std::endl;
      // convert to t and q
      Eigen::Vector3d t;
      Eigen::Quaterniond q;
      
      t<< c[0], c[1], c[2];
      q.w() = c[3]; q.x() = c[4]; q.y() = c[5]; q.z() = c[6];
      for (int j=0; j < balls.size(); ++j)
	{
	  Eigen::Vector3d p = q.inverse()*(balls[j] - t);
	  os << "      - ["<<j<<", "<<p(0)<<", "<<p(1)<<", "<<p(2)<<"]"<<std::endl; 
	}
    }

  // Then write out landmark information
  os << "Landmarks:" << std::endl;
  for (int j=0; j < balls.size(); ++j)
    {
      Eigen::Vector3d& p = balls[j];
      os << "  - ["<<j<<", "<<p(0)<<", "<<p(1)<<", "<<p(2)<<"]"<<std::endl;
    }

  os.close();
}
