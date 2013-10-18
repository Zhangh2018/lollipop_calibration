/*
  Detect the balls for a single camera
 */

#include "background_filter.hpp"
#include "morph_operator.hpp"
#include "blob_extractor.hpp"
#include "sphere_fitter.hpp"

#include <fstream>
#include <sstream>
#include <array>

#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>

using namespace std;
using namespace pcl::visualization;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

boost::shared_ptr<PCLVisualizer> viewer;
static bool g_exit_flag = false;


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  if (event.getKeyCode() == 'v' && event.keyDown ())
  {
    printf("v pressed");
    viewer->setCameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
  }
  if (event.getKeyCode() == 27 && event.keyDown ())
    {
      viewer->close();
      g_exit_flag = true;
    }
}

void morph_open(CloudType::Ptr fg, ImageFilter<PointType>& img_filter,
		std::vector<char>& mask, float fl, float morph_radius, bool debug)
{

}

void showfittedSphere(CloudType::Ptr cloud, pcl::PointIndices& pi, pcl::ModelCoefficients& coeff)
{
  static bool first_time = true;

  CloudType::Ptr inlier_cloud(new CloudType), outlier_cloud(new CloudType);

  if(pi.indices.size() > 0)
    {
      std::sort(pi.indices.begin(), pi.indices.end());

      // Filter points:
      inlier_cloud->points.reserve(pi.indices.size());
      outlier_cloud->points.reserve(cloud->size() - pi.indices.size());
      for(int i=0, j=0; i < cloud->points.size(); ++i)
	{
	  if (i == pi.indices[j])
	    {
	      inlier_cloud->points.push_back(cloud->points[i]);
	      ++j;
	    }
	  else
	    outlier_cloud->points.push_back(cloud->points[i]);
	}
    }
  else
    outlier_cloud = cloud;

  // Set up a custom color handler
  PointCloudColorHandlerCustom<PointType> outlier_handler(outlier_cloud,   0, 255, 0);
  PointCloudColorHandlerCustom<PointType>  inlier_handler( inlier_cloud, 255,   0, 0);

  if(!first_time)
    {
      viewer->removePointCloud("inlier_cloud");
      viewer->removePointCloud("outlier_cloud");
      //      viewer->updatePointCloud<pcl::PointXYZ>(inlier_cloud, inlier_handler, "inlier_cloud");
      //     viewer->updatePointCloud<pcl::PointXYZ>(outlier_cloud, outlier_handler, "outlier_cloud");
      //           viewer.updateSphere(pcl::PointXYZ(x,y,z), 0.1275, 1.0, 1.0, 0.0);
      viewer->removeShape("sphere");
      //      viewer->addSphere(coeff, "sphere");
    }
  else
    first_time = false;

  viewer->addPointCloud<PointType> (inlier_cloud, inlier_handler, "inlier_cloud");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "inlier_cloud");
  viewer->addPointCloud<PointType> (outlier_cloud, outlier_handler, "outlier_cloud");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "outlier_cloud");
  viewer->addSphere(coeff, "sphere");      
}

void remask_inliers_ellipse(Eigen::Vector3d& ctr, const int width, const int height, 
			    const double fl, const double target_radius, pcl::PointIndices& inliers)
{
  double d = ctr.norm();
  double k2= target_radius*target_radius/(d*d-target_radius*target_radius);

  int u0 = width/2;
  int v0 = height/2;

  int u = ctr(0)*fl/ctr(2);
  int v = ctr(1)*fl/ctr(2);

  double A = 1.0+(v*v-k2*u*u)/(fl*fl);
  double B = 1.0+(u*u-k2*v*v)/(fl*fl);
  double C2= -2*(1+k2)*u*v/(fl*fl);
  double D2= -2*(1+k2)*u;
  double E2= -2*(1+k2)*v;
  double F = (u*u+v*v-k2*fl*fl);

  int x, y;
  for(int i=0; i< height; ++i)
    {
      for(int j=0; j<width; j++)
	{
	  x = j - u0;
	  y = i - v0;

	  if(A*x*x+B*y*y+C2*x*y+D2*x+E2*y+F < 1.0)
	    {
	      int linear_idx = i*width+j;
	      inliers.indices.push_back(linear_idx);
	    }
	}
    }
}

void remask_inliers(CloudType::Ptr cloudp, Eigen::Vector3d& ctr, pcl::PointIndices& inliers,
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
	  d = abs(d-target_radius);
#define INLIER_THRESHOLD 0.99	  
	  if ( d > 0.5*target_radius && d < 2.0*target_radius )
	    inliers.indices.push_back(linear_idx);
	  //	  printf("add new inlier\n");
	}
    }
}


int main(int argc, char** argv)
{
  if (argc< 2)
    {
      printf("Usage: %s <config.yaml>\n", argv[0]);
      PCL_WARN("Options:\n");
      PCL_WARN(" -debug [0|1 defualt=0] \n");
      PCL_WARN(" -morph [0|1 default=1] \n");
      return -1;
    }

  bool debug = 0;
  pcl::console::parse_argument(argc, argv, "-debug", debug);

  bool morph = 1;
  pcl::console::parse_argument(argc, argv, "-morph", morph);

  viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
  //      viewer.addCoordinateSystem (0.05);
  viewer->initCameraParameters();
  //  viewer->setCameraFieldOfView(50);
  viewer->setCameraPose(0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
  viewer->registerKeyboardCallback (keyboardEventOccurred);
  
  // Load the config file
  YAML::Node config = YAML::LoadFile(argv[1]);
  
  const double target_radius    = config["target_ball_radius"].as<double>();
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();
  const int num_bg              = config["NumBackground"].as<int>();
  const float bg_threshold      = config["background_threshold"].as<float>();
  const float near_cutoff       = config["near_cutoff"].as<float>();
  const float far_cutoff        = config["far_cutoff"].as<float>();
  const float morph_radius      = config["morphological_radius"].as<float>();
  const double min_volumn       = config["cluster_min_volumn"].as<double>();
  const double max_volumn       = config["cluster_max_volumn"].as<double>();

  const YAML::Node& sensors = config["Sensors"];

  int i=0;
  // Here are some sensor specific information:
  std::string name    = sensors[i]["Name"].as<std::string>();
  std::string type    = sensors[i]["Type"].as<std::string>();

  const float fl      = sensors[i]["focal_length"].as<float>();
  const int width     = sensors[i]["Width"].as<int>();
  const int height    = sensors[i]["Height"].as<int>();

  const int min_count = sensors[i]["cluster_min_count"].as<int>();
  const int max_count = sensors[i]["cluster_max_count"].as<int>();
 
  // The mask is being used throughout the pipeline
  std::vector<char> mask;
  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);

  const YAML::Node& pcd_node = sensors[i]["PCDs"];
  // Load the background models
  for (int j=0; j < num_bg; ++j)
    {
      CloudType::Ptr bg (new CloudType);
      std::string filename = pcd_node[j].as<std::string>();
	  
      std::string full_path= glob_prefix + filename;
      if (pcl::io::loadPCDFile<PointType> (full_path, *bg) == -1)
	{
	  PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	  exit(1);
	}
      img_filter.AddBackgroundCloud(bg);
    }

  // Process each foreground pointcloud:
  // 1. Subtract background
  // 2. Apply morphological operator to remove isolate each cluster
  // 3. Find the center of the sphere from pointcloud
  const int num_fg = pcd_node.size() - num_bg;
  for (int j=0; j < num_fg; ++j)
    {
      CloudType::Ptr fg(new CloudType);
	  
      std::string filename = pcd_node[j+num_bg].as<std::string>();
      std::string full_path= glob_prefix + filename;
      if (pcl::io::loadPCDFile<PointType> (full_path, *fg) == -1)
	{
	  PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	  exit(1);
	}

      PCL_WARN("\nProcessing frame[%d] (%s)\n", j, filename.c_str());

      // filter out background:
      img_filter.GetForegroundMask(fg, mask);

      // morphological operator: operation on mask
      morph_open(fg, img_filter, mask, fl, morph_radius, debug);
      if(debug)
	img_filter.WriteMaskToFile("mask.dump", mask);

      if(morph)
	{
	  // Morphological operations:
	  Morphology::Erode2_5D<PointType> (fg, mask, fl, morph_radius);
  
	  if(debug)
	    img_filter.WriteMaskToFile("erode.dump", mask);
	  
	  Morphology::Dilate2_5D<PointType>(fg, mask, fl, morph_radius);
	  
	  if(debug)
	    img_filter.WriteMaskToFile("final.dump", mask);
	}

      // Extract clusters from binaryImage
      std::vector<pcl::PointIndices> cluster_list;
      FastBlobExtractor<PointType> extractor(width, height, min_count, max_count, min_volumn, max_volumn);
      extractor.setInputCloud(fg);
      extractor.extract(cluster_list, mask);

      pcl::ModelCoefficients coeff;
      if(cluster_list.empty())
	{
	  coeff.values.push_back(0.0f);   
	  coeff.values.push_back(0.0f); 
	  coeff.values.push_back(0.0f);	
	  coeff.values.push_back(0.0f);   
	  pcl::PointIndices empty_idx;
	  showfittedSphere(fg, empty_idx, coeff);
	}
      else // We might find a potential sphere:
	{
	  // Find the cluster that is most likely to be a ball
	  double cost, best_cost = 1e5;
	  int best_idx = -1;
	  std::vector<Eigen::Vector3d> centers(cluster_list.size());
	  for(int t=0; t< cluster_list.size();++t)
	    {
	      if(cluster_list[t].indices.empty())
		continue;

	      LinearFitter<PointType> lsf(target_radius);
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
	  NonlinearFitter<PointType> nlsf(target_radius);
	  nlsf.SetInputCloud(fg, cluster_list[best_idx].indices);
	  // Notice this is a different kind of cost, not comparable to linear fit cost
	  cost = nlsf.ComputeFitCost(centers[best_idx]);
	  
	  pcl::PointIndices& inlier_idx = cluster_list[best_idx];
	  //	  remask_inliers_ellipse(centers[best_idx], width, height, -fl, target_radius, inlier_idx);
	  //remask_inliers(fg, centers[best_idx], inlier_idx, width, height, fl, target_radius);
	  
	  nlsf.Clear();//reset the cost function
	  nlsf.SetInputCloud(fg, inlier_idx.indices);
	  cost = nlsf.ComputeFitCost(centers[best_idx]);
	  //	  */

	  Eigen::Vector3d& best_ctr = centers[best_idx];
	  coeff.values.push_back(best_ctr(0));
	  coeff.values.push_back(best_ctr(1));
	  coeff.values.push_back(best_ctr(2));
	  coeff.values.push_back(target_radius);

	  showfittedSphere(fg, inlier_idx, coeff);
	  // showfittedSphere(fg, cluster_list[best_idx], coeff);
	}

      while(!viewer->wasStopped())
	{
	  viewer->spinOnce(100);
	  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
      if(g_exit_flag)
	break;
      viewer->resetStoppedFlag();	  
    }
  return 0;	 
}
