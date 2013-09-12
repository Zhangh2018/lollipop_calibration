/*
  Detect the balls for a single camera
 */

#include "generic_sensor.hpp"

#include <fstream>
#include <sstream>
#include <array>

#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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

void showfittedSphere(CloudType::Ptr cloud, std::vector<int>& pi, pcl::ModelCoefficients& coeff)
{
  static bool first_time = true;

  CloudType::Ptr inlier_cloud(new CloudType), outlier_cloud(new CloudType);

  if(pi.size() > 0)
    {
      std::sort(pi.begin(), pi.end());

      // Filter points:
      inlier_cloud->points.reserve(pi.size());
      outlier_cloud->points.reserve(cloud->size() - pi.size());
      for(int i=0, j=0; i < cloud->points.size(); ++i)
	{
	  if (i == pi[j])
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
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 3, "inlier_cloud");
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

int main(int argc, char** argv)
{
  if (argc< 2)
    {
      printf("Usage: %s <config.yaml>\n", argv[0]);
      return -1;
    }

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

  boost::shared_ptr<Sensor> sp = RangeSensor::Create(name, type);
  RangeSensorOptions* opt = new RangeSensorOptions(fl, width, height, bg_threshold, near_cutoff, far_cutoff,
						   morph_radius, target_radius, min_volumn, max_volumn,
						   min_count, max_count);

  const YAML::Node& pcd_node = sensors[i]["PCDs"];
  // Load the background models
  CloudType::Ptr bg (new CloudType);
  std::string filename = pcd_node[0].as<std::string>();
	  
  std::string full_path= glob_prefix + filename;
  if (pcl::io::loadPCDFile<PointType> (full_path, *bg) == -1)
    {
      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
      exit(1);
    }
  
  // Process each foreground pointcloud:
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

      // Main workload of the program
      Eigen::Vector3d center;
      std::vector<int> inlier_idx;
      bool flag = sp->RunPipeline((void*) &fg, (void*) &bg, (void*) opt, 
				  inlier_idx, center);
      
      pcl::ModelCoefficients coeff;
      if(!flag)
	{
	  coeff.values.push_back(0.0f);   
	  coeff.values.push_back(0.0f); 
	  coeff.values.push_back(0.0f);	
	  coeff.values.push_back(0.0f);   
	}
      else // We might find a potential sphere:
	{
	  coeff.values.push_back(center(0));
	  coeff.values.push_back(center(1));
	  coeff.values.push_back(center(2));
	  coeff.values.push_back(target_radius);
	}

      showfittedSphere(fg, inlier_idx, coeff);
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
