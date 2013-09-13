/*
  Test the processing pipeline
 */
#include <fstream>
#include <sstream>
#include <array>

#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>

#include "generic_sensor.hpp"

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

#define DEBUG_MODE 0   // Set 1 to enable saving the morphological masks
#define SHOW_FIT   0   // Set 1 to display the fitted sphere
#define USE_NONLINEAR_REFINEMENT 1 // Set 1 to use nonlinear refinement for sphere fitting

//void showfittedSphere(CloudType::ConstPtr cloud, pcl::PointIndices& pi, pcl::ModelCoefficients& coeff);
//void debug_save_clusters(CloudType::Ptr cloud, std::vector<pcl::PointIndices>& cluster_list);

int main(int argc, char** argv)
{
  if (argc< 3)
    {
      printf("Usage: %s <config.yaml> <output.yaml>\n", argv[0]);
      return -1;
    }
  // Open the output file
  std::ofstream os(argv[2]);

  // Load the config file
  YAML::Node config = YAML::LoadFile(argv[1]);
  
  const double target_radius    = config["target_ball_radius"].as<double>();
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();

  const float bg_threshold      = config["background_threshold"].as<float>();
  const float near_cutoff       = config["near_cutoff"].as<float>();
  const float far_cutoff        = config["far_cutoff"].as<float>();
  const float morph_radius      = config["morphological_radius"].as<float>();
  const double min_volumn       = config["cluster_min_volumn"].as<double>();
  const double max_volumn       = config["cluster_max_volumn"].as<double>();

  const YAML::Node& sensors = config["Sensors"];

  std::vector<std::array<double,3> > landmarks;
  os << "Sensors:"<< std::endl;
  for (int i=0; i< sensors.size(); i++)
    {
      // Here are some sensor specific information:
      std::string name    = sensors[i]["Name"].as<std::string>();
      std::string type    = sensors[i]["Type"].as<std::string>();

      const float fl      = sensors[i]["focal_length"].as<float>();
      const int width     = sensors[i]["Width"].as<int>();
      const int height    = sensors[i]["Height"].as<int>();

      const int min_count = sensors[i]["cluster_min_count"].as<int>();
      const int max_count = sensors[i]["cluster_max_count"].as<int>();
 
      os <<"  - Type: " << type << std::endl;
      os <<"    Name: " << name << std::endl;
      os <<"    Origin: [0, 0, 0, 1, 0, 0, 0]" << std::endl;
      os <<"    Observations:" << std::endl;

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
      // 1. Subtract background
      // 2. Apply morphological operator to remove isolate each cluster
      // 3. Find the center of the sphere from pointcloud
      for (int j = 1; j < pcd_node.size(); ++j)
	{
	  CloudType::Ptr fg(new CloudType);
	  
	  std::string filename = pcd_node[j].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *fg) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }
	 
	  PCL_WARN("\nProcessing Sensor[%d], frame[%d] (%s)\n", i, j, filename.c_str()); 

	  // Main workload of the program
	  Eigen::Vector3d best_ctr;
	  std::vector<int> inlier_idx;
	  bool flag = sp->RunPipeline((void*) &fg, (void*) &bg, (void*) opt, 
				      inlier_idx, best_ctr);

	  os << "      - ["<<j<<", "<<best_ctr(0)<<", "<<best_ctr(1)<<", "<<best_ctr(2)<<"]"<<std::endl;

	  // Save the first sensor's measurement as landmarks
	  if (i==0)
	    {
	      std::array<double, 3> ar;
	      ar[0] = best_ctr(0); ar[1] = best_ctr(1); ar[2] = best_ctr(2);
	      landmarks.push_back(ar);
	    }
	}
    }

  os << "Landmarks:" << std::endl;
  for(int i=0; i< landmarks.size(); ++i)
    {
      std::array<double, 3>& ar = landmarks[i];
      os << "  - [" << i<<", "<<ar[0]<<", "<<ar[1]<<", "<<ar[2]<<"]"<<std::endl;
    }

  os.close();
  return 0;
}

void debug_save_clusters(CloudType::Ptr cloud, std::vector<pcl::PointIndices>& cluster_list)
{
  for(int i=0; i< cluster_list.size(); ++i)
    {
      CloudType::Ptr temp(new CloudType);
      std::vector<int>& idx = cluster_list[i].indices;
      temp->width = idx.size();
      temp->height= 1;
      temp->points.resize(idx.size());

      for(int j=0; j< idx.size(); ++j)
	{
	  temp->points[j] = cloud->points[idx[j]];
	}

      std::stringstream ss;
      ss << "db_" << i << ".pcd";
      pcl::io::savePCDFileASCII(ss.str(), *temp);
    }
}
/*
void showfittedSphere(CloudType::ConstPtr cloud, pcl::PointIndices& pi, pcl::ModelCoefficients& coeff)
{
  static pcl::visualization::PCLVisualizer viewer("3D Viewer");
  static bool first_time = true;

  CloudType::Ptr inlier_cloud(new CloudType), outlier_cloud(new CloudType);

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

  // Set up a custom color handler
  pcl::visualization::PointCloudColorHandlerCustom<PointType> outlier_handler(outlier_cloud,   0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType>  inlier_handler( inlier_cloud, 255,   0, 0);
  
  if(first_time)
    {
      viewer.addPointCloud<PointType> (inlier_cloud, inlier_handler, "inlier_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inlier_cloud");
      viewer.addPointCloud<PointType> (outlier_cloud, outlier_handler, "outlier_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier_cloud");

      //     viewer.addSphere(pcl::PointXYZ(x, y, z), 0.1275, 1.0, 1.0, 0.0);
      viewer.addSphere(coeff, "sphere");
  
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
      //           viewer.updateSphere(pcl::PointXYZ(x,y,z), 0.1275, 1.0, 1.0, 0.0);
      viewer.removeShape("sphere");
      viewer.addSphere(coeff, "sphere");
    }

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  viewer.resetStoppedFlag();
}
*/
