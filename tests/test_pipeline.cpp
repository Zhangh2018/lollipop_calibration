/*
  Test the processing pipeline
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
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

void showfittedSphere(CloudType::Ptr cloud, double x, double y, double z);
void debug_save_clusters(CloudType::Ptr cloud, std::vector<pcl::PointIndices>& cluster_list);

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
  const int num_bg              = config["NumBackground"].as<int>();
  const int num_fg              = config["NumForeground"].as<int>();
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

      // The mask is being used throughout the pipeline
      std::vector<char> mask;
      ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);
      
      // Load the background models
      for (int j=0; j < num_bg; ++j)
	{
	  CloudType::Ptr bg (new CloudType);
	  std::string filename = sensors[i]["PCDs"][j].as<std::string>();
	  
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
      for (int j=0; j < num_fg; ++j)
	{
	  CloudType::Ptr fg(new CloudType);
	  
	  std::string filename = sensors[i]["PCDs"][j+num_bg].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *fg) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }
	 
	  PCL_WARN("\nProcessing Sensor[%d], frame[%d] (%s)\n", i, j, filename.c_str()); 
	  // filter out background:
	  img_filter.GetForegroundMask(fg, mask);

	  //	  img_filter.WriteMaskToFile("mask.dump", mask);
	 
	  // Morphological operations:
	  Morphology::Erode2_5D<PointType> (fg, mask, fl, morph_radius);

	  //	  img_filter.WriteMaskToFile("erode.dump", mask);

	  Morphology::Dilate2_5D<PointType>(fg, mask, fl, morph_radius);

	  //	  img_filter.WriteMaskToFile("dilate.dump", mask);

	  // Extract clusters from binaryImage
	  std::vector<pcl::PointIndices> cluster_list;
	  OnePassBlobExtractor<PointType> obe(width, height, min_count, max_count, min_volumn, max_volumn);
	  obe.setInputCloud(fg);
	  obe.extract(cluster_list, mask);

	  assert(!cluster_list.empty());
	  //	  debug_save_clusters(fg, cluster_list);

	  // Find the cluster that is most likely to be a ball
	  double cost, best_cost = 1e5;
	  int best_idx = -1;
	  std::vector<Eigen::Vector3d> centers(cluster_list.size());
	  for(int k=0; k< cluster_list.size();++k)
	    {
	      LinearFitter<PointType> lsf(target_radius);
	      lsf.SetInputCloud(fg, cluster_list[k].indices);
	      cost = lsf.ComputeFitCost(centers[k]);
	      std::cout << "Center at "<< centers[k].transpose() <<" with cost = "<< cost<<std::endl;
	      if (cost < best_cost)
		{
		  best_cost = cost;
		  best_idx = k;
		}
	    }

	  // Nonlinear refinement
	  NonlinearFitter<PointType> nlsf(target_radius);
	  nlsf.SetInputCloud(fg, cluster_list[best_idx].indices);
	  // Notice this is a different kind of cost, not comparable to linear fit cost
	  cost = nlsf.ComputeFitCost(centers[best_idx]);

	  Eigen::Vector3d& best_ctr = centers[best_idx];
	  os << "      - ["<<j<<", "<<best_ctr(0)<<", "<<best_ctr(1)<<", "<<best_ctr(2)<<"]"<<std::endl;
	  
	  //	  showfittedSphere(fg, best_ctr(0), best_ctr(1), best_ctr(2));
	  
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

void showfittedSphere(CloudType::Ptr cloud, double x, double y, double z)
{
  static pcl::visualization::PCLVisualizer viewer("3D Viewer");
  static int i=0;

  viewer.setBackgroundColor (0, 0, 0);

  if(i==0)
    {
      viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
      viewer.addSphere(pcl::PointXYZ(x, y, z), 0.1275, 1.0, 1.0, 0.0);
    }
  else
    {
      viewer.updatePointCloud<pcl::PointXYZ>(cloud, "sample cloud");
      viewer.updateSphere(pcl::PointXYZ(x,y,z), 0.1275, 1.0, 1.0, 0.0);
    }

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.addCoordinateSystem (0.05);
  viewer.initCameraParameters ();

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  viewer.resetStoppedFlag();
  ++i;
}
