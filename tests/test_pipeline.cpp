/*
  Test the processing pipeline
 */

#include "background_filter.hpp"
#include "morph_operator.hpp"
#include "blob_extractor.hpp"
#include "sphere_fitter.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

int main(int argc, char** argv)
{
  if (argc< 2)
    {
      printf("Usage: %s <config.yaml>\n", argv[0]);
      return -1;
    }
  // Load the config file
  YAML::Node config = YAML::LoadFile(argv[1]);
  
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();
  const double target_radius    = config["target_ball_radius"].as<double>();
  const float bg_threshold      = config["background_threshold"].as<float>();
  const float morph_radius      = config["morphological_radius"].as<float>();
  const int min_count           = config["cluster_min_count"].as<int>();
  const int max_count           = config["cluster_max_count"].as<int>();
  const double min_volumn       = config["cluster_min_volumn"].as<double>();
  const double max_volumn       = config["cluster_max_volumn"].as<double>();

  YAML::Node sensors = config["Sensors"];

  for (int i=0; i< sensors.size(); i++)
    {
      // Here are some sensor specific information:
      std::string name = sensors[i]["Name"].as<std::string>();
      std::string type = sensors[i]["Type"].as<std::string>();

      const float fl   = sensors[i]["focal_length"].as<float>();
      const int width  = sensors[i]["Width"].as<int>();
      const int height = sensors[i]["Height"].as<int>();

      const int num_bg = sensors[i]["NumBackground"].as<int>();
      const int num_fg = sensors[i]["NumForeground"].as<int>();

      // The mask is being used throughout the pipeline
      std::vector<char> mask;
      ImageFilter<PointType> img_filter(bg_threshold);
      
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
	  
	  // filter out background:
	  img_filter.GetForegroundMask(fg, mask);

	  // Morphological operations:
	  Morphology::Erode2_5D<PointType> (fg, mask, fl, morph_radius);
	  Morphology::Dilate2_5D<PointType>(fg, mask, fl, morph_radius);

	  // Extract clusters from binaryImage
	  std::vector<pcl::PointIndices> cluster_list;
	  OnePassBlobExtractor<PointType> obe(width, height, min_count, max_count, min_volumn, max_volumn);
	  obe.setInputCloud(fg);
	  obe.extract(cluster_list, mask);

	  // Find the cluster that is most likely to be a ball
	  double cost, best_cost = 1e5;
	  int best_idx = -1;
	  std::vector<Eigen::Vector3d> centers(cluster_list.size());
	  for(int i=0; i< cluster_list.size();++i)
	    {
	      LinearFitter<PointType> lsf(target_radius);
	      lsf.SetInputCloud(fg, cluster_list[i].indices);
	      cost = lsf.ComputeFitCost(centers[i]);
	      std::cout << "Center at "<< centers[i].transpose() <<" with cost = "<< cost<<std::endl;
	      if (cost < best_cost)
		{
		  best_cost = cost;
		  best_idx = i;
		}
	    }

	  // Nonlinear refinement
	  NonlinearFitter<PointType> nlsf(target_radius);
	  nlsf.SetInputCloud(fg, cluster_list[best_idx].indices);
	  // Notice this is a different kind of cost, not comparable to linear fit cost
	  cost = nlsf.ComputeFitCost(centers[best_idx]);

	  
	}
    }

  return 0;
}
