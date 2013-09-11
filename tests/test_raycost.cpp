
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
#include <pcl/ModelCoefficients.h>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

int main()
{
  if (argc< 4)
    {
      printf("Usage: %s <config.yaml> <in_solution.yaml> <out_solution.yaml>\n", argv[0]);
      return -1;
    }
  // Open the output file
  std::ofstream os(argv[2]);

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
	 
	  PCL_WARN("\nProcessing Sensor[%d], frame[%d] (%s)\n", i, j, filename.c_str()); 
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

	  assert(!cluster_list.empty());

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

#if USE_NONLINEAR_REFINEMENT
	  // Nonlinear refinement
	  NonlinearFitter<PointType> nlsf(target_radius);
	  nlsf.SetInputCloud(fg, cluster_list[best_idx].indices);
	  // Notice this is a different kind of cost, not comparable to linear fit cost
	  cost = nlsf.ComputeFitCost(centers[best_idx]);
#endif

	  Eigen::Vector3d& best_ctr = centers[best_idx];
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
