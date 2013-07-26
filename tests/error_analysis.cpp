/*
  Test sensor intrinsics
  Author: Ming Ruan
  Date: July 26 Fri
 */

#include "background_filter.hpp"
#include "morph_operator.hpp"
#include "blob_extractor.hpp"
#include "sphere_fitter.hpp"

#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

// Global configurations
string glob_prefix;   
string name;         
double target_radius;
int    num_bg;              
int    num_fg;             
float  bg_threshold; 
float  near_cutoff;     
float  far_cutoff;      
float  morph_radius; 
double min_volumn;     
double max_volumn;   
int    min_count;    
int    max_count;          
       		     	 
float  fl;           
int    width;               
int    height;              

void print_help(char* arg)
{
  PCL_WARN("Usage: %s <config>.yaml\n", arg);
}

int main(int argc, char** argv)
{
  target_radius = config["target_ball_radius"].as<double>();
  name          = config["Name"].as<std::string>();
  glob_prefix   = config["global_pcd_prefix"].as<std::string>();
  num_bg        = config["NumBackground"].as<int>();
  num_fg        = config["NumForeground"].as<int>();
  bg_threshold  = config["background_threshold"].as<float>();
  near_cutoff   = config["near_cutoff"].as<float>();
  far_cutoff    = config["far_cutoff"].as<float>();
  morph_radius  = config["morphological_radius"].as<float>();
  min_volumn    = config["cluster_min_volumn"].as<double>();
  max_volumn    = config["cluster_max_volumn"].as<double>();
  
  min_count     = config["cluster_min_count"].as<int>();
  max_count     = config["cluster_max_count"].as<int>();
  
  fl            = config["focal_length"].as<float>();
  width         = config["Width"].as<int>();
  height;       = config["Height"].as<int>();

  return 0;
}

void build_background_model(const YAML::Node& pcd_node, ImageFilter<PointType>& filter)
{
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
      filter.AddBackgroundCloud(bg);
    }
}

void extraction_pipeline(CloudType::ConstPtr fg, ImageFilter<PointType>& filter, 
			 Eigen::Vector3d& best_center, pcl::PointIndices& inlier_list)
{
  std::vector<char> mask;
  // filter out background:
  filter.GetForegroundMask(fg, mask);

  // apply opening operation
  Morphology::Erode2_5D<PointType> (fg, mask, fl, morph_radius);
  Morphology::Dilate2_5D<PointType>(fg, mask, fl, morph_radius);

  // extract cluster
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
  best_center = centers[best_idx];
  inlier_list = cluster_list[best_idx];
}

void test_individual_detection(const YAML::Node& pcd_node)
{
  // Load background
  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);
 
  // Build background model
  build_background_model(pcd_node, img_filter);

  // Find sphere in each frames
  for(int i=0; i < num_fg; ++i)
    {
      // Load a new frame
      CloudType::Ptr fg(new CloudType);
	  
      std::string filename = pcd_node[i+num_bg].as<std::string>();
      std::string full_path= glob_prefix + filename;
      if (pcl::io::loadPCDFile<PointType> (full_path, *fg) == -1)
	{
	  PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	  exit(1);
	}	 

      // Extract the sphere from pointcloud
      Eigen::Vector3d centroid;
      pcl::PointIndices inlier_list;
      extraction_pipeline(fg, image_filter, centroid, inlier_list);

      // Nonlinear refinement
      NonlinearFitter<PointType> nlsf(target_radius);
      nlsf.SetInputCloud(fg, inlier_list.indices);
      // Notice this is a different kind of cost, not comparable to linear fit cost
      cost = nlsf.ComputeFitCost(centroid);
      
      // print the result to std::cout
      printf("%lf %lf %lf\n", centroid(0), centroid(1), centroid(2));
    }
}

void test_composite_detection()
{
  // Load background
  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);
 
  // Build background model
  build_background_model(pcd_node, img_filter);

  // Find sphere in each frames
  NonlinearFitter<PointType> nlsf(target_radius); // for sphere fitting later on.
  Eigen::Vector3d centroid; // same as nlsf;

  for(int i=0; i < num_fg; ++i)
    {
      for(int j=0; j < num_frames; ++j)
	{
	  // Load a new frame
	  CloudType::Ptr fg(new CloudType);

	  std::string filename = pcd_node[i*num_frames+num_bg+j].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *fg) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }

	  // Extract the sphere from pointcloud
	  pcl::PointIndices inlier_list;
	  extraction_pipeline(fg, image_filter, centroid, inlier_list);

	  // Add inliers to cloud
	  nlsf.SetInputCloud(fg, inlier_list.indices);
	}

      // fit sphere Nonlinear refinement      
      // Notice this is a different kind of cost, not comparable to linear fit cost
      cost = nlsf.ComputeFitCost(centroid);
      
      // print the result to std::cout
      printf("%lf %lf %lf\n", centroid(0), centroid(1), centroid(2));
    }
}

void test_average_detection()
{
  // Load background
  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);
 
  // Build background model
  build_background_model(pcd_node, img_filter);

  // Find sphere in each frames
  for(int i=0; i < num_fg; ++i)
    {
      CloudType::Ptr fg(new CloudType);
      fg->width = width;
      fg->height= height;
      fg->points.resize(width*height);
      std::fill(fg->points.begin(), fg->points.end(), 0.0f);
      for(int j=0; j < aggr_frames; ++j)
	{
	  // Load a new frame
	  CloudType::Ptr temp(new CloudType);

	  std::string filename = pcd_node[i*aggr_frames+num_bg+j].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *temp) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }

	  // Aggregate

	}

      // Average over rays

      // Extract the sphere from pointcloud
      Eigen::Vector3d centroid;
      pcl::PointIndices inlier_list;
      extraction_pipeline(fg, image_filter, centroid, inlier_list);
      
      // fit sphere Nonlinear refinement
      NonlinearFitter<PointType> nlsf(target_radius);
      nlsf.SetInputCloud(fg, inlier_list.indices);
      // Notice this is a different kind of cost, not comparable to linear fit cost
      cost = nlsf.ComputeFitCost(centroid);
      
      // print the result to std::cout
      printf("%lf %lf %lf\n", centroid(0), centroid(1), centroid(2));
    }
}
