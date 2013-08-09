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
#include <pcl/console/parse.h>

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

void test_individual_detection(const YAML::Node& pcd_node, std::ostream& os);
void test_composite_detection(const YAML::Node& pcd_node, std::ostream& os, int num_frames);
void test_aggregate_detection(const YAML::Node& pcd_node, std::ostream& os, int aggr_frames);

int main(int argc, char** argv)
{
  bool display_help_flag = pcl::console::find_switch(argc, argv, "-h");
  if( display_help_flag || argc < 2)
    {
      PCL_WARN("Usage: %s <config>.yaml [-Options]\n", argv[0]);
      PCL_WARN("Options: -i:  Individual Mode\n");
      PCL_WARN("         -c:  Composite Mode [-f composite frame number, default 8]\n");
      PCL_WARN("         -a:  Aggregate Mode [-f aggregate frame number, defualt 8]\n");
      return 0;
    }

  YAML::Node config = YAML::LoadFile(argv[1]);
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
  height        = config["Height"].as<int>();

  std::ofstream os;
  std::string config_file(argv[1]);
  unsigned found = config_file.find_last_of(".");
  if( pcl::console::find_switch(argc, argv, "-i") )
    {
      os.open(config_file.substr(0, found)+"_ind.out");
      test_individual_detection(config["PCDs"], os);
    }
  else
    {
      int num_frame = 8;
      pcl::console::parse_argument(argc, argv, "-f", num_frame);
      if ( pcl::console::find_switch(argc, argv, "-c") )
	{
	  os.open(config_file.substr(0, found)+"_com.out");
	  test_composite_detection(config["PCDs"], os, num_frame);
	}
      else if ( pcl::console::find_switch(argc, argv, "-a") )
	{
	  os.open(config_file.substr(0, found)+"_agg.out");
	  test_aggregate_detection(config["PCDs"], os, num_frame);
	}
      else
	PCL_ERROR("Unknown command sequence\n");
    }

  os.close();
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

void extraction_pipeline(CloudType::Ptr fg, ImageFilter<PointType>& filter, 
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

void test_individual_detection(const YAML::Node& pcd_node, std::ostream& os)
{
  // Load background
  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);
 
  // Build background model
  build_background_model(pcd_node, img_filter);

  // Find sphere in each frames
  for(int i=0; i < num_fg; ++i)
    {
      PCL_WARN("Processing frame %d/%d\n", i, num_fg);
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
      extraction_pipeline(fg, img_filter, centroid, inlier_list);

      // Nonlinear refinement
      NonlinearFitter<PointType> nlsf(target_radius);
      nlsf.SetInputCloud(fg, inlier_list.indices);
      // Notice this is a different kind of cost, not comparable to linear fit cost
      double cost = nlsf.ComputeFitCost(centroid);
      
      // print the result to output stream
      os << centroid(0)<<" "<< centroid(1)<<" "<< centroid(2) << std::endl;
    }
}

void test_composite_detection(const YAML::Node& pcd_node, std::ostream& os, int num_frames)
{
  // Load background
  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);
 
  // Build background model
  build_background_model(pcd_node, img_filter);

  // Find sphere in each frames
  Eigen::Vector3d centroid; // same as nlsf;

  for(int i=0; i < num_fg; i+= num_frames)
    {
      PCL_WARN("Processing frame %d/%d\n", i, num_fg);
      NonlinearFitter<PointType> nlsf(target_radius); // for sphere fitting later on.
      for(int j=0; j < num_frames; ++j)
	{
	  // Load a new frame
	  CloudType::Ptr fg(new CloudType);

	  std::string filename = pcd_node[num_bg+i+j].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *fg) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }

	  // Extract the sphere from pointcloud
	  pcl::PointIndices inlier_list;
	  extraction_pipeline(fg, img_filter, centroid, inlier_list);

	  // Add inliers to cloud
	  nlsf.SetInputCloud(fg, inlier_list.indices);
	}

      // fit sphere Nonlinear refinement      
      // Notice this is a different kind of cost, not comparable to linear fit cost
      double cost = nlsf.ComputeFitCost(centroid);
      
      // print the result to output stream
      os << centroid(0)<<" "<< centroid(1)<<" "<< centroid(2) << std::endl;
    }
}

bool myfunction (const PointType& i, const PointType& j) { return (i.z<j.z); }
void test_aggregate_detection(const YAML::Node& pcd_node, std::ostream& os, int aggr_frames)
{
  int lower_quantile = aggr_frames/4;
  int upper_quantile = aggr_frames - lower_quantile;
  int median_quantile= upper_quantile - lower_quantile;

  std::vector<CloudType> aggr(aggr_frames);
 
  // Load background
  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);
 
  // Build background model
  build_background_model(pcd_node, img_filter);

  // Find sphere in each frames
  CloudType::Ptr fg(new CloudType);
  fg->width = width;
  fg->height= height;
  fg->points.resize(width*height);
  for(int i=0; i < num_fg; i+= aggr_frames)
    {
      PCL_WARN("Processing frame %d/%d\n", i, num_fg);
      for(int j=0; j < aggr_frames; ++j)
	{
	  // Load a new frame
	  std::string filename = pcd_node[num_bg+i+j].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, aggr[j]) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }
	}
 
      // Average over rays
      std::vector<PointType> stats(aggr_frames);
      for(int j=0; j < aggr[0].size(); ++j)
	{
	  for(int k=0; k < aggr_frames; ++k)
	    {
	      const PointType& p = aggr[k].points[j];
	      stats[k] = p;
	      if (isnan(p.z))
		stats[k].z = 1e2;
	    }
	  std::sort(stats.begin(), stats.end(), myfunction);

	  float x=.0f, y=.0f, z=.0f;
	  for(int k=lower_quantile; k < upper_quantile; ++k)
	    {
	      x+=stats[k].x; y+=stats[k].y; z+=stats[k].z;
	    }
	  PointType q(x/median_quantile, y/median_quantile, z/median_quantile);
	  fg->points[j] = q;
	}

      // Extract the sphere from pointcloud
      Eigen::Vector3d centroid;
      pcl::PointIndices inlier_list;
      extraction_pipeline(fg, img_filter, centroid, inlier_list);
      
      // fit sphere Nonlinear refinement
      NonlinearFitter<PointType> nlsf(target_radius);
      nlsf.SetInputCloud(fg, inlier_list.indices);
      // Notice this is a different kind of cost, not comparable to linear fit cost
      double cost = nlsf.ComputeFitCost(centroid);
      
      // print the result to output stream
      os << centroid(0)<<" "<< centroid(1)<<" "<< centroid(2) << std::endl;

    }
}

