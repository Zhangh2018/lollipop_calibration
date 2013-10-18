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
#include <pcl/ModelCoefficients.h>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

#define DEBUG_MODE 0   // Set 1 to enable saving the morphological masks
#define SHOW_FIT   0   // Set 1 to display the fitted sphere
#define USE_NONLINEAR_REFINEMENT 1 // Set 1 to use nonlinear refinement for sphere fitting

//void showfittedSphere(CloudType::ConstPtr cloud, pcl::PointIndices& pi, pcl::ModelCoefficients& coeff);
//void debug_save_clusters(CloudType::Ptr cloud, std::vector<pcl::PointIndices>& cluster_list);

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
	  
	  double d = ((p.x-ctr(0))*(p.x-ctr(0)) +
		      (p.y-ctr(1))*(p.y-ctr(1)) +
		      (p.z-ctr(2))*(p.z-ctr(2)))/(target_radius*target_radius);
#define INLIER_THRESHOLD 0.99	  
	  if ( d < (1+INLIER_THRESHOLD) && d > (1-INLIER_THRESHOLD) )
     
	    inliers.indices.push_back(linear_idx);
	  //	  printf("add new inlier\n");
	}
    }
}


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
  const float bg_threshold      = config["background_threshold"].as<float>();
  const float near_cutoff       = config["near_cutoff"].as<float>();
  const float far_cutoff        = config["far_cutoff"].as<float>();
  const float morph_radius      = config["morphological_radius"].as<float>();
  const double min_volumn       = config["cluster_min_volumn"].as<double>();
  const double max_volumn       = config["cluster_max_volumn"].as<double>();

  const YAML::Node& sensors = config["Sensors"];

  PCL_INFO("%d sensors in the config\n", sensors.size());
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
	  FastBlobExtractor<PointType> obe(width, height, min_count, max_count, min_volumn, max_volumn);
	  obe.setInputCloud(fg);
	  obe.extract(cluster_list, mask);

	  assert(!cluster_list.empty());

	  // Find the cluster that is most likely to be a ball
	  double cost, best_cost = 1e5;
	  int best_idx = -1;
	  std::vector<Eigen::Vector3d> centers(cluster_list.size());
	  for(int k=0; k< cluster_list.size();++k)
	    {
	      if(cluster_list[k].indices.empty())
		continue;
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
	  nlsf.Clear();
	  nlsf.SetInputCloud(fg, cluster_list[best_idx].indices);
	  // Notice this is a different kind of cost, not comparable to linear fit cost
	  cost = nlsf.ComputeFitCost(centers[best_idx]);

	  pcl::PointIndices inlier_idx;
	  //	  remask_inliers_ellipse(centers[best_idx], width, height, -fl, target_radius, inlier_idx);
	  remask_inliers(fg, centers[best_idx], inlier_idx, width, height, -fl, target_radius);
	  
	  nlsf.Clear();//reset the cost function
	  nlsf.SetInputCloud(fg, inlier_idx.indices);
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
