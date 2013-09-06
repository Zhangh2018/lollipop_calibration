/*
  Detect the balls and display them side by side

  The process is similar to pipeline but very inefficient,
  because the purpose is for visual inspection only.
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
#include <pcl/ModelCoefficients.h>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

#define DEBUG_MODE 0   // Set 1 to enable saving the morphological masks

void showfittedSphere(CloudType::Ptr cloud, pcl::PointIndices& pi, pcl::ModelCoefficients& coeff, std::string& name, int id, int& vi);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);
//void debug_save_clusters(CloudType::Ptr cloud, std::vector<pcl::PointIndices>& cluster_list);

// Assume there are only two sensors
double vp[][7] = {{0.0, 0.0, 0.5, 1.0, 0.3, 0.3, 0.3}, {0.5, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
int g_first_time = 0;
int g_num_sensor = 0;
bool g_exit_flag = false;

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

  // used for view_port, the values are subject to change
  int viewer_property[] = {1, 2, 3, 4};

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
  g_num_sensor = sensors.size();

  const int num_fg = sensors[0]["PCDs"].size() - num_bg;

  // For each frame
  for (int k=0; k< num_fg; ++k)
    {
      //for each sensor:
      for (int i=0; i< sensors.size(); i++)
	{
	  // Load sensor information:
	  std::string name    = sensors[i]["Name"].as<std::string>();
	  const float fl      = sensors[i]["focal_length"].as<float>();
	  const int width     = sensors[i]["Width"].as<int>();
	  const int height    = sensors[i]["Height"].as<int>();
	  
	  const int min_count = sensors[i]["cluster_min_count"].as<int>();
	  const int max_count = sensors[i]["cluster_max_count"].as<int>();

	  // background mask
	  std::vector<char> mask;
	  ImageFilter<PointType> img_filter(bg_threshold, near_cutoff, far_cutoff);

	  const YAML::Node& pcd_node = sensors[i]["PCDs"];

	  // Load the background models (normally just one)
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

	  // Process current foreground
	  CloudType::Ptr fg(new CloudType);
	  
	  std::string filename = pcd_node[k+num_bg].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *fg) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }

	  PCL_WARN("\nProcessing Sensor[%d], frame[%d] (%s)\n", i, k, filename.c_str()); 
	  // filter out background:
	  img_filter.GetForegroundMask(fg, mask);

#if DEBUG_MODE
	  img_filter.WriteMaskToFile("mask.dump", mask);
#endif
	 
	  // Morphological operations:
	  Morphology::Erode2_5D<PointType> (fg, mask, fl, morph_radius);

#if DEBUG_MODE
	  img_filter.WriteMaskToFile("erode.dump", mask);
#endif

	  Morphology::Dilate2_5D<PointType>(fg, mask, fl, morph_radius);

#if DEBUG_MODE
	  img_filter.WriteMaskToFile("dilate.dump", mask);
#endif

	  // Extract clusters from binaryImage
	  std::vector<pcl::PointIndices> cluster_list;
	  OnePassBlobExtractor<PointType> obe(width, height, min_count, max_count, min_volumn, max_volumn);
	  obe.setInputCloud(fg);
	  obe.extract(cluster_list, mask);
	  
	  pcl::ModelCoefficients coeff;
	  if(cluster_list.empty())
	    {
	      coeff.values.push_back(0.0f); 	  coeff.values.push_back(0.0f); 
	      coeff.values.push_back(0.0f);	  coeff.values.push_back(0.0f);   
	      pcl::PointIndices empty_idx;
	      showfittedSphere(fg, empty_idx, coeff, name, i, viewer_property[i]);
	    }
	  else // We might find a potential sphere:
	    {
	      // Find the cluster that is most likely to be a ball
	      double cost, best_cost = 1e5;
	      int best_idx = -1;
	      std::vector<Eigen::Vector3d> centers(cluster_list.size());
	      for(int t=0; t< cluster_list.size();++t)
		{
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

	      Eigen::Vector3d& best_ctr = centers[best_idx];
	      coeff.values.push_back(best_ctr(0));
	      coeff.values.push_back(best_ctr(1));
	      coeff.values.push_back(best_ctr(2));
	      coeff.values.push_back(target_radius);

	      showfittedSphere(fg, cluster_list[best_idx], coeff, name, i, viewer_property[i]);
	    }	  
	} // end of Sensor loop

      while(!viewer->wasStopped())
	{
	  viewer->spinOnce(100);
	  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
      if(g_exit_flag)
	break;
      viewer->resetStoppedFlag();
    }// end of fg loop

  return 0;
}
/*
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
*/
void showfittedSphere(CloudType::Ptr cloud, pcl::PointIndices& pi, 
		      pcl::ModelCoefficients& coeff, 
		      std::string& name, int id, int& vi)
{
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
  pcl::visualization::PointCloudColorHandlerCustom<PointType> outlier_handler(outlier_cloud,   0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType>  inlier_handler( inlier_cloud, 255,   0, 0);
  
  if(g_first_time >= g_num_sensor)
    {
      viewer->removePointCloud(name+"_inlier_cloud", vi);
      viewer->removePointCloud(name+"_outlier_cloud", vi);
      viewer->removeShape(name+"_sphere", vi);
    }
  else
    {
      viewer->createViewPort (vp[id][0], vp[id][1], vp[id][2], vp[id][3], vi);
      viewer->setBackgroundColor (0.0, 0.0, 0.0, vi);

      ++g_first_time;
    }
  viewer->addPointCloud<PointType> (inlier_cloud, inlier_handler, name+"_inlier_cloud", vi);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name+"_inlier_cloud");
  viewer->addPointCloud<PointType> (outlier_cloud, outlier_handler, name+"_outlier_cloud", vi);
  //      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier_cloud");
  
  viewer->addSphere(coeff, name+"_sphere", vi);
}

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
