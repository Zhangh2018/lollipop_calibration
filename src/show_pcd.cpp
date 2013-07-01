/*
 Global Attributes:
 1. Number of pcds
 2. Global path prefix
 Individual PCD attributes:
  1. Each pcd must specify a file_path
  2. Each pcd has an optional origin (default to [0 0 0 1 0 0 0])
  3. Each pcd has an optional color (default to white [255 255 255])
 */

#include <yaml-cpp/yaml.h>   // For yaml support

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>

using namespace pcl::visualization;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef PointCloudColorHandlerCustom<Point> ColorHandler;

// Global variable: viewer
boost::shared_ptr<PCLVisualizer> viewer;

void printHelpAndExit(char* str)
{
  printf("Usage: %s <pcds.yaml>\n", str);
  exit(1);
}

void pointpicking_cb(const PointPickingEvent& event)
{
  static unsigned char status = 0;
  static Point p;
  if (event.getPointIndex () == -1)
    {
      //     PCL_WARN("No point selected\n");
      return;
    }
  std::stringstream ss;
  if (status <2)
    {
      event.getPoint(p.x, p.y, p.z);
      ss << p;
      PCL_WARN("First point picked at %s\n", ss.str().c_str());
      if (status)
	{
	  viewer->removeShape("arrow");
	  viewer->removeText3D("arrow");
	}
      status = 2;
    }
  else
    {
      Point q;
      event.getPoint(q.x, q.y, q.z);
      ss << q;
      PCL_ERROR("Second point picked at %s\n", ss.str().c_str());
      viewer->addArrow(q, p, 1.0, 1.0, 1.0, true);
      status = 1;
    }
}

std::string splitPathName(const std::string& str)
{
  int f_slash = str.find_last_of("/\\");
  if (f_slash == std::string::npos)
    f_slash = -1;
  int f_dot = str.find_last_of(".");
  return str.substr(f_slash+1, f_dot);
}

int main(int argc, char** argv)
{
  if(argc<2)
    printHelpAndExit(argv[0]);

  // Load the display config file
  YAML::Node config = YAML::LoadFile(argv[1]);
  int nPCDs, nObjs;
  if (config["NumPCD"])
    nPCDs = config["NumPCD"].as<int>();
  if (config["NumObj"])
    nObjs = config["NumObj"].as<int>();
  const std::string glob_prefix = config["GlobalPathPrefix"].as<std::string>();


  // Set up PCL::Visualizer related variables:
  viewer = boost::shared_ptr<PCLVisualizer>(new PCLVisualizer(argc, argv, "show_pcd"));
  viewer->registerPointPickingCallback(&pointpicking_cb);

  // Set up visualizer callback functions
  for (int i=0; i< nPCDs; ++i)
    {
      Cloud::Ptr cloud(new Cloud), cloud_tf(new Cloud);

      YAML::Node node = config["PCDs"][i];
      std::string pull_path = glob_prefix + node["Path"].as<std::string>();
      // Load the point cloud
      if (pcl::io::loadPCDFile<Point> (pull_path, *cloud) == -1)
	return -1;

      /*
	transform and colorize the point cloud:
      */
      Eigen::Vector3f offset;
      offset(0) = node["Origin"][0].as<float>();
      offset(1) = node["Origin"][1].as<float>();
      offset(2) = node["Origin"][2].as<float>();
      Eigen::Quaternionf quat( node["Origin"][3].as<float>(),  node["Origin"][4].as<float>(),
			       node["Origin"][5].as<float>(),  node["Origin"][6].as<float>());
      
      // Transform the point cloud
      pcl::transformPointCloud(*cloud, *cloud_tf, offset, quat);

      // Set up a custom color handler
      int r = 255, g = 255, b = 255;
      if (node["Color"])
	{
	  r=node["Color"][0].as<int>(); 
	  g=node["Color"][1].as<int>(); 
	  b=node["Color"][2].as<int>();
	}
      ColorHandler colorH (cloud_tf, r, g, b);

      // Add the point cloud to viewer
      std::string name = splitPathName(node["Path"].as<std::string>());
      viewer->addPointCloud<Point> (cloud_tf, colorH, name);
      PCL_INFO("Add PointCloud: %s with %d points\n", name.c_str(), cloud->size());
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    }

  for (int i=0; i < nObjs; ++i)
    {
      YAML::Node node = config["Objects"][i];
      std::string obj_type = node["Type"].as<std::string>();
      if (obj_type == "sphere")
	{
	  Point p;
	  p.x = node["Origin"][0].as<float>();
	  p.y = node["Origin"][1].as<float>();
	  p.z = node["Origin"][2].as<float>();
	  std::stringstream ss;
	  ss << "sphere_" << i;
	  viewer->addSphere(p, node["Radius"].as<double>(), ss.str());
	}
    }
  // Let it run forever
  viewer->spin();
}
