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
std::string glob_prefix;

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

void keyboardEvent_cb(const pcl::visualization::KeyboardEvent &event, void* cookie)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
    *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(cookie);

  if(event.getKeySym() == "n" && event.keyDown())
    {
      std::cout << "n was pressed" <<std::endl;
    }
}

void AddSphereToVisualizer(const YAML::Node& origin_nd,
			   boost::shared_ptr<PCLVisualizer>& viewer,
			   double radius, double r, double g, double b,
			   const std::string prefix    = "",
			   const Eigen::Vector3f& t    = Eigen::Vector3f::Zero(),
			   const Eigen::Quaternionf& q = Eigen::Quaternionf::Identity())
{
  Eigen::Vector3f p;
  for(int i=0; i < origin_nd.size(); ++i)
    {
      const YAML::Node& node = origin_nd[i];
      float x, y, z; // TODO: for debugging only
      x = node[0].as<float>();
      y = node[1].as<float>();
      z = node[2].as<float>();
      p << x, y, z;
      Eigen::Vector3f temp = q*p+t;
      Point pt(temp(0), temp(1), temp(2));
      std::stringstream ss;
      ss << prefix << "sphere_" << i;
      viewer->addSphere(pt, radius, r, g, b, ss.str());
      std::cout << " at "<< pt << " with r="<<radius<<std::endl;
    }
}

void AddObjectToVisualizer(const YAML::Node& obj_nd,
			   boost::shared_ptr<PCLVisualizer>& viewer,
			   const std::string prefix    = "",
			   const Eigen::Vector3f& t    = Eigen::Vector3f::Zero(),
			   const Eigen::Quaternionf& q = Eigen::Quaternionf::Identity())
{
  for (int i=0; i < obj_nd.size(); ++i)
    {
      YAML::Node node = obj_nd[i];
      std::string obj_type = node["Type"].as<std::string>();
      std::cout << "Add object " << obj_type;

      // Set up a custom color handler
      int r = 255, g = 255, b = 255;
      if (node["Color"])
	{
	  r=node["Color"][0].as<int>(); 
	  g=node["Color"][1].as<int>(); 
	  b=node["Color"][2].as<int>();
	}

      if (obj_type == "sphere") // TODO: Whay passing so many params to function?
	{
	  double radius = node["Radius"].as<double>();
	  AddSphereToVisualizer(node["Origin"], viewer, radius, r, g, b, prefix, t, q);
	}
    }
}

void AddPointCloudToVisualizer(const YAML::Node& pcd_nd,
			       boost::shared_ptr<PCLVisualizer>& viewer,
			       double r = 255.0, double g= 255.0, double b=255.0,
			       const Eigen::Vector3f& t    = Eigen::Vector3f::Zero(),
			       const Eigen::Quaternionf& q = Eigen::Quaternionf::Identity())
{
  Cloud::Ptr cloud(new Cloud), cloud_tf(new Cloud);
  for (int j=0; j< pcd_nd.size(); ++j)
    {
      std::string identifier= pcd_nd[j].as<std::string>();
      std::string pull_path = glob_prefix + identifier;
      // Load the point cloud
      if (pcl::io::loadPCDFile<Point> (pull_path, *cloud) == -1)
	exit(1);
     
      // Transform the point cloud
      pcl::transformPointCloud(*cloud, *cloud_tf, t, q);

      // Set up a custom color handler
      ColorHandler colorH (cloud_tf, r, g, b);
      
      // Add the point cloud to viewer
      viewer->addPointCloud<Point> (cloud_tf, colorH, identifier);
      PCL_INFO("Add PointCloud: %s with %d points\n", identifier.c_str(), cloud->size());
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, identifier); 
    }
}

void AddSensorToVisualizer(const YAML::Node& sensor_nd,
			   boost::shared_ptr<PCLVisualizer>& viewer)
{
  for (int i=0; i< sensor_nd.size(); ++i)
    {
      const YAML::Node& node = sensor_nd[i];

      // Find the sensor Affine transform
      Eigen::Vector3f offset;
      offset(0) = node["Origin"][0].as<float>();
      offset(1) = node["Origin"][1].as<float>();
      offset(2) = node["Origin"][2].as<float>();
      Eigen::Quaternionf quat( node["Origin"][3].as<float>(),  node["Origin"][4].as<float>(),
			       node["Origin"][5].as<float>(),  node["Origin"][6].as<float>());

      double r = 255, g = 255, b = 255;
      if (node["Color"])
	{
	  r=node["Color"][0].as<double>(); 
	  g=node["Color"][1].as<double>(); 
	  b=node["Color"][2].as<double>();
	}

      // Load PCDs
      if (node["PCDs"])
	AddPointCloudToVisualizer(node["PCDs"], viewer, r, g, b, offset, quat);

      if (node["Objects"])
	{
	  std::string name = node["Name"].as<std::string>();
	  AddObjectToVisualizer(node["Objects"], viewer, name, offset, quat);
	}
    }
}

int main(int argc, char** argv)
{
  if(argc<2)
    printHelpAndExit(argv[0]);

  // Load the display config file
  YAML::Node config = YAML::LoadFile(argv[1]);
  glob_prefix = config["GlobalPathPrefix"].as<std::string>();

  // Set up PCL::Visualizer related variables:
  viewer = boost::shared_ptr<PCLVisualizer>(new PCLVisualizer(argc, argv, "show_pcd"));
  viewer->registerPointPickingCallback(&pointpicking_cb);
  //  viewer->registerKeyboardCallback(keyboardEvent_cb, (void*)&viewer);
  viewer->addCoordinateSystem(0.05);
  viewer->initCameraParameters ();
  viewer->setCameraPose(0.0, 0.0, -1.5, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);

  // Load the Sensors:
  if (config["Sensors"])
    AddSensorToVisualizer(config["Sensors"], viewer);
  
  // Add Object if provided
  if(config["Objects"])
    AddObjectToVisualizer(config["Objects"], viewer);
  
  // Let it run forever
  viewer->spin();
}

/*
std::string splitPathName(const std::string& str)
{
  int f_slash = str.find_last_of("/\\");
  if (f_slash == std::string::npos)
    f_slash = -1;
  int f_dot = str.find_last_of(".");
  return str.substr(f_slash+1, f_dot);
}
*/
