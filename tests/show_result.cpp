/*
  Display calibration results
 */

#include <fstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_picking_event.h>
#include <boost/thread/thread.hpp>

// Loading config files
#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointType> ColorHandler;

double r[] = {255.0,   0.0, 255.0,   0.0, 255.0,   0.0};
double g[] = {  0.0, 255.0, 255.0,   0.0,   0.0, 255.0};
double b[] = {  0.0,   0.0,   0.0, 255.0, 255.0, 255.0};
pcl::visualization::PCLVisualizer viewer("Result Viewer");

void pointpicking_cb(const pcl::visualization::PointPickingEvent& event)
{
  static unsigned char status = 0;
  static PointType p;
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
	  viewer.removeShape("arrow");
	  viewer.removeText3D("arrow");
	}
      status = 2;
    }
  else
    {
      PointType q;
      event.getPoint(q.x, q.y, q.z);
      ss << q;
      PCL_ERROR("Second point picked at %s\n", ss.str().c_str());
      viewer.addArrow(q, p, 1.0, 1.0, 1.0, true);
      status = 1;
    }
}

int main(int argc, char** argv)
{
  // TODO: add more command line options
  if (argc < 3)
    {
      PCL_ERROR("Usage: %s <config.yaml> <solution.yaml>\n", argv[0]);
      exit(1);
    }

  std::ofstream os;

  YAML::Node config = YAML::LoadFile(argv[1]);
  YAML::Node solved = YAML::LoadFile(argv[2]);
 
  const double target_radius    = config["target_ball_radius"].as<double>();
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();

  const YAML::Node& solved_sensor_nd = solved["Sensors"];
  const YAML::Node& solved_landmk_nd = solved["Landmarks"];

  const YAML::Node& config_sensor_nd = config["Sensors"];

  // TODO: this is a hack, MUST BE FIXED IN NEAR FUTURE
  const int num_bg = 1;//config["NumBackground"].as<int>();
  const int num_fg = config_sensor_nd[0]["PCDs"].size() - num_bg;

  // Fill in the sensor origins, so pointclouds can be transformed to a common frame
  const int num_sensors = solved_sensor_nd.size();
  std::vector<Eigen::Vector3f> t(num_sensors);
  std::vector<Eigen::Quaternionf> q(num_sensors);
  for (int j=0; j < num_sensors; ++j)
    {
      const YAML::Node& origin = solved_sensor_nd[j]["Origin"];
      float w, x, y, z;
      x = origin[0].as<float>();
      y = origin[1].as<float>();
      z = origin[2].as<float>();
      t[j] << x, y, z;

      printf("Sensor[%d] at [%f %f %f ", j, x, y, z);
      w = origin[3].as<float>(); x = origin[4].as<float>();
      y = origin[5].as<float>(); z = origin[6].as<float>();
      Eigen::Quaternionf& qj = q[j];
      qj.w() = w; qj.x() = x; qj.y() = y; qj.z() = z;
      printf("%f %f %f %f]\n", w, x, y, z);
    }

  // Initialize the visualizer
  viewer.setBackgroundColor (0, 0, 0);
  viewer.registerPointPickingCallback(&pointpicking_cb);
  viewer.addCoordinateSystem (0.5);
  viewer.initCameraParameters ();
  viewer.setCameraPosition(0.0, 0.0, -1.5, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
  //  viewer.registerKeyboardCallback (keyboardEventOccurred, NULL);

  // Load the point cloud and display them in the common frame
  for (int i=0; i < num_fg; ++i)
    {
      // Its scope is essential
      //      std::vector<boost::shared_ptr<ColorHandler> > colorH(num_sensors);

      for (int j=0; j < num_sensors; ++j)
	{
	  CloudType::Ptr raw_cloud(new CloudType), tf_cloud(new CloudType);

	  std::string sensor_name = solved_sensor_nd[j]["Name"].as<std::string>();
	  std::string filename = config_sensor_nd[j]["PCDs"][i+num_bg].as<std::string>();
	  std::string full_path= glob_prefix + filename;

	  if (pcl::io::loadPCDFile<PointType> (full_path, *raw_cloud) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }
	  else
	    PCL_WARN("Loaded PCD %s\n", full_path.c_str());

	  pcl::transformPointCloud(*raw_cloud, *tf_cloud, t[j], q[j]);

	  ColorHandler colorH(tf_cloud, r[j], g[j], b[j]);
	  // Add/Update point cloud to viewer:
	  if (i > 0)//TODO: Is filename a good identifier?
	    {
	      viewer.updatePointCloud<PointType>(tf_cloud, colorH, sensor_name);
	      //	      printf("Update points\n");
	    }
	  else
	    {
	      //	      colorH[j].reset(new ColorHandler(tf_cloud, r[j], g[j], b[j]));
	      viewer.addPointCloud<PointType>(tf_cloud, colorH, sensor_name);
	      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, sensor_name);
	    }
	}

      // Center of the target sphere:
      double sx, sy, sz;// note index[0] is just its ID
      sx = solved_landmk_nd[i][1].as<double>();
      sy = solved_landmk_nd[i][2].as<double>();
      sz = solved_landmk_nd[i][3].as<double>();

      if (i > 0)
	viewer.updateSphere(pcl::PointXYZ(sx,sy,sz), target_radius, 1.0, 1.0, 1.0, "sphere");
      else
	{
	  viewer.addSphere(pcl::PointXYZ(sx, sy, sz), target_radius, 1.0, 1.0, 1.0, "sphere");
	  viewer.addCoordinateSystem (0.05);
	  viewer.initCameraParameters ();
	  viewer.resetCameraViewpoint("sphere");
	  //	  viewer.setCameraPose(0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
	}

      // Let the viewer spin...
      while(!viewer.wasStopped())
	{
	  viewer.spinOnce(100);
	  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
      
      viewer.resetStoppedFlag();
    }
  viewer.close();
  return 0;
}

/*
bool next_flag = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* ptr)
{
  if (event.getKeySym () == " " && event.keyDown ())
    next_flag = true;
}
*/
