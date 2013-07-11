/*
  Assume we already know where the balls are located and where the 
  cameras are placed, project the balls back to camera frame and
  do a refit
 */

#include "sphere_fitter.hpp"

#include <fstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

int main(int argc, char** argv)
{
  if (argc < 3)
    {
      PCL_ERROR("Usage: %s <solution.yaml> <config.yaml>\n");
      exit(1);
    }

  std::ofstream os;
  os.open("run2.yaml");

  YAML::Node solved = YAML::LoadFile(argv[1]);
  YAML::Node config = YAML::LoadFile(argv[2]);

  const double target_radius    = config["target_ball_radius"].as<double>();
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();
  const int num_bg              = config["NumBackground"].as<int>();
  const int num_fg              = config["NumForeground"].as<int>();

  const YAML::Node& sensor_nd = solved["Sensors"];
  const YAML::Node& landmk_nd = solved["Landmarks"];

  // For convenience, save all landmarks into a vector of Vector3d
  std::vector<Eigen::Vector3d> landmk_vc(landmk_nd.size());
  for (int j=0; j < landmk_vc.size(); j++)
    {
      landmk_vc[j] << static_cast<double>(landmk_nd[j][1].as<double>()),
	              static_cast<double>(landmk_nd[j][2].as<double>()),
	              static_cast<double>(landmk_nd[j][3].as<double>());
    }

  // Project the landmarks to each sensor frame
  for (int i=0; i < sensor_nd.size(); i++)
    {
      const YAML::Node& origin_nd = sensor_nd[i]["Origin"];

      // convert to t and q
      Eigen::Vector3d t;
      t<< static_cast<double>(origin_nd[0].as<double>()), 
	static_cast<double>(origin_nd[1].as<double>()), 
	static_cast<double>(origin_nd[2].as<double>());
      Eigen::Quaterniond q;
      q.w() = origin_nd[3].as<double>();
      q.x() = origin_nd[4].as<double>();
      q.y() = origin_nd[5].as<double>();
      q.z() = origin_nd[6].as<double>();

      // Dump back into the stream:
      os <<"  - Type: " << sensor_nd[i]["Type"] << std::endl;
      os <<"    Name: " << sensor_nd[i]["Name"] << std::endl;
      os <<"    Origin: ["<<t(0)<<", "<<t(1)<<", "<<t(2)<<", "<<q.w()<<", "<<q.x()<<", "<<q.y()<<", "<<q.z()<<"]\n";
      os <<"    Observations:"<< std::endl;

      const double fl     = config["Sensors"][i]["focal_length"].as<double>();
      const int width     = config["Sensors"][i]["Width"].as<int>();
      const int height    = config["Sensors"][i]["Height"].as<int>();

      for (int j=0; j < landmk_vc.size(); j++)
	{
	  CloudType::Ptr cloudp(new CloudType);

	  std::string filename = config["Sensors"][i]["PCDs"][j+num_bg].as<std::string>();
	  std::string full_path= glob_prefix + filename;
	  if (pcl::io::loadPCDFile<PointType> (full_path, *cloudp) == -1)
	    {
	      PCL_ERROR("Failed to load file %s\n", full_path.c_str());
	      exit(1);
	    }

	  // Where is the ball supposed to be at in the sensor frame?
	  Eigen::Vector3d new_center = q.inverse()*(landmk_vc[j] - t);
	  std::cout << "Detected Center at ["<< sensor_nd[i]["Observations"][j]<<"]"<<std::endl;
	  std::cout << "Projected Center at ["<<j <<","<< new_center <<"]"<<std::endl;

	  // TODO: Create a mask(inlier list) to select the new ball
	  int u0 = width /2 + static_cast<int>(new_center(0) / new_center(2) * fl);
	  int v0 = height/2 + static_cast<int>(new_center(1) / new_center(2) * fl);
	  int ws = static_cast<int>(target_radius / new_center(2) * fl);
	  
	  std::vector<int> inlier_list;
	  for(int u=u0-ws; u<=u0+ws; ++u)
	    {
	      for(int v=v0-ws; v0+ws; ++v)
		{
		  if (u<0 || u>=width || v<0 || v>=height)
		    continue;
		    
		  int linear_idx = v*width+u;
		  PointType& p = cloudp->points[linear_idx];
		  
		  double d = ((p.x-new_center(0))*(p.x-new_center(0)) +
		              (p.y-new_center(1))*(p.y-new_center(1)) +
		              (p.z-new_center(2))*(p.z-new_center(2)))/(target_radius*target_radius);
		  
		  if ( abs(1-d) < 0.05 )
		    inlier_list.push_back(linear_idx);
		}
	    }

	  // Nonlinear refinement
	  NonlinearFitter<PointType> nlsf(target_radius);
	  nlsf.SetInputCloud(cloudp, inlier_list);
	  double cost = nlsf.ComputeFitCost(new_center);

	  os<<"      - ["<<j<<", "<<new_center(0)<<", "<<new_center(1)<<", "<<new_center(2)<<"]\n";
	}
    }

  os << "Landmarks:"<<std::endl;
  for (int j=0; j < landmk_vc.size(); j++)
    {
      Eigen::Vector3d& v = landmk_vc[j];
      os << "  - ["<< j<<", "<<v(0)<<", "<<v(1)<<", "<<v(2)<<"]"<<std::endl;
    }

  os.close();
}
