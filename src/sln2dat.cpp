/*
  Convert a solution.yaml file to a data-oriented format
 */

// For loading config files
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv)
{
  YAML::Node sln = YAML::LoadFile(argv[1]);

  const YAML::Node& sensor_nd = sln["Sensors"];
  const YAML::Node& landmk_nd = sln["Landmarks"];
  std::cout << sensor_nd.size() << " " << landmk_nd.size() << std::endl;

  std::cout << std::setprecision(9);
  // Write sensors' origins
  for(int i=0; i < sensor_nd.size(); i++)
    {
      const YAML::Node& origin_nd = sensor_nd[i]["Origin"];
      std::cout << origin_nd[0].as<double>() <<" "<<origin_nd[1].as<double>()<<" "<<origin_nd[2].as<double>()<<" "<<origin_nd[3].as<double>()<<" "<<origin_nd[4].as<double>()<<" "<<origin_nd[5].as<double>()<<" "<<origin_nd[6].as<double>()<<std::endl;
    }

  // Write out landmark locations
  for(int i=0; i < landmk_nd.size(); i++)
    {
      const YAML::Node& obs = landmk_nd[i];
      std::cout << obs[1].as<double>() <<" "<<obs[2].as<double>() <<" "<<obs[3].as<double>() << std::endl;
    }

  // Write out each senser's readings
  for(int i=0; i < sensor_nd.size(); i++)
    {
      const YAML::Node& nd = sensor_nd[i]["Observations"];
      for(int j=0; j < landmk_nd.size(); j++)
	{
	  std::cout << nd[j][1].as<double>()<<" "<<nd[j][2].as<double>()<<" "<<nd[j][3].as<double>()<<std::endl;
	}
    }
}
