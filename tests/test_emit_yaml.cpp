/*
  A sample script to emit a standard config yaml file for pipeline
 */

#include <yaml-cpp/yaml.h>
#include <sstream>
#include <fstream>

int main(int argc, char** argv)
{
  YAML::Node cfg;

  cfg["NumSensors"] = 2;
  cfg["NumBackgroundPCDs"] = 1;
  cfg["NumForegroundPCDs"] = 6;
  cfg["GlobalPathPrefix"]  = "/home/ming/Project/PCL/";

  for(int i=0; i<2; i++)
    {
      YAML::Node sensor;
      sensor["Name"] = "x0";
      sensor["Type"] = "openni";
      sensor["PathPrefix"] = "//";
      double v[3] = {1.0, 2.4, 3.2};
      std::vector<double> vv(v, v+3);
      sensor["Origin"] = vv;
      
      for (int j=0; j< 6; j++)
	{
	  std::stringstream ss;
	  ss << "x" << i<<"_"<<j <<".pcd";
	  sensor["Data"].push_back(ss.str());
	}

      cfg["Sensors"].push_back(sensor);
    }

  std::ofstream ofs("config.yaml");
  ofs << cfg;
}
