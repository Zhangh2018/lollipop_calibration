
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <fstream>

int main(int argc, char** argv)
{
  if (argc <2)
    {
      printf("Error\n");
      exit(1);
    }

  YAML::Node config = YAML::LoadFile(argv[1]);
  
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();

  std::cout << "Prefix: " << glob_prefix << std::endl;

  const double target_radius = config["target_ball_radius"].as<double>();
  const double bg_threshold = config["background_threshold"].as<double>();
  const double morph_radius = config["morphological_radius"].as<double>();
  const int min_count = config["cluster_min_count"].as<int>();
  const int max_count = config["cluster_max_count"].as<int>();
  const double min_volumn = config["cluster_min_volumn"].as<double>();
  const double max_volumn = config["cluster_max_volumn"].as<double>();
						
  std:: cout << "Min volumn = " << min_volumn << std::endl;

  YAML::Node sensors = config["Sensors"];
  for(int i=0; i< sensors.size(); i++)
    {
      std::string name = sensors[i]["Name"].as<std::string>();
      std::string type = sensors[i]["Type"].as<std::string>();

      std::cout << "Name: "<<name << " Type: "<< type << std::endl;
      double f = sensors[i]["focal_length"].as<double>();
      int width= sensors[i]["Width"].as<int>();
      int height=sensors[i]["Height"].as<int>();

      int num_bg = sensors[i]["NumBackground"].as<int>();
      int num_fg = sensors[i]["NumForeground"].as<int>();
      
      for (int j=0; j < num_bg; j++)
	{
	  std::string pcd = sensors[i]["PCDs"][j].as<std::string>();
	  std::cout << "BG: " << pcd << std::endl;
	}

      for (int j=0; j < num_fg; j++)
	{
	  std::string pcd = sensors[i]["PCDs"][j+num_bg].as<std::string>();
	  std::cout << "FG: " << pcd << std::endl;
	}
    }

  return 0;
}
