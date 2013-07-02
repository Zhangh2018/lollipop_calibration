/*
  A sample script to emit a standard config yaml file for pipeline
 */

#include <yaml-cpp/yaml.h>
#include <sstream>
#include <fstream>

void test_node(std::ostream& os);
void test_emitter(std::ostream& os);

int main(int argc, char** argv)
{
  if (argc<2)
    {
      printf("Usage: %s <output.yaml>\n", argv[0]);
      return 0;
    }
  std::ofstream ofs(argv[1], std::ofstream::out);
  test_emitter(ofs);
  ofs.close();
}

void test_node(std::ostream& os)
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
  os << cfg;
}
/*
void test_emitter(std::ostream& os)
{
  YAML::Emitter out;
  out <<YAML::BeginMap;
  out <<YAML::Key << "NumSensors" << YAML::Value << 2;
  out <<YAML::Key << "GlobalPathPrefix"  << YAML::Value << "/home/ming/Project/PCL/";
  out <<YAML::Key << "Sensors" << YAML::Value << YAML::BeginSeq;
  for(int i=0; i<2; i++)
    {
      out << YAML::BeginMap;
      out << YAML::Key << "Name" << YAML::Value << "x0";
      out << YAML::Key << "Type" << YAML::Value << "openni";
      out << YAML::Key << "Origin" << YAML::Value;
      out << YAML::Flow << YAML::BeginSeq << 0<<0<<0<<1<<0<<0<<0<<YAML::EndSeq;

      out << YAML::Key << "Data" << YAML::Value << YAML::BeginSeq;
      for (int j=0; j<6; j++)
	{
	  std::stringstream ss;
	  ss << "x" << i<<"_"<<j<<".pcd";
	  out << ss.str();
	}
      out << YAML::EndSeq << YAML::EndMap;
    }
  out << YAML::EndSeq << YAML::EndMap;

  os << out.c_str();
}
*/
void test_emitter(std::ostream& os)
{
  YAML::Emitter out;
  out <<YAML::BeginMap;
  //  out <<YAML::Key << "NumPCDs" << YAML::Value << 2;
  //  out <<YAML::Key << "NumObjs" << YAML::Value << 2;
  out <<YAML::Key << "GlobalPathPrefix"  << YAML::Value << "/home/ming/Project/PCL/";
  out <<YAML::Key << "PCDs" << YAML::Value << YAML::BeginSeq;
  for(int i=0; i<2; i++)
    {
      out << YAML::BeginMap;
      out << YAML::Key << "Path" << YAML::Value << "data/xtion/x0_0.pcd";
      out << YAML::Key << "Origin" << YAML::Value;
      out << YAML::Flow << YAML::BeginSeq << 0<<0<<0<<1<<0<<0<<0<<YAML::EndSeq;
      out << YAML::Key << "Color" << YAML::Value;
      out << YAML::Flow << YAML::BeginSeq << 255<<0<<0<<YAML::EndSeq;
      /*
      out << YAML::Key << "Data" << YAML::Value << YAML::BeginSeq;
      for (int j=1; j<2; j++)
	{
	  std::stringstream ss;
	  ss << "x" << i<<"_"<<j<<".pcd";
	  out << ss.str();
	}
      out << YAML::EndSeq;
      */
      out << YAML::EndMap;
    }
  out << YAML::EndSeq;

  out << YAML::Key << "Objects" << YAML::Value << YAML::BeginSeq;
  for (int i=0; i< 1; ++i)
    {
      out << YAML::BeginMap;
      out << YAML::Key << "Origin" << YAML::Value;
      out << YAML::Flow << YAML::BeginSeq << 0 << 0 << 0<< YAML::EndSeq;
      out << YAML::Key << "Radius" << YAML::Value << 0.1275;
      out << YAML::EndMap;
    }
  out << YAML::EndMap;

  os << out.c_str();
}
