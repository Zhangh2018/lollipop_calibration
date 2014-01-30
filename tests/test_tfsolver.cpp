
#include <yaml-cpp/yaml.h> // for loading config yaml file
#include <iostream>
#include <fstream>
#include <vector>

#include "generic_sensor.hpp"

bool SaveToYaml(std::string file, std::vector<Eigen::Vector3d>& lv,
		std::vector<boost::shared_ptr<Sensor> >& sv);

int main(int argc, char** argv)
{
  if (argc < 2)
    {
      printf("Usage: %s <problem.yaml>\n", argv[0]);
      printf("Two files will be produces in current folder: lsqr.yaml and nlsqr.yaml\n");
      exit(1);
    }

  YAML::Node config = YAML::LoadFile(argv[1]);
  YAML::Node sensors= config["Sensors"];
  YAML::Node landmarks= config["Landmarks"];

  std::vector<boost::shared_ptr<Sensor>> sv(sensors.size());
  std::vector<Eigen::Vector3d> lv(landmarks.size());

  // Process the sensors first 
  for (int i=0; i < sv.size(); ++i)
    {
      YAML::Node node = sensors[i];

      std::string type = sensors[i]["Type"].as<std::string>();
      std::string name = sensors[i]["Name"].as<std::string>();

      sv[i] = RangeSensor::Create(name, type);
      std::cout<< "Create sensor of type: "<< sv[i]->GetType() << " and name "<< sv[i]->GetName() <<std::endl;

      YAML::Node origin=node["Origin"];

      std::vector<double> vec(origin.size());
      for (int j=0; j < origin.size(); ++j)
	vec[j] = origin[j].as<double>();
	
      sv[i]->SetOrigin(vec);

      YAML::Node obs = node["Observations"];
      vec.resize(obs[0].size() -1);
      for (int j=0; j < obs.size(); ++j)
	{
	  // obs[j] contains 0: landmark ID; 1-N: actual observations
	  YAML::Node obs_j = obs[j];
	  int id = obs_j[0].as<int>();
	  for(int k=1; k < obs_j.size(); k++)
	    vec[k-1] = obs_j[k].as<double>();

	  sv[i]->AddObservation(id, vec);
	}
    }

  // Then process the landmarks
  for (int i=0; i < lv.size(); ++i)
    {
      // Notice the 1st entry is landmark id:
      // coz Eigen doesn't like undetermined type (unless you explictly type cast it)
      double x, y, z;
      x = landmarks[i][1].as<double>();
      y = landmarks[i][2].as<double>();
      z = landmarks[i][3].as<double>();
      lv[i] << x, y, z;

      std::cout << "Add landmark at ["<<x<<","<<y<<","<<z<<"]"<<std::endl;
    }

  
  // Now do the solving part...
  for (int i=0; i < sv.size(); ++i)
    sv[i]->SolveLinearTf(lv);

  //  SaveToYaml("lsqr.yaml", lv, sv);
  // Save intermediate result:
  bool status = SaveToYaml("lsqr.yaml", lv, sv);
  
  // Then do the non-linear solving
  ceres::Problem problem;
  for (int i=0; i < sv.size(); ++i)
    sv[i]->InitCeresProblem(problem, lv);

  ceres::Solver::Options options;
  options.max_num_iterations = 40;
  options.function_tolerance = 1e-10;
  options.minimizer_progress_to_stdout = true;
  options.linear_solver_type = ceres::DENSE_SCHUR;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  
  status = SaveToYaml("nlsqr.yaml", lv, sv);
  
  return 0;
}

// Given the current setting of the problem, save it to a yaml file
// which can be reloaded for a linear or nonlinear solving session
bool SaveToYaml(std::string file, std::vector<Eigen::Vector3d>& lv,
		std::vector<boost::shared_ptr<Sensor> >& sv)
{
  YAML::Emitter out;
  out <<YAML::BeginMap;
  out <<YAML::Key << "Sensors" << YAML::Value << YAML::BeginSeq;

  for(int i=0; i < sv.size(); ++i)
    {
      out << YAML::BeginMap;
      out << YAML::Key << "Type" << YAML::Value << sv[i]->GetType();
      out << YAML::Key << "Name" << YAML::Value << sv[i]->GetName();
      out << YAML::Key << "Origin"<< YAML::Value << YAML::Flow << YAML::BeginSeq;

      const double* org = sv[i]->GetOrigin();
      out << org[0] << org[1] << org[2] << org[3] << org[4] << org[5] << org[6] << YAML::EndSeq;

      out << YAML::Key << "Observations" << YAML::Value << YAML::BeginSeq;
      std::map<int, Eigen::Vector3d>::iterator it = sv[i]->_measure.begin();
      while (it != sv[i]->_measure.end())
	{
	  Eigen::Vector3d& v = it->second;
	  out << YAML::Flow << YAML::BeginSeq << it->first << v(0) << v(1) << v(2) << YAML::EndSeq;
	  it++;
	}
      out << YAML::EndSeq << YAML::EndMap;
    }

  out << YAML::EndSeq;
  out << YAML::Key << "Landmarks" << YAML::Value << YAML::BeginSeq;

  for(int i=0; i< lv.size(); ++i)
    {
      Eigen::Vector3d& v = lv[i];
      out << YAML::Flow << YAML::BeginSeq << i<< v(0) << v(1) << v(2) << YAML::EndSeq;
    }

  out << YAML::EndSeq << YAML::EndMap;

  std::ofstream os(file, std::ofstream::out);
  os << out.c_str();
  os.close();
  return true;
}
