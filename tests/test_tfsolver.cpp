
#include <yaml-cpp/yaml.h> // for loading config yaml file
#include <fstream>

bool SaveToYaml(std::string file, std::vector<Eigen::Vector3d>& lv,
		std::vector<boost::shared_ptr<Sensor> >& sv);

int main(int argc, char** argv)
{
  if (argc < 2)
    {
      printf("Usage: %s <config.yaml>\n", argv[0]);
      exit(1);
    }

  YAML::Node config = YAML::LoadFile(argv[1]);
  YAML::Node sensors= config["Sensors"];
  YAML::Node landmarks= config["Landmarks"];

  std::vector<boost::shared_ptr<Sensor> > sv(sensors.size());
  std::vector<Eigen::Vector3d> lv(landmarks.size());

  // Process the sensors first 
  for (int i=0; i < sv.size(); ++i)
    {
      YAML::Node node = sensors[i];

      sv[i] = Sensor::Create(si["Type"].as<std::string>(), si["Name"].as<std::string>());

      YAML::Node origin=node["Origin"];

      std::vector<double> vec(origin.size());
      for (int j=0; j < origin.size(); ++j)
	vec[i] = origin[j].as<double>();
	
      sv[i]->SetOrigin(vec);

      YAML::Node obs = node["Observations"];
      for (int j=0; j < obs.size(); ++j)
	{
	  // obs[i] contains 0: landmark ID; 1-N: actual observations
	  vec.resize(obs[i].size() -1);
	  for(int k=1; k< obs[i].size(); k++)
	    vec[k] = obs[i][k];

	  sv[i]->AddObservation(obs[i][0].as<int>, vec);
	}
    }

  // Then process the landmarks
  YAML::Node landmarks= config["Landmarks"];
  for (int i=0; i < lv.size(); ++i)
    {
      // coz Eigen doesn't like undetermined type (unless you explictly type cast it)
      double x, y, z;
      x = landmarks[i][0].as<double>();
      y = landmarks[i][1].as<double>();
      z = landmarks[i][2].as<double>();
      lv[i] << x, y, z;
    }


  // Now do the solving part...
  for (int i=0; i < sv.size(); ++i)
    sv[i]->SolveLinearTf(lv);

  // Save intermediate result:
  bool status = SaveToYaml("lsqr.yaml", lv, sv);

  // Then do the non-linear solving
  ceres::Problem problem;
  bool fixed = false;
  for (int i=0; i < sv.size(); ++i)
    sv[i]->InitCeresProblem(problem, lv, fixed);

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.minimizer_progress_to_stdout = true;
  options.linear_solver_type = ceres::DENSE_SCHUR;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  status = SaveToYaml("nlsqr.yaml", lv, sv);

  return 0;
}

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
      out << sv[i].offset(0) << sv[i].offset(1) << sv[i].offset(2);
      out << sv[i].orient.w()<< sv[i].orient.x()<< sv[i].orient.y()<< sv[i].orient.z()<<YAML::EndSeq;

      out << YAML::Key << "Observations:" << YAML::Value << YAML::BeginSeq;

      std::map<int, Eigen::Vector3d>::iterator it = sv[i]->Obs.begin();
      while (it != sv[i]->Obs.end())
	{
	  Eigen::Vector3d& v = it->second;
	  out << YAML::BeginSeq << YAML::Flow << it->first << v(0) << v(1) << v(2) << YAML::EndSeq;
	}
      out << YAML::EndSeq;
    }

  out << YAML::EndSeq;
  out << YAML::Key << "Landmarks" << YAML::Value << YAML::BeginSeq;

  for(int i=0; i< lv.size(); ++i)
    {
      Eigen::Vector3d& v = lv[i];
      out << YAML::BeginSeq << YAML::Flow << v(0) << v(1) << v(2) << YAML::EndSeq;
    }

  out << YAML::EndSeq << YAML::EndMap;

  ofstream os;
  os.open(file);
  os << out.c_str();
  os.close();
  return true;
}
