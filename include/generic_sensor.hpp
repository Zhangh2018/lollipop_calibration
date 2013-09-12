
#pragma once

#include <iostream>
#include <vector>
#include <array>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

// Generic base sensor class
class Sensor
{
public:
  Sensor(std::string name, std::string type):_name(name), _type(type){};

  // Solve the extrinsic as a linear problem -> gives a good initial guess
  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk)=0;

  // Setup the joint problem as a nonlinear optimization problem
  virtual void InitCeresProblem(ceres::Problem& prob,
				std::vector<Eigen::Vector3d>& ldmk) = 0;

  virtual bool RunPipeline(void* query, void* train, void* options,
			   std::vector<int>& inlier, Eigen::Vector3d& result)=0;

  //  static boost::shared_ptr<Sensor> Create(std::string& name, std::string& type);

  void SetOrigin(std::vector<double>& vec);

  void AddObservation(int id, std::vector<double>& vec);

  const std::string GetName(){return _name;}
  const std::string GetType(){return _type;}
  const double* GetOrigin()  {return _origin.data();}

  // Variables
  const std::string _name;
  const std::string _type;
  std::array<double, 7> _origin;          // extrinsic: [x, y, z, w, rx, ry, rz]
  std::map<int, Eigen::Vector3d> _measure;// short for measurement
};


/*
class Camera: protected Sensor
{
public:
  Camera(std::string _name="cam"):Sensor(_name){}

  Camera(std::istream& is);

  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk);

  virtual void InitCeresProblem(ceres::Problem& prob,
			std::vector<Eigen::Vector3d>& ldmk);


};
*/

#include "impl/generic_sensor.cpp"
#include "impl/range_sensor.cpp"
