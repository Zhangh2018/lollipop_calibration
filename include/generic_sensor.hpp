
#pragma once

#include <iostream>
#include <vector>
#include <array>

#include <ceres/ceres.h>
#include <Eigen/Core>

// Generic base sensor class
class Sensor
{
public:
  Sensor(std::string _name, std::string _type):name(_name), type(_type){};

  // Solve the extrinsic as a linear problem -> gives a good initial guess
  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk)=0;

  // Setup the joint problem as a nonlinear optimization problem
  virtual void InitCeresProblem(ceres::Problem& prob,
				std::vector<Eigen::Vector3d>& ldmk) = 0;

  static boost::shared_ptr<Sensor> Create(std::string type, std::string name);

  void SetOrigin(std::vector<double>& vec);

  void AddObservation(int id, std::vector<double>& vec);

  const std::string GetName(){return name;}
  const std::string GetType(){return type;}
  const double* GetOrigin()  {return origin.data();}

  // Variables
  const std::string name;
  const std::string type;
  std::array<double, 7> origin;          // extrinsic: [x, y, z, w, rx, ry, rz]
  std::map<int, Eigen::Vector3d> measure;// short for measurement
};

class RangeSensor: public Sensor
{
public:
  RangeSensor(std::string _name="rs", std::string _type="rs"):Sensor(_name, _type){}

  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk);

  virtual void InitCeresProblem(ceres::Problem& prob,
				std::vector<Eigen::Vector3d>& ldmk);

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
