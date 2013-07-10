

#pragma once

#include <iostream>
#include <vector>

// Forward declarations:
class Eigen::Vector3d;
class Eigen::Quaterniond;
class ceres::Problem;

// Generic base sensor class
class Sensor
{
public:
  Sensor(std::string _name="generic"):name(_name){};
  Sensor(std::istream& is);

  virtual ~Sensor();

  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk)=0;

  virtual void InitCeresProblem(ceres::Problem& prob,
				    std::vector<Eigen::Vector3d>& ldmk) = 0;

  virtual void AddObservation(int id, std::vector<double>& vec);

  static boost::shared_ptr<Sensor> Create(std::string type, std::string name);

  void SetOrigin(std::vector<double>& vec);

protected:
  const std::string name;
  Eigen::Vector3d    offset; // origin
  Eigen::Quaterniond orient; // short for orientation

  std::map<int, Eigen::Vector3d> measure; // short for measurement
};

class RangeSensor: protected Sensor
{
public:
  RangeSensor(std::string _name="rs"):Sensor(_name){}

  RangeSensor(std::istream& is);

  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk);

  virtual void InitCeresProblem(ceres::Problem& prob,
			std::vector<Eigen::Vector3d>& ldmk);

};

class Camera: protected Sensor
{
public:
  Camera(std::string _name="cam"):Sensor(_name){}

  Camera(std::istream& is);

  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk);

  virtual void InitCeresProblem(ceres::Problem& prob,
			std::vector<Eigen::Vector3d>& ldmk);


};
