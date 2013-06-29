

#pragma once

// Generic base sensor class
class Sensor
{

public:
  Sensor(std::string _name="generic"):name(_name);
  Sensor(std::istream& is);
  

  virtual void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk)=0;

  virtual void InitCeresProblem(ceres::Problem& prob,
				std::vector<Eigen::Vector3d>& ldmk) = 0;

  virtual void WriteToStream (std::ostream& os) = 0;
  virtual void ReadFromStream(std::istream& is) = 0;

protected:
  std::string name;
  Eigen::Vector3d    offset; // origin
  Eigen::Quaterniond orient; // short for orientation

  std::map<int, Eigen::Vector3d> measure; // short for measurement

  void WriteCommonInfo(std::ostream& os);
  void ReadCommonInfo (std::istream& is);
};

class RangeSensor: protected Sensor
{
public:
  RangeSensor(std::string _name="rs"):Sensor(_name){}

  RangeSensor(std::istream& is);

  void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk);

  void InitCeresProblem(ceres::Problem& prob,
			std::vector<Eigen::Vector3d>& ldmk);

  void WriteToStream (std::ostream& os);
  void ReadFromStream(std::istream& is);
};

class Camera: protected Sensor
{
public:
  Camera(std::string _name="cam"):Sensor(_name){}

  Camera(std::istream& is);

  void SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk);

  void InitCeresProblem(ceres::Problem& prob,
			std::vector<Eigen::Vector3d>& ldmk);

  void WriteToStream (std::ostream& os);
  void ReadFromStream(std::istream& is);
};
