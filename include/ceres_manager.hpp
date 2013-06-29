
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include "generic_sensor.hpp"

class CeresResourceManager
{
public:
  CeresResourceManager(int num_sensor=0; int num_landmarks=0);

  // Basic I/O operations
  bool ReadFromStream(std::istream& is);

  void WriteToStream(std::ostream& os);

  // Adding items to manager
  void AddSensor(std::shared_ptr<Sensor> s_ptr);

  void AddLandmark(Eigen::Vector3d ldmk);

  // Accessors:
  Eigen::Vector3d& GetSensorReading(int sensor_id, int obs_id);
  Eigen::Vector3d& GetLandmark(int ldmk_id);

private:
  std::vector<std::shared_ptr<Sensor> > _sensors;
  std::vector<Eigen::Vector3d> _landmarks;
};
