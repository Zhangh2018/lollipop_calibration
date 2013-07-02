
#pragma once

#include "generic_sensor.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

class Openni: public RangeSensor
{
  RangeSensor(std::string _name="openni"):RangeSensor(_name){};

  RangeSensor(std::istream& is);

  double f;
};
