
#pragma once

#include "generic_sensor.hpp"

class Openni: public RangeSensor
{
  RangeSensor(std::string _name="openni"):RangeSensor(_name){}

  RangeSensor(std::istream& is);

  double f;
};
