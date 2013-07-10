
#include "generic_sensor.hpp"

Sensor::Sensor(YAML::Node n)
{
  ReadFromYamlNode(n);
}

void Sensor
