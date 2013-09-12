
#include "generic_sensor.hpp"

/*
// Depending on the type name, create the corresponding sensor type
boost::shared_ptr<Sensor> Sensor::Create(std::string& name, std::string& type)
{
  if (type == "openni" || type == "swissranger" || type == "rs" || type=="sr" || type=="stereo")
    {
      return boost::shared_ptr<Sensor>(new RangeSensor(name, type));
    }
  else
    {
      std::cerr <<"Error: Unknown type of sensor: " << type <<std::endl;
      exit(1);
    }
}
*/
void Sensor::SetOrigin(std::vector<double>& vec)
{
  assert(vec.size() == 7);
  std::copy(vec.begin(), vec.end(), _origin.begin());
}

void Sensor::AddObservation(int id, std::vector<double>& vec)
{
  // TODO: Unnecessary creation of temporary variable?
  Eigen::Vector3d v;
  if (vec.size() == 3)
    v << vec[0], vec[1], vec[2];
  else
    v << vec[0], vec[1], 1.0;
  _measure[id] = v;
}



