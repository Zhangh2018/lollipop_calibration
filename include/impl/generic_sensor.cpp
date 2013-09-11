
#include "generic_sensor.hpp"

#include "error_models.hpp"   // for Euclidean3DError
#include "linearTF_solver.hpp" // for LinearTfSolver

#include <ceres/ceres.h>

// Depending on the type name, create the corresponding sensor type
boost::shared_ptr<Sensor> Sensor::Create(std::string type, std::string name)
{
  if (type == "openni" || type == "swissranger" || type == "rs" || type=="sr" || type=="stereo")
    {
      return boost::shared_ptr<Sensor>(new RangeSensor(name));
    }
  /*
  else if (type == "camera")
    {
      return boost::shared_ptr<Sensor>(new Camera(name));
    }
  */
  else
    {
      std::cerr <<"Error: Unknown type of sensor: " << type <<std::endl;
      exit(1);
    }
}

void Sensor::SetOrigin(std::vector<double>& vec)
{
  assert(vec.size() == 7);
  std::copy(vec.begin(), vec.end(), origin.begin());
}

void Sensor::AddObservation(int id, std::vector<double>& vec)
{
  // TODO: Unnecessary creation of temporary variable?
  Eigen::Vector3d v;
  if (vec.size() == 3)
    v << vec[0], vec[1], vec[2];
  else
    v << vec[0], vec[1], 1.0;
  measure[id] = v;
}


/***************************************
           class RangeSensor
***************************************/
void RangeSensor::SolveLinearTf(std::vector<Eigen::Vector3d>& ldmk)
{
  LinearTfSolver lts(measure.size());
  std::map<int, Eigen::Vector3d>::iterator it = measure.begin();
  while( it != measure.end() )
    {
      lts.AddPointPair(ldmk[it->first], it->second);
      ++it;
    }
  
  lts.EstimateTfSVD(&(origin[0]));
}

void RangeSensor::InitCeresProblem(ceres::Problem& prob,
				   std::vector<Eigen::Vector3d>& ldmk)
{
  std::map<int, Eigen::Vector3d>::iterator it = measure.begin();
  while( it != measure.end() )
    {
      Eigen::Vector3d& v = it->second;
      ceres::CostFunction* cf = Euclidean3DError::Create(v(0),v(1),v(2), false);
      ceres::LossFunction* lf = NULL;
      prob.AddResidualBlock(cf, lf, &(origin[3]), &(origin[0]), ldmk[it->first].data());
      ++it;
    }
  prob.SetParameterization(&(origin[3]), new ceres::QuaternionParameterization);
}
