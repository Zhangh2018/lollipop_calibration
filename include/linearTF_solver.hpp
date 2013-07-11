
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

class LinearTfSolver
{
public:
  LinearTfSolver(unsigned int num_pts);

  void AddPointPair(Eigen::Vector3d& ref, Eigen::Vector3d& tgt);

  void EstimateTfSVD(double* origin);

private:
  unsigned int max_pts, cur_pt;
  Eigen::MatrixXd cloud_ref;
  Eigen::MatrixXd cloud_tgt;
};

#include "impl/linearTF_solver.cpp"
