#pragma once

#include <pcl/point_cloud.h>

#include <ceres/ceres.h>
#include <ceres/types.h>

#include <vector>

// Template Base class
template <typename T>
class SphereFitter
{
public:
  SphereFitter(double ball_radius):R(ball_radius){};
  
  virtual void SetInputCloud(typename pcl::PointCloud<T>::Ptr input)=0;
  
  virtual void SetInputCloud(typename pcl::PointCloud<T>::Ptr input,
			     std::vector<int>& indices) =0;
  
  // Center can be used as 
  // 1. return value (in linear fitters)
  // 2. initial guess and return value (in nonlinear fitters)
  virtual double ComputeFitCost(Eigen::Vector3d& center)=0;

  // The radius is fixed
  const double R;
};

/***********************************************************
 // Linear fitting method from Chernov[09]
 // but modified for 3D and fixed radius
***********************************************************/
template <typename T>
class LinearFitter: public SphereFitter<T>
{
public:
  LinearFitter(double radius);

  virtual void SetInputCloud(typename pcl::PointCloud<T>::Ptr input);
  
  virtual void SetInputCloud(typename pcl::PointCloud<T>::Ptr input,
			     std::vector<int>& indices);

  virtual double ComputeFitCost(Eigen::Vector3d& center);

private:
  Eigen::MatrixXd M; // 5x5 matrix
};


/***********************************************************
 // Nonlinear fit from Witzgall[09]
 // implemented using ceres
***********************************************************/
template <typename T>
class NonlinearFitter: public SphereFitter<T>
{
public:
  NonlinearFitter(double radius);

  virtual void SetInputCloud(typename pcl::PointCloud<T>::Ptr input);
  
  virtual void SetInputCloud(typename pcl::PointCloud<T>::Ptr input,
			     std::vector<int>& indices);

  virtual double ComputeFitCost(Eigen::Vector3d& center);
private:
  ceres::Solver::Options options;
  ceres::Problem problem;
  std::vector<ceres::CostFunction*> cost_fn;
};

#include "sphere_fitter.cpp"
