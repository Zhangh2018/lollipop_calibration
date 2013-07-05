
#include "sphere_fitter.hpp"

#include <limits> // for +inf

// For dealing with pointcloud type
#include <pcl/point_cloud.h>

// For Linear Sphere Fitting methods
#include <Eigen/Eigenvalues>

// For nonlinear fitting
#include <ceres/ceres.h>
#include <ceres/types.h>

/***********************************************************
 ***********************************************************
                     LinearFitter
 ***********************************************************
 **********************************************************/
template<typename T>
LinearFitter<T>::LinearFitter(double radius)
 :SphereFitter<T>(radius)
{
}

template<typename T> void LinearFitter<T>::
SetInputCloud(typename pcl::PointCloud<T>::Ptr input)
{
  Eigen::MatrixXd Z(5, input->size());

  for(int i=0; i< input->size(); ++i)
    {
      T& p = input->points[i];
      Z(0, i) = double(p.x*p.x+p.y*p.y+p.z*p.z);
      Z(1, i) = double(p.x);
      Z(2, i) = double(p.y);
      Z(3, i) = double(p.z);
      Z(4, i) = 1.0;
    }

  M = Z*Z.transpose() /input->size();
}

template<typename T> void LinearFitter<T>::
SetInputCloud(typename pcl::PointCloud<T>::Ptr input,
	      std::vector<int>& indices)
{
  Eigen::MatrixXd Z(5, indices.size());

  for(int i=0; i< indices.size(); ++i)
    {
      T& p = input->points[indices[i]];
      Z(0, i) = double(p.x*p.x+p.y*p.y+p.z*p.z);
      Z(1, i) = double(p.x);
      Z(2, i) = double(p.y);
      Z(3, i) = double(p.z);
      Z(4, i) = 1.0;
    }

  M = Z*Z.transpose() /indices.size();
}

template<typename T> double LinearFitter<T>::
ComputeFitCost(Eigen::Vector3d& center)
{
  Eigen::MatrixXd N(5,5);
  // Pratt's Constraint matrix: B^2+C^2+D^2-4AE=1
  N << 0.0, 0.0, 0.0, 0.0,-2.0,
       0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0,
      -2.0, 0.0, 0.0, 0.0, 0.0;

  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es(M,N);

  // Find the min eigen value (in an absolute sense)
  Eigen::VectorXd eigVal(es.eigenvalues());
  double min_v = std::abs(eigVal(0));
  int min_i = 0;
  for(int i=1; i<5; ++i)
    {
      if(std::abs(eigVal(i)) < min_v)
	{
	  min_v = std::abs(eigVal(i));
	  min_i = i;
	}
    }

  // Then the solution is the column with minimum eigen value
  Eigen::VectorXd eigVec(es.eigenvectors().col(min_i));

  // Return the found center
  center(0) = -0.5*eigVec(1)/eigVec(0);
  center(1) = -0.5*eigVec(2)/eigVec(0);
  center(2) = -0.5*eigVec(3)/eigVec(0);
  
  // But this sphere is with unconstraintd Radius:
  // Assume fixed Radius, then E should be E= A*R^2-(B^2+C^2+D^2)/(4A)
  eigVec(4) = eigVec(0)*(this->R*this->R-center.squaredNorm());
  return eigVec.transpose()*M*eigVec;
}

/***********************************************************
 ***********************************************************
                    NonlinearFitter
 ***********************************************************
 **********************************************************/

// Helper class: RayCostFunction
class RayCostFunction:public ceres::SizedCostFunction<1, 3>
{
public:
  RayCostFunction(double xj, double yj, double zj)
  {
    r  = std::sqrt(xj*xj+yj*yj+zj*zj);// + 1e-5
    x  = xj / r;
    y  = yj / r;
    z  = zj / r;
    //    printf("r= %lf x=%lf y=%lf z=%lf\n", r, x, y, z);
  }

  virtual ~RayCostFunction(){}

  virtual bool Evaluate(double const* const* params,
			double* res, double** jac) const
  {
    const double& X = params[0][0];
    const double& Y = params[0][1];
    const double& Z = params[0][2];

    double p   = X*x + Y*y + Z*z;
    double q2  = pow(y*Z-z*Y,2)+pow(z*X-x*Z,2)+pow(x*Y-y*X,2);
    double q   = std::sqrt(q2);

    if (q < this->R)
      res[0] = p - std::sqrt(this->R*this->R - q2) - r;
    else
      res[0] = std::sqrt((p-r)*(p-r) + (q-this->R)*(q-this->R));

    if (jac != NULL && jac[0] != NULL)
      {
	if (q < this->R)
	  {
	    double denom = std::sqrt(this->R*this->R - q2);
	    jac[0][0] = x + (X - x*p) /denom;
	    jac[0][1] = y + (Y - y*p) /denom;
	    jac[0][2] = z + (Z - z*p) /denom;
	  }
	else
	  {
	    double pr = p - r;
	    double qR = 1 - this->R/q;
	    jac[0][0] = (pr * x + qR *(X - x*p))/res[0];
	    jac[0][1] = (pr * y + qR *(Y - y*p))/res[0];
	    jac[0][2] = (pr * z + qR *(Z - z*p))/res[0];
	  }
      }
    return true;
  }

  static double R;
private:
  double r;
  double x, y, z;// x, y, z are normalized
};

template<typename T>
NonlinearFitter<T>::NonlinearFitter(double radius)
  :SphereFitter<T>(radius)
{
  RayCostFunction::R = radius;

  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
}


template<typename T> void NonlinearFitter<T>::
SetInputCloud(typename pcl::PointCloud<T>::Ptr input)
{
  cost_fn.resize(input->size());
  
  for(int i=0; i< input->size(); ++i)
    {
      T& p = input->points[i];

      cost_fn[i] = new RayCostFunction(p.x,p.y,p.z);
    }
}

template<typename T> void NonlinearFitter<T>::
SetInputCloud(typename pcl::PointCloud<T>::Ptr input,
	      std::vector<int>& indices)
{
  cost_fn.resize(indices.size());

  for(int i=0; i< indices.size(); ++i)
    {
      T& p = input->points[indices[i]];

      cost_fn[i] = new RayCostFunction(p.x,p.y,p.z);
    }
}

template<typename T> double NonlinearFitter<T>::
ComputeFitCost(Eigen::Vector3d& center)
{

  for(int i=0; i< cost_fn.size(); ++i)
    {
      // Robust loss function to deal with outliers
      ceres::LossFunction* lf = new ceres::HuberLoss(1.0);
      problem.AddResidualBlock(cost_fn[i],lf,static_cast<double*>(center.data()));
    }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (summary.termination_type == ceres::FUNCTION_TOLERANCE ||
      summary.termination_type == ceres::GRADIENT_TOLERANCE ||
      summary.termination_type == ceres::PARAMETER_TOLERANCE)
    {
      std::cout << "Center found at "<< center<<"\n";
      return summary.final_cost;
    }
  else
    {
      std::cout << "Return the initial guess"<<std::endl;
      return std::numeric_limits<double>::max();
    }
}
