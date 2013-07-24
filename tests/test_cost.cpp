#include <stdio.h>
#include <fstream>

#include "error_models.hpp"

#include <ceres/ceres.h>
#include <ceres/types.h>

double** construct_arrays(int n, int size);
void destroy_arrays(double**, int n);

int main(int argc, char** argv)
{
  // Ceres variables
  ceres::Problem prob;
  ceres::Solver::Options opt;  
  ceres::Solver::Summary summary;
  Euclidean3DError::RayCostError::R = 0.1275;

  // open file
  std::ifstream is(argv[1]);
  if( !is.is_open())
    {
      printf("Fail to open file %s\n", argv[1]);
      return -1;
    }

  int num_cam, num_ball, num_points;
  is >> num_cam >> num_ball >> num_points;
  
  printf("Num_cam: %d, Num_ball: %d\n", num_cam, num_ball);

  // allocate camera origins
  double** cams = construct_arrays(num_cam, 7);
  double** balls= construct_arrays(num_ball,3);
  
  // Read in camera origins:
  for (int i=0; i< num_cam; ++i)
    is>>cams[i][0]>>cams[i][1]>>cams[i][2]>>cams[i][3]>>cams[i][4]>>cams[i][5]>>cams[i][6];

  for (int i=0; i< num_ball; ++i)
    is >> balls[i][0] >> balls[i][1] >> balls[i][2];

  double x, y, z;
  for (int i=0; i< num_cam; ++i)
    {
      for(int j=0; j< num_ball; ++j)
	{
	  for(int k=0; k< num_points; ++k)
	    {
	      is >> x >> y >> z;
	      ceres::CostFunction* cf = new Euclidean3DError::RayCostError(x,y,z);
	      ceres::LossFunction* lf = NULL;
	      prob.AddResidualBlock(cf, lf, &(cams[i][3]), cams[i], balls[j]);
	    }
	}
    }
  is.close();

  opt.max_num_iterations = 25;
  opt.function_tolerance = 1e-10;
  opt.minimizer_progress_to_stdout = true;
  opt.linear_solver_type = ceres::DENSE_SCHUR;

  ceres::Solve(opt, &prob, &summary);
  std::cout << summary.FullReport() << std::endl;
  // delete
  destroy_arrays(cams, num_cam);
  destroy_arrays(balls,num_ball);
}

double** construct_arrays(int n, int size)
{
  double** p = new double*[n];
  for(int i=0; i< n; ++i)
    p[i] = new double[size];
  return p;
}

void destroy_arrays(double** p, int n)
{
  for(int i=0; i< n; ++i)
    delete[] p[i];

  delete[] p;
}
