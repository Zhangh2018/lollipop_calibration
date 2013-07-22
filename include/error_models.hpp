
#ifndef ERROR_MODELS_H
#define ERROR_MODELS_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <ceres/jet.h>
//#include <iomanip>

// Templated range sensor model for use with Ceres.
// The camera is parameterized using 7 parameters: 4 for rotation+ 3 for translation
// i.e. only the extrinsics are being optimized.
namespace Euclidean3DError
{
  class AutoDiffError
  {
  public:
    AutoDiffError(double ox, double oy, double oz)
      :_ox(ox), _oy(oy), _oz(oz){}

    template <typename T>
    bool operator()(const T* const cam_quat, const T* const cam_tran,
		    const T* const point, T* residuals) const
    {
      // Use a quaternion rotation that doesn't assume the quaternion is
      // normalized, since one of the ways to run the bundler is to let Ceres
      // optimize all 4 quaternion parameters unconstrained.    
      T p[3], rp[3];
      p[0] = T(_ox); p[1]=T(_oy); p[2]=T(_oz);
      ceres::QuaternionRotatePoint(cam_quat, p, rp);

      rp[0] += cam_tran[0];
      rp[1] += cam_tran[1];
      rp[2] += cam_tran[2];
      /*      
      std::cout<<"point [";
      std::cout<< std::setw(10) << std::setfill(' ')<< p[0]<<" ";
      std::cout<< std::setw(10) << std::setfill(' ')<< p[1]<<" ";
      std::cout<< std::setw(10) << std::setfill(' ')<< p[2]<<"]->[";
      std::cout<< std::setw(10) << std::setfill(' ')<< rp[0]<<" ";
      std::cout<< std::setw(10) << std::setfill(' ')<< rp[1]<<" ";
      std::cout<< std::setw(10) << std::setfill(' ')<< rp[2]<<"]<-[";
      std::cout<< std::setw(10) << std::setfill(' ')<< point[0] <<" "; 
      std::cout<< std::setw(10) << std::setfill(' ')<< point[1] << " ";
      std::cout<< std::setw(10) << std::setfill(' ')<< point[2]<<"]\n";
      */
      residuals[0] = rp[0] - point[0];
      residuals[1] = rp[1] - point[1];
      residuals[2] = rp[2] - point[2];
      return true;
    }

    double _ox, _oy, _oz; // observed (x, y, z)
  };

  class FixedDiffError: public ceres::SizedCostFunction<3, 4, 3, 3>
  {
  public:
    FixedDiffError(double ox, double oy, double oz)
    {p[0] =ox; p[1] = oy; p[2] = oz;}

    virtual ~FixedDiffError() {}

    virtual bool Evaluate(double const* const* params, 
			  double* residuals, double** jacobians) const
    {  
      const double* cam_quat = params[0];
      const double* cam_tran = params[1];
      const double* point = params[2];
      double temp[3];
      ceres::QuaternionRotatePoint(cam_quat, p, temp);

      temp[0] += cam_tran[0];
      temp[1] += cam_tran[1];
      temp[2] += cam_tran[2];

      residuals[0] = temp[0] - p[0];
      residuals[1] = temp[1] - p[1];
      residuals[2] = temp[2] - p[2];

      if (jacobians != NULL) 
	{
	  // The camera extrinsics are fixed
	  if(jacobians[0] != NULL)
	    for(int i=0; i<3*4; i++)
	      jacobians[0][i] = 0.0;
	  //	    jacobians[0] = NULL;
	  if(jacobians[1] != NULL)
	    //	    for(int i=0; i<3*3; i++)
	      //	      jacobians[1][i] = 0.0;
	    jacobians[1] = NULL;
      
	  // but the points are allowed to move
	  if(jacobians[2] != NULL)
	    {
	      //	      double rot[9];
	      ceres::QuaternionToRotation(cam_quat, jacobians[2]);

	      //	      for(int i=0; i<9;++i)
	      //		jacobians[2][i] = rot[i];
	    }
      //      jacobians[2][0*3+0]=1.0; jacobians[2][0*3+1]=0.0; jacobians[2][0*3+2]=0.0;
      //      jacobians[2][1*3+0]=0.0; jacobians[2][1*3+1]=1.0; jacobians[2][1*3+2]=0.0;
      //      jacobians[2][2*3+0]=0.0; jacobians[2][2*3+1]=0.0; jacobians[2][2*3+2]=1.0;
	}
      return true;
    }
    
    double p[3]; // observed (x, y, z)
  };

  ceres::CostFunction* Create(double x, double y, double z, bool fixed= false)
  {
    if (!fixed)
      return new ceres::AutoDiffCostFunction<AutoDiffError,3,4,3,3>(
			new AutoDiffError(x, y, z));
    else
      return new FixedDiffError(x, y, z);
  }

  
  class RayError
  {
  public:
    RayError(double px, double py, double pz)
      :x(px), y(py), z(pz)
    {
      r  = std::sqrt(px*px+py*py+pz*pz);// + 1e-5
      x  = px / r;
      y  = py / r;
      z  = pz / r;
    }
    
    template <typename T>
    bool operator()(const T* const cam_quat, const T* cam_tran,
		    const T* const center, T* residuals) const
    {
      T tx = T(x);
      T ty = T(y);
      T tz = T(z);
      T tr = T(r);
      T tR = T(this->R);

      T Qp[4];
      Qp[0] = cam_quat[0];
      Qp[1] =-cam_quat[1];
      Qp[2] =-cam_quat[2];
      Qp[3] =-cam_quat[3];

      T center_w_demean[3];
      center_w_demean[0] = center[0] - cam_tran[0];
      center_w_demean[1] = center[1] - cam_tran[1];
      center_w_demean[2] = center[2] - cam_tran[2];

      // Project the center to camera frame
      T center_c[3];
      ceres::QuaternionRotatePoint(Qp, center_w_demean, center_c);

      T p = center_c[0]*tx + center_c[1]*ty + center_c[2]*tz;
      T yz= ty*center_c[2] - tz*center_c[1];
      T zx= tz*center_c[0] - tx*center_c[2];
      T xy= tx*center_c[1] - ty*center_c[0];

      T q2= yz*yz + zx*zx + xy*xy;
      T q = ceres::sqrt(q2);

      if (q < tR)
	residuals[0] = p - ceres::sqrt(tR*tR - q2) - tr;
      else
	residuals[0] = ceres::sqrt((p-tr) + (q-tr)*(q-tR));
    }
    
    static double R;
    double x,y,z,r;
  };
  double RayError::R = 0.0;

};

#include "impl/rayError_models.cpp"

#endif
