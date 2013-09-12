
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

  class AngleReprojError
  {
  public:
    static ceres::CostFunction* Create(double x, double y, double z)
    {
      return new ceres::AutoDiffCostFunction<AngleReprojError,1,4,3,3>(
			new AngleReprojError(x, y, z));
    }

    AngleReprojError(double ox, double oy, double oz)
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

      T sine2 = (rp[2]*p[1]-rp[1]*p[2])*(rp[2]*p[1]-rp[1]*p[2])+
	        (rp[0]*p[2]-rp[2]*p[0])*(rp[0]*p[2]-rp[2]*p[0])+
	        (rp[0]*p[1]-rp[1]*p[0])*(rp[0]*p[1]-rp[1]*p[0]);
      // As in Guan08 paper
      residuals[0] = ceres::sqrt(sine2)/(rp[0]*p[0]+rp[1]*p[1]*rp[2]*p[2]);
      return true;
    }

    double _ox, _oy, _oz; // observed (x, y, z)
  };

  ceres::CostFunction* Create(double x, double y, double z, bool fixed= false)
  {
    if (!fixed)
      return new ceres::AutoDiffCostFunction<AutoDiffError,3,4,3,3>(
			new AutoDiffError(x, y, z));
    else
      return new FixedDiffError(x, y, z);
  }

  class RayAutoError
  {
  public:
    RayAutoError(double px, double py, double pz)
      :x(px), y(py), z(pz)
    {
      //      r  = std::sqrt(px*px+py*py+pz*pz);// + 1e-5
      //      x  = px / r;
      //      y  = py / r;
      //      z  = pz / r;
    }
    
    template <typename T>
    bool operator()(const T* const cam_quat, const T* cam_tran,
		    const T* const center, T* residuals) const
    {
      static int i=0;
      double r  = std::sqrt(x*x+y*y+z*z);// + 1e-5
      T tx = T(x/r);
      T ty = T(y/r);
      T tz = T(z/r);
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

      if ( q < tR )
	residuals[0] = p - tr - ceres::sqrt(tR*tR-q2);
      else
	residuals[0] = ceres::sqrt((p-tr)*(p-tr) + (q-tR)*(q-tR));

      //      std::cout << ++i <<" res=" << residuals[0]<< " q=" << q << std::endl;
      return true;
    }
    
    static double R;
    double x,y,z;
  };
  double RayAutoError::R = 0.0;


  // WARNING: This one optimizes a local quaternion, 
  // Q.w needs to be re-computed after the optimization
  class RayCostError:public ceres::SizedCostFunction<1, 4, 3, 3>
  {
  public:
    RayCostError(double px, double py, double pz)
    {
      r = std::sqrt(px*px+py*py+pz*pz);
      x = px/r;
      y = py/r;
      z = pz/r;
    }

    virtual ~RayCostError(){}

    virtual bool Evaluate(double const* const* params,
			  double *res, double** jac) const
    {
      static int i=0;
      // For convenience:
      const double* Q = params[0]; // quaternion
      const double* T = params[1]; // translation
      const double* O = params[2]; // sphere origin in world frame

      // m -> convert Quaternion to rotation Matrix in row major
      // Consider fixing w, so Q only has 3 DoF:
      double Qw = std::sqrt(1-Q[1]*Q[1]-Q[2]*Q[2]-Q[3]*Q[3]);
      double a2 = 2*Q[1]*Q[1], b2 = 2*Q[2]*Q[2], c2 = 2*Q[3]*Q[3];
      double aw = 2*Q[1]*Qw,   bw = 2*Q[2]*Qw,   cw = 2*Q[3]*Qw;
      double ab = 2*Q[1]*Q[2], bc = 2*Q[2]*Q[3], ca = 2*Q[3]*Q[1];

      double m00 = 1-b2-c2, m01 = ab - cw, m02 = ca + bw;
      double m10 = ab + cw, m11 = 1-a2-c2, m12 = bc - aw;
      double m20 = ca - bw, m21 = bc + aw, m22 = 1-a2-b2;

      // Transform the centroid in world frame -> camera frame
      double Xp = O[0] - T[0], Yp = O[1] - T[1], Zp = O[2] - T[2];
      double X  = m00*Xp + m10*Yp + m20*Zp;
      double Y  = m01*Xp + m11*Yp + m21*Zp;
      double Z  = m02*Xp + m12*Yp + m22*Zp;
      
      // Use the same notations as in the paper
      double p  = X*x + Y*y + Z*z;
      double q2 = (y*Z-z*Y)*(y*Z-z*Y)+(z*X-x*Z)*(z*X-x*Z)+(x*Y-y*X)*(x*Y-y*X);
      double q  = std::sqrt(q2);

      if( q < this->R)
	res[0] = p - std::sqrt(this->R*this->R - q2) - r;
      else
	res[0] = std::sqrt((p-r)*(p-r) + (q- this->R)*(q- this->R));

      // TODO: Make sure the memory addresses are valid
      if(jac && jac[0] && jac[1] && jac[2])
	{
	  double E_X, E_Y, E_Z;
	  if( q < this->R)
	    {
	      double denom = std::sqrt(this->R*this->R - q2);
	      E_X = x + (X - x*p) /denom; 
	      E_Y = y + (Y - y*p) /denom;
	      E_Z = z + (Z - z*p) /denom;
	    }
	  else
	    {
	      double pr = p - r;
	      double qR = 1 - this->R/q;
	      E_X = (pr * x + qR *(X - x*p))/res[0];
	      E_Y = (pr * y + qR *(Y - y*p))/res[0];
	      E_Z = (pr * z + qR *(Z - z*p))/res[0];
	    }
	  
	  // TODO: how to differentiate with respect Quaternion:
	  double m00_a = 0.0,          m00_b =-4*Q[2],       m00_c =-4*Q[3];
	  double m01_a = 2*Q[2]+ca/Qw, m01_b = 2*Q[1]+bc/Qw, m01_c =-2*Qw  +c2/Qw;
	  double m02_a = 2*Q[3]-ab/Qw, m02_b = 2*Qw  -b2/Qw, m02_c = 2*Q[1]-bc/Qw;
	  
	  double m10_a = 2*Q[2]-ca/Qw, m10_b = 2*Q[1]-bc/Qw, m10_c = 2*Qw  -c2/Qw;
	  double m11_a =-4*Q[1],       m11_b = 0.0,          m11_c =-4*Q[3];
	  double m12_a =-2*Qw  +a2/Qw, m12_b = 2*Q[3]+ab/Qw, m12_c = 2*Q[2]+ca/Qw;

	  double m20_a = 2*Q[3]+ab/Qw, m20_b =-2*Qw  +b2/Qw, m20_c = 2*Q[1]+bc/Qw;
	  double m21_a = 2*Qw  -a2/Qw, m21_b = 2*Q[3]-ab/Qw, m21_c = 2*Q[2]-ca/Qw;
	  double m22_a =-4*Q[1],       m22_b =-4*Q[2],       m22_c = 0.0;

	  // ----------------------------------------
	  double Ox_a  = m00_a * Xp + m10_a * Yp + m20_a * Zp;
	  double Oy_a  = m01_a * Xp + m11_a * Yp + m21_a * Zp;
	  double Oz_a  = m02_a * Xp + m12_a * Yp + m22_a * Zp;

	  double Ox_b  = m00_b * Xp + m10_b * Yp + m20_b * Zp;
	  double Oy_b  = m01_b * Xp + m11_b * Yp + m21_b * Zp;
	  double Oz_b  = m02_b * Xp + m12_b * Yp + m22_b * Zp;

	  double Ox_c  = m00_c * Xp + m10_c * Yp + m20_c * Zp;
	  double Oy_c  = m01_c * Xp + m11_c * Yp + m21_c * Zp;
	  double Oz_c  = m02_c * Xp + m12_c * Yp + m22_c * Zp;

	  // with respect to quaternion
	  jac[0][0] = 0.0; // fix w (so Q always normalize to 1)
	  jac[0][1] = E_X * Ox_a + E_Y * Oy_a + E_Z * Oz_a;
	  jac[0][2] = E_X * Ox_b + E_Y * Oy_b + E_Z * Oz_b;
	  jac[0][3] = E_X * Ox_c + E_Y * Oy_c + E_Z * Oz_c;

	  // with respect to Translation
	  jac[1][0] = -E_X*m00 - E_Y*m10 - E_Z*m20;
	  jac[1][1] = -E_X*m01 - E_Y*m11 - E_Z*m21;
	  jac[1][2] = -E_X*m02 - E_Y*m12 - E_Z*m22;

	  // with respect to sphere's Origin
	  jac[2][0] =  - jac[1][0];
	  jac[2][1] =  - jac[1][1];
	  jac[2][2] =  - jac[1][2];

	  i++;
	  printf("%2d, %u, res=%7.4lf E=[%7.4lf %7.4lf %7.4lf]\n", i, q < this->R, res[0], E_X, E_Y, E_Z);
	  //	  printf("%d, %u, res=%7.4lf jac=[%7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf %7.4lf\n", i, q < this->R, res[0], jac[0][0], jac[0][1], jac[0][2], jac[0][3], jac[1][0], jac[1][1], jac[1][2], jac[2][0], jac[2][1], jac[2][2]);

	  return true;
	}
      else
	return false;
    }
    
    static double R;
    double x,y,z,r;
  };
  double RayCostError::R = 0.0;

};

#endif
