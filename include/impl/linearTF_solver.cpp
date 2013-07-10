
#include "transform_solver.hpp"


void LinearTfSolver::SetRefPoints(std::vector<Eigen::Vector3d>& ref_points)
{
  compute3DCentroid(ref_points, centroid_ref);
    
  demeanPointCloud(ref_points, centroid_ref, cloud_ref_demean);
}

void LinearTfSolver::SetTgtPoints(std::vector<Eigen::Vector3d>& tgt_points)
{
  compute3DCentroid(tgt_points, centroid_tgt);
    
  demeanPointCloud(tgt_points, centroid_tgt, cloud_tgt_demean);
}

void LinearTfSolver::EstimateTfSVD(Eigen::Vector3d& t, Eigen::Quaterniond& q)
{
  // Assemble the correlation matrix H = target * reference'
  Eigen::Matrix3d H = (cloud_tgt_demean * cloud_ref_demean.transpose ()).topLeftCorner<3, 3>();
  
  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3d> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU ();
  Eigen::Matrix3d v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
    {
      for (int i = 0; i < 3; ++i)
	v (i, 2) *= -1;
      }
  
  //    std::cout<< "centroid_src: "<<centroid_src(0) <<" "<< centroid_src(1) <<" "<< centroid_src(2) << " "<< centroid_src(3)<<std::endl;
  //    std::cout<< "centroid_tgt: "<<centroid_tgt(0) <<" "<< centroid_tgt(1) <<" "<< centroid_tgt(2) << " "<< centroid_tgt(3)<<std::endl;
  
  Eigen::Matrix3d R = v * u.transpose ();
  Eigen::Quaterniond Q(R);
  q = Q;
  const Eigen::Vector3d Rc (R * centroid_tgt.head<3> ());
  t = centroid_ref.head<3> () - Rc;
}
