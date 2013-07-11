
#include "linearTF_solver.hpp"

// Allocate space for points
LinearTfSolver::LinearTfSolver(unsigned int num_pts)
  :max_pts(num_pts), cur_pt(0), cloud_ref(3, num_pts), cloud_tgt(3, num_pts){}

void LinearTfSolver::AddPointPair(Eigen::Vector3d& ref, Eigen::Vector3d& tgt)
{
  if (cur_pt < max_pts)
    {
      cloud_ref.col(cur_pt) = ref;
      cloud_tgt.col(cur_pt) = tgt;
      ++cur_pt;
    }
}

void LinearTfSolver::EstimateTfSVD(double* origin)
{
  // Subtract the mean from the points
  Eigen::Vector3d mean_ref = cloud_ref.rowwise().mean();
  cloud_ref.colwise() -= mean_ref;
  Eigen::Vector3d mean_tgt = cloud_tgt.rowwise().mean();
  cloud_tgt.colwise() -= mean_tgt;

  // Assemble the correlation matrix H = target * reference'
  Eigen::Matrix3d H = cloud_tgt * cloud_ref.transpose ();
  
  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3d> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU ();
  Eigen::Matrix3d v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant() * v.determinant() < 0)
    {
      for (int i = 0; i < 3; ++i) 
	v (i, 2) *= -1;
    }
  
  //    std::cout<< "centroid_src: "<<centroid_src(0) <<" "<< centroid_src(1) <<" "<< centroid_src(2) << " "<< centroid_src(3)<<std::endl;
  //    std::cout<< "centroid_tgt: "<<centroid_tgt(0) <<" "<< centroid_tgt(1) <<" "<< centroid_tgt(2) << " "<< centroid_tgt(3)<<std::endl;
  
  Eigen::Quaterniond q(v * u.transpose());
  Eigen::Vector3d t = mean_ref - q * mean_tgt;

  // Explicitly copy (to prevent misalignment in memory, esp. in quaternion)
  origin[0] = t[0];
  origin[1] = t[1];
  origin[2] = t[2];
  origin[3] = q.w();
  origin[4] = q.x();
  origin[5] = q.y();
  origin[6] = q.z();
}
