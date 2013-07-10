
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

class LinearTfSolver
{
public:
  void SetRefPoints(std::vector<Eigen::Vector3d>& ref_points);

  void SetTgtPoints(std::vector<Eigen::Vector3d>& tgt_points);

  void EstimateTfSVD(Eigen::Vector3d& t, Eigen::Quaterniond& q);

private:
  bool compute3DCentroid(std::vector<Eigen::Vector3d>& cloud, Eigen::Vector4d& centroid);

  bool demeanPointCloud(std::vector<Eigen::Vector3d>& cloud, Eigen::Vector4d& centroid,
			Eigen::MatrixXd& cloud_out);


  Eigen::Vector4d centroid_ref;
  Eigen::Vector4d centroid_tgt;
  Eigen::MatrixXd cloud_ref_demean;
  Eigen::MatrixXd cloud_tgt_demean;
};
