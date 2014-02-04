
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <yaml-cpp/yaml.h>

#include "pcl_utils.hpp"
#include "sphere_fitter.hpp"

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
SetupViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	    pcl::ModelCoefficients::Ptr coeff)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // black background
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  //  viewer->initCameraParameters ();
  viewer->setCameraPosition(coeff->values[0], coeff->values[1], coeff->values[2],
			    0.0, -1.0, 0.0);

  //  pcl::PointXYZ p(coeff.values[0], coeff.values[1], coeff.values[2], 
  viewer->addSphere(*coeff);

  return (viewer);
}

pcl::PointIndices::Ptr
RansacMethod(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudp, pcl::ModelCoefficients::Ptr coeff)
{
  //Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inlier (new pcl::PointIndices ());

  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  seg.setRadiusLimits(0.20315, 0.20325);

  seg.setInputCloud(cloudp);
  seg.segment(*inlier, *coeff);

  return inlier;
}

void RayCostMethod(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coeff, double radius)
{
  Eigen::Vector3d center;
  center << coeff->values[0], coeff->values[1], coeff->values[2];

  NonlinearFitter<pcl::PointXYZ> nlsf(radius);
  nlsf.SetInputCloud(cloud);
  // Notice this is a different kind of cost, not comparable to linear fit cost
  nlsf.ComputeFitCost(center);
}


int main(int argc, char** argv)
{
  if (argc < 2)
    {
      PCL_ERROR("Usage: %s gt_pcd.yaml output.yaml\n", argv[0]);
      exit(1);
    }

   // Load the config file
  YAML::Node config = YAML::LoadFile(argv[1]);
  std::string ofs_name(argc==3? argv[2]:"gt.yaml");
  std::ofstream os(ofs_name);

  PCL_INFO("Output result to %s\n", ofs_name.c_str());
  const double target_radius    = config["target_ball_radius"].as<double>();
  const std::string glob_prefix = config["global_pcd_prefix"].as<std::string>();

  const YAML::Node& pcd_node = config["PCDs"];

  PCL_INFO("%d files in the config\n", pcd_node.size());

  os << "GroundTruth:"<< std::endl;
  for(int i=0; i < pcd_node.size(); ++i)
    {
      std::string filename = pcd_node[i].as<std::string>();
      std::string fullpath = glob_prefix + filename;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (fullpath, *cloudp) == -1)
	{
	  PCL_ERROR("Failed to load file %s\n", fullpath.c_str());
	  exit(1);
	}
      else
	PCL_WARN("Loaded PCD %s\n", fullpath.c_str());

      pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inlier = RansacMethod(cloudp, coeff);
      // RaycostMethod(cloudp, center, target_radius);

      printf("%lf %lf %lf %lf\n",  coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);

      os<< "  - ["<< i <<", "<< coeff->values[0] <<", "<< coeff->values[1]<<", "<<coeff->values[2]<<", "<<coeff->values[3]<<"]"<<std::endl;

#if 0
      // Separate and colorize inliers and outliers
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clr_cloud = ColorizeCloud(cloudp, inlier);

      // Display the final result
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      viewer = SetupViewer(clr_cloud, coeff);

      while( !viewer->wasStopped())
	{
	  viewer->spinOnce(100);
	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
#endif

    }
  os.close();
  return 0;
}
