
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "pcl_utils.hpp"
#include "sphere_fitter.hpp"

#define TARGET_RADIUS 0.204

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

void RayCostMethod(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coeff)
{
  Eigen::Vector3d center;
  center << coeff->values[0], coeff->values[1], coeff->values[2];

  NonlinearFitter<pcl::PointXYZ> nlsf(TARGET_RADIUS);
  nlsf.SetInputCloud(cloud);
  // Notice this is a different kind of cost, not comparable to linear fit cost
  nlsf.ComputeFitCost(center);
}


int main(int argc, char** argv)
{
  if (argc < 2)
    {
      PCL_ERROR("Usage: %s gt.pcd\n", argv[0]);
      exit(1);
    }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloudp) == -1)
    {
      PCL_ERROR("Failed to load file %s\n", argv[1]);
      exit(1);
    }
  else
    PCL_WARN("Loaded PCD %s\n", argv[1]);

  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inlier = RansacMethod(cloudp, coeff);
  // RaycostMethod(cloudp, center);

  printf("%lf %lf %lf %lf\n",  coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);

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
  return 0;
}
