#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ImageFilter
{
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> CloudT;

public:
  ImageFilter(float _max_range=10.0f);

  void AddBackgroundCloud(CloudT::Ptr bg);

  std::vector<bool>& GetForegroundMask(CloudT::Ptr fg);

private:
  std::vector<CloudT::Ptr> bg_ptrs;
  std::vector<bool> mask;
  std::vector<float> mean;
  std::vector<float> sigma;
  int cloud_size;
  const float max_range;
};
