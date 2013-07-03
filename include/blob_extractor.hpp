#pragma once

#include <pcl/point_cloud.h>

template<typename T>
class ImageBlobExtractor
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;
public:
  // Input:
  // float search_radius: how far should be considered connected
  // int [min|max]_volumn: what size should be considered a blot
  ImageBlobExtractor(float focal_length, float search_radius,
		     int min_volumn, int max_volumn);
  
  void setInputCloud(CloudPtr cloud);

  void extract(std::vector<pcl::PointIndices> cluster_list);

private:
  float f;  // focal length
  float r;  // search radius
  int min_v, max_v;
  CloudPtr cloud;
};
