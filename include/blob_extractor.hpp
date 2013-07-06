#pragma once

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
template<typename T>

class ImageBlobExtractor
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;
public:
  // Input:
  // float search_radius: how far should be considered connected
  // int [min|max]_volumn: what size should be considered a blot
  ImageBlobExtractor(float focal_length, float search_radius,
		     int min_volumn, int max_volumn,
		     int min_count,  int max_count);
  
  void setInputCloud(CloudPtr cloud);

  void extract(std::vector<pcl::PointIndices>& cluster_list, //output
	       std::vector<char>& mask); // input

private:
  const float f;         // focal length
  const float search_r;  // search radius
  int min_v, max_v;      // [min|max] volumn
  int min_c, max_c;      // [min|max] point count
  CloudPtr cloud;
};
