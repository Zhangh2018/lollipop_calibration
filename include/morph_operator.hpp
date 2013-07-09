
#pragma once

#include <pcl/point_cloud.h>
#include <vector>
 
namespace Morphology
{
  template <typename T>
  void Dilate2_5D(typename pcl::PointCloud<T>::Ptr cloud,
		  std::vector<char>& binaryImage,
		  float focal_length, float radius);

  template <typename T>
  void Erode2_5D(typename pcl::PointCloud<T>::Ptr cloud,
		 std::vector<char>& binaryImage,
		 float focal_length, float radius);
  
};

#include "impl/morph_operator.cpp"
