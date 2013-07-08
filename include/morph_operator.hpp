
#pragma once

#include <pcl/point_cloud.h>
#include <vector>

namespace Morphology
{
  template <typename T>
  void Dilate2_5D(pcl::PointCloud<T>& cloud,
		  std::vector<char>& binaryImage,
		  float focal_length, float radius);

  template <typename T>
  void Erode2_5D(pcl::PointCloud<T>& cloud,
		 std::vector<char>& binaryImage,
		 float focal_length, float radius);
  
};

#include "imp/morph_operator.cpp"
