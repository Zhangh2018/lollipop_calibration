#pragma once

#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>

template <typename T>
class ImageFilter
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;

public:
  ImageFilter(float threshold_ = 10.0f, float near = 1e-2, float far = 1e2);

  void AddBackgroundCloud(CloudPtr bg);

  void GetForegroundMask(CloudPtr fg, std::vector<char>& mask);

  // Debugging functions:
  void WriteMaskToStream(std::ostream& os, std::vector<char>& mask);

  void WriteMaskToFile(std::string str, std::vector<char>& mask);
private:
  std::vector<CloudPtr> bg_ptrs;
  //  std::vector<float> mean;
  //  std::vector<float> sigma;
  //  int cloud_size;
  const float threshold;
  const float near_cutoff, far_cutoff;
};

#include "impl/background_filter.cpp"
