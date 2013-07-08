#pragma once

#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>

template <typename T>
class ImageFilter
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;

public:
  ImageFilter(float threshold_ = 10.0f);

  void AddBackgroundCloud(CloudPtr bg);

  std::vector<char>& GetForegroundMask(CloudPtr fg);

  // Debugging functions:
  WriteMaskToStream(std::ostream& os);

private:
  std::vector<CloudPtr> bg_ptrs;
  std::vector<char> mask;
  //  std::vector<float> mean;
  //  std::vector<float> sigma;
  //  int cloud_size;
  const float threshold;
};
