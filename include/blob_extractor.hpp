#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ImageBlobExtractor
{
public:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> CloudT;

  // Input:
  // float search_radius: how far should be considered connected
  // int [min|max]_volumn: what size should be considered a blot
  ImageBlobExtractor(float focal_length, float search_radius,
                        int min_volumn, int max_volumn);

  void SetInputCloud(CloudT::Ptr cloud, std::vector<bool>& mask);

  std::vector<std::vector<int> >& GetBlobsIndices();

private:
  float f;
  float r2; // r^2, where r is the search radius
  int min_v, max_v;
};
