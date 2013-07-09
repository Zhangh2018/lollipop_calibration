#pragma once

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include <vector>
#include <list>

template<typename T>
class ImageBlobExtractor
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;
public:
  // Input:
  // float search_radius: how far should be considered connected
  // int [min|max]_volumn: what size should be considered a blot
  ImageBlobExtractor(float focal_length, float search_radius,
		     float min_volumn, float  max_volumn,
		     int min_count,  int max_count);
  
  void setInputCloud(CloudPtr cloud);

  void extract(std::vector<pcl::PointIndices>& cluster_list, //output
	       std::vector<char>& mask); // input

private:
  const float f;         // focal length
  const float search_r;  // search radius
  float min_v, max_v;    // [min|max] volumn
  int min_c, max_c;      // [min|max] point count
  CloudPtr cloud;
};

template<typename T>
class OnePassBlobExtractor
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;
public:
  OnePassBlobExtractor(int width, int height, int min_count,  int max_count,
		       float min_volumn, float  max_volumn);

  void setInputCloud(CloudPtr cloud);

  void extract(std::vector<pcl::PointIndices>& cluster_list, //output
	       std::vector<char>& mask); // input

private:
  int horizontal_scan(int i, char label, std::vector<char>& mask);

  // Clockwise(counter-clockwise) trace the external(internal) contour
  void contour_trace(const int i, char init_dir, std::vector<char>& mask);
  CloudPtr cloud;
  const int W;
  const int H;
  const int min_c, max_c;
  const float min_v, max_v;
  std::vector<std::list<int> > q_list;
  int Dir2IdxOffset[8];
};

#include "impl/blob_extractor.cpp"
#include "impl/onepass_blob_extractor.cpp"
