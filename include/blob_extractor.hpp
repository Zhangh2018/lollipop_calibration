#pragma once

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include <vector>
#include <list>

#include "connected.hpp"

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
  int horizontal_scan(const int i, char label, std::vector<char>& mask);

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

template<typename T>
class FastBlobExtractor
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;
public:
  FastBlobExtractor(int width, int height, int min_C,  int max_C,
		    float min_V, float  max_V)
    :W(width), H(height), min_c(min_C), max_c(max_C), min_v(min_V), max_v(max_V){};

  void setInputCloud(CloudPtr cld){cloud = cld;}

  void extract(std::vector<pcl::PointIndices>& cluster_list, //output
	       std::vector<char>& mask)
  {
    std::vector<char>* output  = new std::vector<char>(mask.size(), 0);

    ConnectedComponents cc(30);
    int nLabels = cc.connected(mask.data(), output->data(), W, H,
			       std::equal_to<unsigned char>(), true);

    cluster_list.resize(nLabels);
    std::vector<float> min_x(nLabels, 100.0f), min_y(nLabels, 100.0f), min_z(nLabels, 100.0f);
    std::vector<float> max_x(nLabels,-100.0f), max_y(nLabels,-100.0f), max_z(nLabels,-100.0f);
    for(int i=0; i< mask.size(); ++i)
      {
	char tag = output->at(i);
	cluster_list[tag].indices.push_back(i);

	T& pt= cloud->points[i];
	if(pt.x < min_x[tag]) min_x[tag] = pt.x;
	if(pt.y < min_y[tag]) min_y[tag] = pt.y;
	if(pt.z < min_z[tag]) min_z[tag] = pt.z;
	if(pt.x > max_x[tag]) max_x[tag] = pt.x;
	if(pt.y > max_y[tag]) max_y[tag] = pt.y;
	if(pt.z > max_z[tag]) max_z[tag] = pt.z;
      }

    for(int i=0; i< nLabels; ++i)
      {
	int s = cluster_list[i].indices.size();
	if( s < min_c || s >max_c)
	  {
	    cluster_list[i].indices.clear();
	    printf("Cluster[%d] rejected: size=%d\n", i, s);
	  }
	else
	  {
	    float v = (max_x[i]-min_x[i])*(max_y[i]-min_y[i])*(max_z[i]-min_z[i]);
	    if (v < min_v || v> max_v)
	      {
		cluster_list[i].indices.clear();
		printf("Cluster[%d] rejected: volume=%f\n", i, v);
	      }
	    printf("Cluster[%d] Added: size=%d volume=%f\n", i, s, v);
	  }
      }
    delete output;
  }

private:
  CloudPtr cloud;
  const int W;
  const int H;
  const int min_c, max_c;
  const float min_v, max_v;
};

#include "impl/blob_extractor.cpp"
#include "impl/onepass_blob_extractor.cpp"
