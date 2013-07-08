
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>

template<class T>
class PCLStatsFilter  
{
  typedef typename pcl::PointCloud<T>::Ptr CloudPtr;
public: 
  PCLStatsFilter(int meanK, double threshold);
  
  void setInputCloud(CloudPtr c );

  void applyFilter(std::vector<char>& mask);
 
private:
  typename pcl::StatisticalOutlierRemoval<T> sor;
  CloudPtr cloud;
  unsigned int size;
};

#include "impl/filter.cpp"
