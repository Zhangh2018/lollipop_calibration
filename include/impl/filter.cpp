
#include "filter.hpp"

template <class T>
PCLStatsFilter<T>::PCLStatsFilter(int meanK, double threshold)
{
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(threshold);
}

template <class T>
void PCLStatsFilter<T>::setInputCloud(CloudPtr c )
{
  cloud = c; size = c->size();
  sor.setInputCloud(c);
}

template <class T>
void PCLStatsFilter<T>::applyFilter(std::vector<char>& mask)
{
  mask.resize(size);
  std::fill(mask.begin(), mask.end(), 0x00);
  
  std::vector<int> indices;
  sor.filter(indices);
  for(int i=0; i< indices.size(); ++i)
    mask[indices[i]] = 0x01;

  printf("%lu//%lu remaining\n", indices.size(), cloud->size());
}
