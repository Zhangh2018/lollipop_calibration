
#include "background_filter.hpp"

template <typename T>
ImageFilter<T>::ImageFilter(float _threshold, float far, float near)
  :threshold(_threshold), far_cutoff(far), near_cutoff(near)
{
}

template <typename T>
void ImageFilter<T>::AddBackgroundCloud(CloudPtr cloud)
{
  // TODO: make background model
  bg_ptrs.push_back(cloud);
}

template <typename T>
void ImageFilter<T>::GetForegroundMask(CloudPtr cloud, std::vector<char>& mask)
{
  // Make sure the size can match
  assert(cloud->size() == bg_ptrs.front()->size());

  // Set to all background
  mask.resize(cloud->size());
  std::fill(mask.begin(), mask.end(), 0x00);

  int count = 0;
  for (int i=0; i< mask.size(); ++i)
    {
      T& pb = bg_ptrs[0]->points[i];
      T& pf = cloud->points[i];

      if( pf.z > far_cutoff || pf.z < near_cutoff)
	continue;

      if (pf.z < pb.z - threshold || 
	  !isnan(pf.z) && isnan(pb.z))
	{
	  mask[i] = 0x01;
	  ++count;
	}
    }
  printf("%d/%lu remaining\n", count, cloud->size());
}


template <typename T>
void ImageFilter<T>::WriteMaskToStream(std::ostream& os, std::vector<char>& mask)
{
  int height = bg_ptrs.front()->height;
  int width  = bg_ptrs.front()->width;
  for(int i=0; i < height; ++i)
    {
      for(int j=0; j< width; ++j)
	os << static_cast<int>(mask[i*width+j]) << " ";

      os << std::endl;
    }
}
