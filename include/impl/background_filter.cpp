
#include "background_filter.hpp"

template <typename T>
ImageFilter<T>::ImageFilter(float _threshold)
  :threshold(_threshold)
{
}

template <typename T>
void ImageFilter<T>::AddBackgroundCloud(CloudPtr cloud)
{
  // TODO: make background model
  bg_ptrs.push_back(cloud);
}

template <typename T>
std::vector<char>& ImageFilter<T>::GetForegroundMask(CloudPtr cloud)
{
  // Make sure the size can match
  assert(cloud->size() == bg_ptrs.front()->size());

  // Set to all background
  mask.resize(cloud->size());
  std::fill(mask.begin(), mask.end(), 0x00);

  for (int i=0; i< mask.size(); ++i)
    {
      T& pb = bg_ptrs.front()->points[i];
      T& pf = cloud->points[i];

      if (pf.z < pb.z - threshold)
	mask[i] = 0x01;
    }
}

template <typename T>
void ImageFilter<T>::WriteMaskToStream(std::ostream& os)
{
  for(int i=0; i < bg_ptrs.front().height; ++i)
    {
      for(int j=0; j< bg_ptrs.front().width; ++j)
	os << static_cast<int>(mask[i]) << " ";

      os << std::endl;
    }
}
