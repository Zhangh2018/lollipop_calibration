
#include "background_filter.hpp"
#include <vector>

ImageFilter::ImageFilter(float _max_range)
:max_range(_max_range)
{}

ImageFilter::AddBackgroundCloud(CloudT::Ptr bg)
{
  // If this is the first cloud, set the size
  if (bg_ptrs.empty())
    cloud_size = bg->size();
  else
    assert(bg->size() == cloud_size); // all must conform with the size

  bg_ptrs.push_back(bg);
}

ImageFilter::BuildBackgroundModel()
{
}

std::vector<bool>& ImageFilter::GetForegroundMask(CloudT::Ptr fg)
{
  // TODO: do filtering
  return mask;
}
