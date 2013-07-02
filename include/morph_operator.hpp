
#include <pcl/point_types.h>

namespace Morphology
{
  void Dilate(std::vector<bool>& binaryImage, std::vector<float>& rangeImage, int width, int height);

  void Erode(std::vector<bool>& binaryImage, std::vector<float>& rangeImage, int width, int height);
};
