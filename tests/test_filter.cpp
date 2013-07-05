#include "filter.hpp"

int main(int argc, char** argv)
{
  //  PCLStatsFilter<int> filter(50,1.0);
  
  PCLStatsFilter<pcl::PointXYZ> a(5,1);
  return 1;
}
