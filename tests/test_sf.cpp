
#include "sphere_fitter.hpp"

#include <pcl/point_types.h>

int main(int argc, char** argv)
{
  SphereFitter<pcl::PointXYZ>* sf = new LinearFitter<pcl::PointXYZ>(0.1275);

  delete sf;
}
