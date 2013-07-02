#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// The global variable radius
extern const float g_radius;

namespace SphereFitter
{
  class LinearFitter
  {
    void SetInputCloud(CloudT::Ptr input, std::vector<bool>& mask);

    void SetInputCloud(CloudT::Ptr input, std::vector<int>& inliers);

    // Returns a score to indicate how well the point cloud
    // fits the underlining sphere
    float GetFitCost();
  };

  class NonlinearFitter
  {
    void SetInputCloud(CloudT::Ptr input, std::vector<bool>& mask);

    void SetInputCloud(CloudT::Ptr input, std::vector<int>& inliers);

    // Returns a score to indicate how well the point cloud
    // fits the underlining sphere (i.e. the cost)
    float GetFitCost();
  };
};
