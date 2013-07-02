
#include "blob_extractor.hpp"

#include <queue>
#include <cfloat>

extern const float g_radius;


float inline L2_sqr(PointT& p)
{
  return (p.x*p.x+p.y*p.y+p.z*p.z);
}

float inline L2_norm(PointT& p)
{
  return sqrt(L2_sqr(p));
}

ImageBlobExtractor::ImageBlobExtractor(float _f, float r, int min_volumn, int max_volumn)
:f(_f), r2(r*r), min_v(min_volumn), max_v(max_volumn)
{}

void setInputCloud(CloudT::Ptr cloud, std::vector<bool>& mask)
{

}

void extract(pcl::PointIndices& cluster_list)
{
  std::queue<int> q_list;
  std::vector<bool> processed(mask.size(), 0);

  for (int i=0; i< cloud->size(); ++i)
    {
      // If it is background or has been processed, then skip it.
      if (!mask[i] || processed[i])
        continue;

      // Add this point to Q_list
      q_list.push_back(i);

      float min_x, min_y, min_z;
      float max_x, max_y, max_z;
      min_x = min_y = min_z = -std::numeric_limits<double>::infinity();
      max_x = max_y = max_z =  std::numeric_limits<double>::infinity();

      // Pi holds all the indices for this cluster
      pcl::PointIndices pi;

      while(!q_list.empty())
        {
          // Take out the current head
          int j = q_list.front();
          q_list.pop();

          PointT p& = cloud->points[j];

          // Convert linear index to row and column
          int r = j / cloud->width;
          int c = j % cloud->width;
          // The search window is a function of range
          int w = static_cast<int>(floor(f / L2_norm(p) * g_radius));

          for(int r_ = r-w; r_ <= r+w; ++r)
            {
              for(int c_ = c-w; c_ <= c+w; ++c)
                {
                  // skip if it's marked as background or has been processed
                  int linear_idx = r_*cloud->height+c_;
                  if(!mask[linear_idx] || processed[linear_idx] ||
                     r_<0 || r_>cloud->height ||
                     c_<0 || c_>cloud->width)
                    continue;

                  PointT q& = cloud->points[linear_idx];
                  // Check if this point is
                  if(L2_sqr(q-p) < r2)
                    {
                      q_list.push_back(linear_idx);

                      if(q.x < min_x) min_x = q.x;
                      if(q.y < min_y) min_y = q.y;
                      if(q.z < min_z) min_z = q.z;
                      if(q.x > max_x) max_x = q.x;
                      if(q.y > max_y) max_y = q.y;
                      if(q.z > max_z) max_z = q.z;
                    }
                }
            }
          // Add this point to inlier indices and set it to "processed"
          processed[j] = true;
          pi.indices.push_back(j);
        }
      // If this cluster meet the size requirement, then add to cluster list
      float volumn = (max_x-min_x)*(max_y-min_y)*(max_z-min_z);
      if (volumn > min_v && volumn < max_v && !pi.indices.empty())
        cluster_list.push_back(pi);
    }
}
