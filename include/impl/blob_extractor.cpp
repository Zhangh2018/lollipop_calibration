
#include "blob_extractor.hpp"

#include <queue>
#include <cfloat>

// Math util functions to find the vector norm
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

void setInputCloud(CloudT::Ptr c)
{
  cloud = c;
}

void extract(std::vector<pcl::PointIndices>& cluster_list, std::vector<char>& mask)
{
  std::vector<char> label(mask);
  char label_count = 0x01;
  std::vector<char> equiv(1);
  // Skip first and last row
  for(int i = cloud->width+1; i< mask.size()-cloud->width-1; ++i)
    {
      int west = i-1;
      if (mask[west] == mask[i])
	label[i];
      else if (mask[west]==mask[i] && mask[north]==mask[i] && label[west]!=label[north])
	{
	  label[i] = std::min(label[west], label[north]);
	  // TODO: left off from here last time
	  char min_equiv = std::min();
	}
      // if this is the second last pixel of the row
      if ((i+1) % cloud->width ==0)
	i +=2; // move on the next row
    }
}


void extract(std::vector<pcl::PointIndices>& cluster_list, std::vector<bool>& mask)
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

      // Use bounding box for volumn measurement
      // TODO: is there a better way to determine the volumn?
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
          int w = static_cast<int>(floor(f / L2_norm(p) * r));

	  // Set this pixel to "processed"
	  processed[j] = true;

	  // Only flood from current row downward
	   min_x = p.x; min_y = p.y; min_z = p.z;
	   max_x = p.x; max_y = p.y; max_z = p.z;
	  for(int r_ = r+w; r_ >= r; --r_)
	    {
	      bool left_flag = false;
	      bool right_flag= false;
	      for(int c_ = c+w; c_ >=0; --c_)
		{
		  int linear_idx = r_*cloud->height+c_;
		  if (r_>cloud->height || c_>cloud->width ||
		      !mask[linera_idx]|| processed[linear_idx])
		    continue;

		  PointT q& = cloud->points[linear_idx];
		  if(L2_sqr(q-p) < r2)
                    {
                      q_list.push_back(linear_idx);
                      if(q.z < min_z) min_z = q.z;
		      if(q.x > max_x) max_x = q.x;
                      if(q.y > max_y) max_y = q.y;
                      if(q.z > max_z) max_z = q.z;
		    }
		}

	      for(int c_ = c+w; c_ >=0; --c_)
		{
		  int linear_idx = r_*cloud->height+c_;
		  if (r_>cloud->height || c_>cloud->width ||
		      !mask[linera_idx]|| processed[linear_idx])
		    continue;

		  PointT q& = cloud->points[linear_idx];
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
          for(int r_ = r-w; r_ <= r+w; ++r)
            {
              for(int c_ = c-w; c_ <= c+w; ++c)
                {
                  // skip if it's marked as background or has been processed
                  int linear_idx = r_*cloud->height+c_;
                  if(r_<0 || r_>cloud->height ||
                     c_<0 || c_>cloud->width  || // must be within boundary
		     !mask[linear_idx]     || // don't process background 
		     processed[linear_idx])   // don't double count 
                    continue;

                  PointT q& = cloud->points[linear_idx];
                  // Check if this point is within range
                  if(L2_sqr(q-p) < r2)
                    {
                      q_list.push_back(linear_idx);

		      // Update bounding box
                      if(q.x < min_x) min_x = q.x;
                      if(q.y < min_y) min_y = q.y;
                      if(q.z < min_z) min_z = q.z;
                      if(q.x > max_x) max_x = q.x;
                      if(q.y > max_y) max_y = q.y;
                      if(q.z > max_z) max_z = q.z;		    
                    }
                }
            }
	  // Add this point to inlier indices
	  pi.indices.push_back(j);
        }//END OF WHILE

      // If this cluster meet the size requirement, then add to cluster list
      float volumn = (max_x-min_x)*(max_y-min_y)*(max_z-min_z);
      if (volumn > min_v && volumn < max_v && !pi.indices.empty())
        cluster_list.push_back(pi);
    }
}

/*
void extract(std::vector<pcl::PointIndices>& cluster_list)
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

      // Use bounding box for volumn measurement
      // TODO: is there a better way to determine the volumn?
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

	  // Set this pixel to "processed"
	  processed[j] = true;

          for(int r_ = r-w; r_ <= r+w; ++r)
            {
              for(int c_ = c-w; c_ <= c+w; ++c)
                {
                  // skip if it's marked as background or has been processed
                  int linear_idx = r_*cloud->height+c_;
                  if(r_<0 || r_>cloud->height ||
                     c_<0 || c_>cloud->width  || // must be within boundary
		     !mask[linear_idx]     || // don't process background 
		     processed[linear_idx])   // don't double count 
                    continue;

                  PointT q& = cloud->points[linear_idx];
                  // Check if this point is within range
                  if(L2_sqr(q-p) < r2)
                    {
                      q_list.push_back(linear_idx);

		      // Update bounding box
                      if(q.x < min_x) min_x = q.x;
                      if(q.y < min_y) min_y = q.y;
                      if(q.z < min_z) min_z = q.z;
                      if(q.x > max_x) max_x = q.x;
                      if(q.y > max_y) max_y = q.y;
                      if(q.z > max_z) max_z = q.z;		    
                    }
                }
            }
	  // Add this point to inlier indices
	  pi.indices.push_back(j);
        }//END OF WHILE

      // If this cluster meet the size requirement, then add to cluster list
      float volumn = (max_x-min_x)*(max_y-min_y)*(max_z-min_z);
      if (volumn > min_v && volumn < max_v && !pi.indices.empty())
        cluster_list.push_back(pi);
    }
}

 */
