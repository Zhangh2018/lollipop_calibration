
#include "blob_extractor.hpp"

#include <list>
#include <cfloat>

// Math util functions to find the vector norm
template <typename T>
inline float L2_sqr(T& p, T& q)
{
  return (p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z);
}

template <typename T>
inline float L2_sqr(T& p)
{
  return (p.x*p.x+p.y*p.y+p.z*p.z);
}

template <typename T>
inline float L2_norm(T& p)
{
  return sqrt(L2_sqr(p));
}

template <typename T>
ImageBlobExtractor<T>::
ImageBlobExtractor(float _f, float r, 
		   float min_volumn, float max_volumn,
		   int min_count,  int max_count)
  :f(_f), search_r(r), min_v(min_volumn), max_v(max_volumn), 
   min_c(min_count), max_c(max_count)
{}

template <typename T>
void ImageBlobExtractor<T>::setInputCloud(typename pcl::PointCloud<T>::Ptr c)
{
  cloud = c;
}

template <typename T>
void ImageBlobExtractor<T>::extract(std::vector<pcl::PointIndices>& cluster_list,
				    std::vector<char>& mask)
{
  std::list<int> q_list; // stores current cluster indices
  char cluster_id = 2; // one above unprocessed foreground

  // Loop over all pixels
  for (int i=0; i< cloud->size(); ++i)
    {
      // Skip unless it is unprocessed foreground
      if (mask[i] != 1)
        continue;

      // Starting a new cluster
      q_list.clear();
      q_list.push_back(i);
      mask[i] = cluster_id;
      std::list<int>::iterator it = q_list.begin();

      // Use bounding box for volumn measurement
      // TODO: is there a better way to determine the volumn?
      float min_x, min_y, min_z;
      float max_x, max_y, max_z;
      min_x = min_y = min_z =  std::numeric_limits<double>::infinity();
      max_x = max_y = max_z = -std::numeric_limits<double>::infinity();

      while(it != q_list.end())
	{
          // Check the current node
          T& p = cloud->points[*it];

          // Convert linear index to row and column
          int r = i / cloud->width;
          int c = i % cloud->width;

          // The search window is a function of range
          int w = ceil(f / p.z * search_r);

	  // Search within the window
	  for(int r_ = r-w; r_ <= r+w; ++r_)
	    {
	      for(int c_ = c-w; c_ <=c+w; ++c_)
		{
		  int linear_idx = r_*cloud->width+c_;
		  //1. boundary check
		  //2. skip unless this pixel is unprocessed foreground
		  if (r_>=cloud->height || r_<0 ||
		      c_>=cloud->width  || c_<0 ||
		      mask[linear_idx] != 1)
		    continue;

		  T& q = cloud->points[linear_idx];
		  if(L2_sqr(q, p) < search_r*search_r)
                    {
		      // Add this point to Q_list
                      q_list.push_back(linear_idx);
		      mask[linear_idx] = cluster_id;
                      if(q.x < min_x) min_x = q.x;
                      if(q.y < min_y) min_y = q.y;
                      if(q.z < min_z) min_z = q.z;
		      if(q.x > max_x) max_x = q.x;
                      if(q.y > max_y) max_y = q.y;
                      if(q.z > max_z) max_z = q.z;
		    }
		}
	    }
	  // Move onto the next index in the Q_list
	  ++it;
	}//while(it != q_list.end());
      
      // Done flooding this cluster
      // If this cluster meet the size requirement, then add to cluster list
      float volumn = (max_x-min_x)*(max_y-min_y)*(max_z-min_z);
      if (volumn > min_v && volumn < max_v && 
	  q_list.size() > min_c && q_list.size() < max_c)
	{
	  pcl::PointIndices pi;
	  cluster_list.push_back(pi);
	  cluster_list.back().indices.resize(q_list.size());
	  std::list<int>::iterator   lit = q_list.begin();
	  std::vector<int>::iterator vit = cluster_list.back().indices.begin(); 
	  while(lit != q_list.end())
	    {
	      *vit = *lit;
	      ++vit; ++lit; // for clarity, let's not mix "++" with "*"
	    }

	  printf("Pixel %d, Cluster %d, volumn= %f, count = %lu\n", i, cluster_id, volumn, q_list.size()); 

	  // Increment cluster_id by 1
	  ++cluster_id;
	}
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
