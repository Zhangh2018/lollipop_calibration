
#include "morph_operator.hpp"

namespace Morphology
{
  template <typename T>
  inline float L2_sqr(T& p, T& q)
  {
    return (p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z);
  }

  template <typename T>
  void Erode2_5D(typename pcl::PointCloud<T>::Ptr cloud, 
		 std::vector<char>& binaryImage,
		 float f, float radius)
  {
    std::vector<char> temp(binaryImage.size(), 0x01);

    for(int i=0; i < cloud->height; i++)
      {
	for(int j=0; j < cloud->width; j++)
	  {
	    int linear_idx = i*cloud->width+j;
	    // If it's already 0, leave it that way
	    if(!binaryImage[linear_idx])
	      continue;

	    T& p = cloud->points[linear_idx];
	    // Otherwise, if any of its neighbor is 0, it becomes 0
	    int pixel_radius = floor(f/p.z*radius);
	    bool break_flag = false;
	    for(int k=-pixel_radius; k<= pixel_radius; ++k)
	      {
		for(int l=-pixel_radius; l<=pixel_radius; ++l)
		  {
		    int r = i+k;
		    int c = j+l;

		    if(r>=0 && r< cloud->height &&
		       c>=0 && c< cloud->width  &&
		       binaryImage[r*cloud->width+c] == 0)
		      {
			temp[linear_idx] = 0x00;
			break_flag = true;
			break;
		      }
		  }
		if (break_flag)
		  {
		    break_flag = false;
		    break;
		  }
	      }
	  }
      }
    
    // The final binary image is AND of the two
    for (int i=0; i < temp.size(); ++i)
      {
	binaryImage[i] = binaryImage[i] & temp[i];
      }
  }


  template <typename T>
  void Dilate2_5D(typename pcl::PointCloud<T>::Ptr cloud, 
		 std::vector<char>& binaryImage,
		 float f, float radius)
  {
    std::vector<char> temp(cloud->size(), 0);// start with all zeros

    for(int i=0; i< cloud->size(); ++i)
      {
	// Skip this pixel if it's 0
	if (!binaryImage[i])
	  continue;

	int row = i / cloud->width;
	int col = i % cloud->width;
	    
	// If this pixel is 1, then all its neighbors are 1
	T& p = cloud->points[i];
	int pixel_radius = floor(f/p.z*radius);
	for(int j=-pixel_radius; j<=pixel_radius; j++)
	  {
	    for(int k=-pixel_radius; k<=pixel_radius; k++)
	      {
		int r = row + j;
		int c = col + k;
		int linear_idx = r*cloud->width + c;
		// 1. Boundary check
		// 2. Don't double count
		if (r < 0 || r >= cloud->height || 
		    c < 0 || c >= cloud->width  ||
		    binaryImage[linear_idx])
		  continue;
		
		T& q = cloud->points[linear_idx];
		if (L2_sqr(p, q) < radius*radius)
		  temp[linear_idx] = 0x01;
	      }
	  }
      }

    for (int i=0; i< binaryImage.size(); ++i)
      {
	binaryImage[i] = binaryImage[i] | temp[i];
      }
  }

};
