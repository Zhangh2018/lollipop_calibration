
#include "morph_operator.hpp"

namespace Morphology
{
  template <typename T>
  void Dilate2_5D(pcl::PointCloud<T>& cloud, std::vector<char>& binaryImage,
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
		    int r = j+k;
		    int c = j+l;

		    if(r>=0 && r< cloud->height &&
		       c>=0 && c< cloud->width  &&
		       binrayImage[r*cloud->width+c] == 0)
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
	binrayImage[i] = binaryImage[i] & temp[i];
      }
  }


  template <typename T>
  void Erode2_5D(pcl::PointCloud<T>& cloud, std::vector<char>& binaryImage,
		 float f, float radius)
  {
    std::vector<char> temp(cloud->size(), 0);// start with all zeros

    for(int i=0; i< cloud->height; ++i)
      {
	for(int j=0; j < cloud->width; ++j)
	  {
	    int linear_idx = i*cloud->width+j;
	    
	    // Skip this pixel if it's 0
	    if (!binaryImage[linear_idx])
	      continue;

	    // If this pixel is 1, then all its neighbors are 1
	    T& p = cloud->points[linear_idx];
	    int pixel_radius = floor(f/p.z*radius);
	    for(int k=-pixel_radius; k<=pixel_radius; k++)
	      {
		for(int l=-pixel_radius; l<=pixel_radius; l++)
		  {
		    int r = i+k;
		    int c = j+l;
		    if (r>= 0 && r< cloud->height &&
			c>= 0 && c< cloud->width)
		      temp[r*cloud->width+c] = 0x01;
		  }
	      }
	  }
      }

    for (int i=0; i< binaryImage.size(); ++i)
      {
	binaryImage[i] = binaryImage[i] | temp[i];
      }
  }

};
