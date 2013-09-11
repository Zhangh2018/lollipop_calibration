#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <algorithm> // for std::sort

 // pack r/g/b into rgb
static int red  = ((int)255) << 16 | ((int)0)   << 8 | ((int)0);
static int green= ((int)0)   << 16 | ((int)255) << 8 | ((int)0);
static int blue = ((int)0)   << 16 | ((int)0)   << 8 | ((int)255);

/*
uint32_t red = (static_cast<uint32_t>(255) << 16 |
		static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));

uint32_t blue= (static_cast<uint32_t>(0) << 16 |
		static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(255));
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
ColorizeCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
	      const std::vector<bool>& mask)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ccl(new pcl::PointCloud<pcl::PointXYZRGB>);
  ccl->points.resize(mask.size());
  ccl->width = cloud->width;
  ccl->height = cloud->height;
  ccl->is_dense = cloud->is_dense;

  int inlier = 0;
  for(int i=0; i< mask.size(); ++i)
    {
      if (mask[i])
	{
	  ccl->points[i].rgb = *reinterpret_cast<float*>(&red);
	  inlier++;
	}
      else
	{
	  ccl->points[i].rgb = *reinterpret_cast<float*>(&blue);
	}
      ccl->points[i].x = cloud->points[i].x;
      ccl->points[i].y = cloud->points[i].y;
      ccl->points[i].z = cloud->points[i].z;
    }
  return ccl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
ColorizeCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
	      std::vector<int>& indices)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  output->points.resize(input->points.size());
  output->width  = input->width;
  output->height = input->height;
  output->is_dense = input->is_dense;

  int numInlier = 0, numOutlier = 0;
  // In case it wasn't sorted
  std::sort(indices.begin(), indices.end());
  size_t j=0;
  for(size_t i=0; i< input->points.size(); ++i)
    {
      if (i==indices[j])
	{
	  output->points[i].rgb = *reinterpret_cast<float*>(&red);
	  ++j;
	  numInlier++;
	}
      else
	{
	  output->points[i].rgb = *reinterpret_cast<float*>(&blue);
	  numOutlier++;
	}
      
      output->points[i].x = input->points[i].x;
      output->points[i].y = input->points[i].y;
      output->points[i].z = input->points[i].z;
    }
  //  std::cout << "Num inliers="<< numInlier << ", Num outliers="<< numOutlier<<std::endl;
  return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
ColorizeCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
	      pcl::PointIndices::Ptr inlierIdx)
{
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output;
  return ColorizeCloud(input, inlierIdx->indices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
ColorizeCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
	      std::string color)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

  output->points.resize(input->points.size());
  output->width  = input->width;
  output->height = input->height;
  output->is_dense = input->is_dense;

  float clr;
  if (color == "red")
    clr = *reinterpret_cast<float*>(&red);
  else if (color == "green")
    clr = *reinterpret_cast<float*>(&green);
  else
    clr = *reinterpret_cast<float*>(&blue);

  for(size_t i=0; i< input->points.size(); ++i)
    {
      output->points[i].rgb = clr;
      output->points[i].x = input->points[i].x;
      output->points[i].y = input->points[i].y;
      output->points[i].z = input->points[i].z;
    }

  return output;
}
