/*
  Test filter, background subtraction
 */

//#include "filter.hpp"
#include "background_filter.hpp"
#include "morph_operator.hpp"
#include "blob_extractor.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <sstream>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

int main(int argc, char** argv)
{
  if (argc< 3)
    {
      printf("Usage: %s [bg pcd] [fg pcd]\n", argv[0]);
      return -1;
    }

  // Load PCD file:
  CloudType::Ptr bg (new CloudType), raw(new CloudType);

  if (pcl::io::loadPCDFile<PointType> (argv[1], *bg) == -1 ||
      pcl::io::loadPCDFile<PointType> (argv[2], *raw) == -1)
    {
      PCL_ERROR ("Couldn't read file\n");
      return (-1);
    }

  ofstream ofs;
  // Test background subtraction
  std::vector<char> mask;
  ImageFilter<PointType> img_filter(0.5);
  img_filter.AddBackgroundCloud(bg);
  img_filter.GetForegroundMask(raw, mask);

  ofs.open("raw.dump");
  img_filter.WriteMaskToStream(ofs, mask);
  ofs.close();

  // Test morpholigical operators
  Morphology::Erode2_5D<PointType>(raw, mask, 525.0f, 0.03f);

  ofs.open("erode.dump");
  img_filter.WriteMaskToStream(ofs, mask);
  ofs.close();

  Morphology::Dilate2_5D<PointType>(raw, mask, 525.0f, 0.03f);

  ofs.open("dilate.dump");
  img_filter.WriteMaskToStream(ofs, mask);
  ofs.close();
 
 // Extract blobs
  std::vector<pcl::PointIndices> cluster_list;

  ImageBlobExtractor<PointType> ibe(525.0f, 0.06f, 1e-4, 0.125, 100, 100000);
  ibe.setInputCloud(raw);
  ibe.extract(cluster_list, mask);

  // Roughly where the center of each cluster is?
  for(int i=0; i< cluster_list.size(); ++i)
    {
      double r = .0f;
      double c = .0f;
      pcl::PointIndices& pi = cluster_list[i];

      CloudType::Ptr save (new CloudType);
      save->points.reserve(pi.indices.size());
      save->width = pi.indices.size();
      save->height= 1;

      for (int j=0; j < pi.indices.size(); ++j)
	{
	  int index = pi.indices[j];
	  r += (index / raw->width);
	  c += (index % raw->width);

	  save->points.push_back(raw->points[index]);
	}
      r /= pi.indices.size();
      c /= pi.indices.size();
      printf("Cluster %d has center at [%lf, %lf]\n", i, r,c);

      std::stringstream ss;
      ss << "c" << i << ".pcd" ;
      pcl::io::savePCDFileASCII (ss.str(), *save);
    }

  return 0;
}

/*


  PCLStatsFilter<PointType> stats_filter(20,1);
  stats_filter.setInputCloud(bg);
  stats_filter.applyFilter(mask);

*/

/*

*/
