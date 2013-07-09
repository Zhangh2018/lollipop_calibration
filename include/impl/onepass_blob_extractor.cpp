
#include "blob_extractor.hpp"

template<typename T>
OnePassBlobExtractor<T>::OnePassBlobExtractor(int width, int height, 
					      int min_C, int max_C,
					      float min_V, float max_V)
  :W(width), H(height), min_c(min_C), max_c(max_C), min_v(min_V), max_v(max_V)
{
  Dir2IdxOffset[0] =   1; Dir2IdxOffset[1] = W+1; Dir2IdxOffset[2] = W;
  Dir2IdxOffset[3] = W-1; Dir2IdxOffset[4] =  -1; Dir2IdxOffset[5] =-W-1;
  Dir2IdxOffset[6] =-W  ; Dir2IdxOffset[7] =-W+1;
}

template<typename T>
void OnePassBlobExtractor<T>::extract(std::vector<pcl::PointIndices>& cluster_list,
				      std::vector<char>& mask) // input
{
  char label = 2;

  // start from 2nd row, ends on second last row
  for(int i= W; i<mask.size()-W; i++)
    {
      char P = mask[i];
      switch (P)
	{
	case 0:
	  break;
	case 1:
	  // always start searching from EAST
	  mask[i] = label++;
	  boost::shared_ptr<std::list<int> > list_ptr(new std::list<int>);
	  list_ptr->push_back(i);
	  q_list.push_back(list_ptr);
	  contour_trace(i, label, 0, mask);
	  break;
	default:
	  i = horizontal_scan(i, label, mask);
	  break;
	}
    }

  // move all to indices list
  cluster_list.reserve(q_list.size());
  float min_x = 1e5, min_y = 1e5, min_z = 1e5;
  float max_x =-1e5, max_y =-1e5, max_z =-1e5;
  for (int j=0; j < q_list.size(); ++j)
    {
      int count = q_list[j]->size();
      if (count > min_c && count < max_c)
	{
	  std::list<int>::iterator it = q_list[j]->begin();
	  pcl::PointIndices pi;
	  pi.indices.reserve(count);// pre-allocate space
	  while (it != q_list[j]->end())
	    {
	      T& p = cloud->points[*it];
	      
	      pi.indices.push_back(*it);
	      
	      if(p.x < min_x) min_x = p.x;
	      if(p.y < min_y) min_y = p.y;
	      if(p.z < min_z) min_z = p.z;
	      if(p.x > max_x) max_x = p.x;
	      if(p.y > max_y) max_y = p.y;
	      if(p.z > max_z) max_z = p.z;
	      ++it;
	    }

	  float volumn = (max_x-min_x)*(max_y-min_y)*(max_z-min_z);
	  if (volumn > min_v && volumn < max_v)
	    cluster_list.emplace_back(pi);
	}
    }
}

template<typename T>
int OnePassBlobExtractor<T>::horizontal_scan(int i, char label, 
					     std::vector<char>& mask)
{
  while(true)
    {
      int right = i+1;
  
      // End of Row
      if (right % W == 0) 
	break;

      // Hit the other end of line scan
      if (mask[right] == label)
	{
	  i = right;
	  break;
	}

      // This is internal contour
      if (mask[right] == 0x00)
	{
	  contour_trace(i, 0x07, label, mask);
	  break;
	}

      // Unlabeled pixel
      if (mask[right] == 0x01)
	{
	  mask[right] = label;
	  q_list[label-2]->push_back(right);
	  i = right;
	}
    }
  return i;
}

template<typename T>
void OnePassBlobExtractor<T>::contour_trace(const int i, char Dir, char label,
					    std::vector<char>& mask)
{
  int j = i;
  // TODO: boundary check
  while(true)
    {
      int k;
      k = j + Dir2IdxOffset[Dir];
      while(mask[k] == 0)
	{
	  Dir = (Dir+1) & 0x07; // same as mod by 8;
	  k = j + Dir2IdxOffset[Dir];
	}
      j = k;

      // End of the contour
      if(j==i)
	break;

      // Copy the label
      mask[j] = label;
      q_list[label-2]->push_back(j);

      // From where to start searching next time?
      // If Dir is 1 3 5 7->Diagonals
      if(Dir & 0x01)
	Dir = (Dir+6) & 0x07;
      else
	Dir = (Dir+7) & 0x07;
    }
}
