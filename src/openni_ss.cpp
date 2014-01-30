/*
  Author: Ming Ruan
  Date: June 11, 2013

  This program opens up a OpenNI device (Xtion or Kinect) and displays
  the point cloud in the PCLVisualizer window.

  Press <S> to save in PCD format. Copy the saved PCDs to a another location 
  otherwise next time you run the program they will be overwritten by new ones

  Before starting the program, you may want to "killall Xn[tab]" to shut off 
  any dangling services
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#define DEFAULT  0
#define SAVE_PCD 1

bool myfunction (const pcl::PointXYZ& i, const pcl::PointXYZ& j) { return (i.z<j.z); }
class OpenNIViewer
{
  typedef pcl::visualization::PCLVisualizer Visualizer;
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
private:
  
  boost::shared_ptr<Visualizer> viewer;
  CloudType::ConstPtr _cloud;
  CloudType::Ptr _temp;
  std::vector<CloudType> aggr;

  int aggr_frames;
  int lower_quantile, upper_quantile, mean_quantile;
  int aggr_counter;
  int counter;
  int key_status;
  bool updateflag;
  boost::mutex updateMutex;

  void _cloud_cb (const CloudType::ConstPtr &cloud)
  {
    static bool first_time = true;
    if (viewer->wasStopped())
      return;

    boost::mutex::scoped_lock updateLock(updateMutex);
    _cloud = cloud;
    updateflag = true;
    updateLock.unlock();

    if (first_time)
      {
	for(int i=0; i< aggr.size();++i)
	  {
	    aggr[i].points.resize(cloud->size());
	    aggr[i].width = cloud->width;
	    aggr[i].height= cloud->height;
	  }
	_temp->points.resize(_cloud->size());
	_temp->width = _cloud->width;
	_temp->height= _cloud->height;
	first_time = false;
      }
  }

  void _key_cb(const pcl::visualization::KeyboardEvent &event)
  {
    //    Visualizer* pViewer = static_cast<Visualizer *> (view_void);
    if (event.getKeySym () == "s" && event.keyDown ())
      {
	key_status = SAVE_PCD;
      }
  }

  void aggregate_frames()
  {
    std::vector<pcl::PointXYZ> stats(aggr_frames);
    float x=.0f, y=.0f, z=.0f;
    // for each pixel
    //    printf("sizeof aggr[0] = %d\n", aggr[0].size());
    for(int i=0; i< aggr[0].size(); i++)
      {
	for(int j=0; j< aggr_frames; j++)
	  {
	    const PointType& p = aggr[j].points[i];
	    stats[j] = p;
	    if (isnan(p.z))
	      stats[j].z = 1e2;
	  }
	std::sort(stats.begin(), stats.end(), myfunction);

	x=.0f; y=.0f; z=.0f;
	for(int k= lower_quantile; k < upper_quantile; ++k)
	  {
	    x+= stats[k].x; y+= stats[k].y; z+= stats[k].z;
	  }
	PointType q(x/mean_quantile, y/mean_quantile, z/mean_quantile);
	_temp->points[i] = q;
      }
  }

  void copy_pointCloud(const CloudType& src, CloudType& des)
  {
    for (int i=0; i < src.size(); ++i)
      {
	des.points[i] = src.points[i];
      }
  }

public:
  OpenNIViewer(int _aggr_frames)
    :aggr_frames(_aggr_frames),aggr_counter(0), counter(0), key_status(0),
     _cloud(new CloudType), _temp(new CloudType), aggr(_aggr_frames),
     viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
  {
    lower_quantile = aggr_frames/4;
    upper_quantile = aggr_frames - lower_quantile;
    mean_quantile = upper_quantile - lower_quantile;

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters();
    //   viewer->setCameraPose(0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
    //    viewer->updateCamera ();
    viewer->addPointCloud<PointType>(_cloud, "cloud");
    
    boost::function<void (const pcl::visualization::KeyboardEvent&)> key_cb = boost::bind(&OpenNIViewer::_key_cb, this, _1);
    
    viewer->registerKeyboardCallback(key_cb);

    std::cout<< "viewer setup" << std::endl;
  }

  void run (std::string name)
  {
    
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    
    boost::function<void (const CloudType::ConstPtr&)> cloud_cb =
      boost::bind (&OpenNIViewer::_cloud_cb, this, _1);
    
    interface->registerCallback (cloud_cb);
    
    interface->start ();

    std::cout<< "Start frame grabber"<< std::endl;
    pcl::PCDWriter pcd_writer;
    while (!viewer->wasStopped())
      {
	viewer->spinOnce (100);
	// Main viewer loop

	if(updateflag)
	  { 				
	    updateflag = false;
	    if (key_status == SAVE_PCD)
	      {
		// Aggregate frames:
		boost::mutex::scoped_lock updateLock(updateMutex);
		this->copy_pointCloud(*_cloud, aggr[aggr_counter++]);
		updateLock.unlock();
		printf("Frame_aggregated: %d\r", aggr_counter);fflush(stdout);

		// Now aggregate over all frames and write to file
		if (aggr_counter == aggr_frames)
		  {
		    // reset aggr_counter;
		    aggr_counter = 0;

		    this->aggregate_frames();

		    std::stringstream ss;
		    ss << name <<"_"<<counter <<".pcd"; 
		    //		    pcl::io::savePCDFileASCII(ss.str(), *_cloud);
		    pcd_writer.writeBinary(ss.str(), *_temp);
		    //		    pcd_writer.writeASCII(ss.str(), *_temp);
		    std::cerr<<"\nSaved "<< _cloud->points.size() << " data points to: "<< ss.str() << std::endl;
		    
		    ++counter;
		    // Reset key_status to default
		    key_status = DEFAULT;
		  }
	      }
	    else
	      {
		boost::mutex::scoped_lock updateLock(updateMutex);
		// update viewer
		viewer->updatePointCloud(_cloud, "cloud");
		updateLock.unlock();
	      }
	  }

	boost::this_thread::sleep (boost::posix_time::microseconds (10000));
      }

    interface->stop ();
    delete interface;
  }
  
 };

int main (int argc, char** argv)
{
  //  SimpleOpenNIViewer v;
  OpenNIViewer v(16);
  if (argc < 2)
    {
      printf("Using default sensor name: x1\n");
      v.run("x1");
    }
  else
    {
      printf("Start sensor %s\n", argv[1]);
      v.run (argv[1]);
    }

  return 0;
}
