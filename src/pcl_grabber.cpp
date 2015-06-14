#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni_grabber.h>
#include "PCDGrabberExt.h"
#include "PCDWriterExt.h"
//#include <pcl/io/ensenso_grabber.h>
//#include "kinect2_grabber.h"
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/time.h> //fps calculations

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
	    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
	    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

class SimpleOpenNIViewer
{
public:

	SimpleOpenNIViewer() : viewer("PCL Grabber") {}

	void image_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
			viewer.showCloud(cloud);

//		FPS_CALC("cloud callback");


	}

	void run()
	{
		//		pcl::Grabber* grabber = new pcl::OpenNIGrabber();
		pcl::Grabber* grabber = new pcl::io::OpenNI2Grabber();
//		pcl::Grabber* grabber = new pcl::EnsensoGrabber();
//  		((pcl::EnsensoGrabber*)grabber)->openTcpPort();
//		((pcl::EnsensoGrabber*)grabber)->openDevice();
//		pcl::Grabber* grabber = new pcl::Kinect2Grabber();
//		pcl::Grabber* grabber = pcl::PCDGrabberExt<pcl::PointXYZ>(".\\data\\20150613T212929\\data.tar", 30);
		pcl::PCDWriterExt writer;
		
		boost::function<void(const boost::shared_ptr<pcl::io::DepthImage>&)> f_3 =
			boost::bind(&pcl::PCDWriterExt::WriteLZFDepthImage, &writer, _1);

		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f_1 =
			boost::bind(&pcl::PCDWriterExt::WritePCDCloud<pcl::PointXYZ>, &writer, _1);

		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f_2 =
			boost::bind(&SimpleOpenNIViewer::image_cb_, this, _1);

//		grabber->registerCallback(f_1);
		grabber->registerCallback(f_2);
		grabber->registerCallback(f_3);

		grabber->start();

		while (!viewer.wasStopped())
		{
		}

		grabber->stop();

//		((pcl::EnsensoGrabber*)grabber)->closeDevice();
	}

	pcl::visualization::CloudViewer viewer;
};

int main()
{
	SimpleOpenNIViewer v;
	v.run();
	return 0;
}

