#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/ensenso_grabber.h>
#include "kinect2_grabber.h"
#include <pcl/visualization/cloud_viewer.h>

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer") {}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
	}

	void run()
	{
		//		pcl::Grabber* grabber = new pcl::OpenNIGrabber();
//		pcl::Grabber* grabber = new pcl::io::OpenNI2Grabber();
//		pcl::Grabber* grabber = new pcl::EnsensoGrabber();
//  		((pcl::EnsensoGrabber*)grabber)->openTcpPort();
//		((pcl::EnsensoGrabber*)grabber)->openDevice();
		pcl::Grabber* grabber = new pcl::Kinect2Grabber();

		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		grabber->registerCallback(f);

		grabber->start();

		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
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

