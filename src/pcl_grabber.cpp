#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/ensenso_grabber.h>
#include "kinect2_grabber.h"
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer() {}

	void image_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		cerr << "!" << endl;
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
	}

	bool cloud_init = false;

	void run()
	{
		//		pcl::Grabber* grabber = new pcl::OpenNIGrabber();
		pcl::Grabber* grabber = new pcl::io::OpenNI2Grabber();
//		pcl::Grabber* grabber = new pcl::EnsensoGrabber();
//  		((pcl::EnsensoGrabber*)grabber)->openTcpPort();
//		((pcl::EnsensoGrabber*)grabber)->openDevice();
//		pcl::Grabber* grabber = new pcl::Kinect2Grabber();

		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind(&SimpleOpenNIViewer::image_cb_, this, _1);


		grabber->registerCallback(f);

		grabber->start();

		while (!image_viewer.wasStopped())
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;

			image_viewer.spinOnce();

			// See if we can get a cloud
			if (cloud_mutex_.try_lock())
			{
				cloud_.swap(cloud);
				cloud_mutex_.unlock();
			}

			if (cloud)
			{
				image_viewer.addRGBImage<pcl::PointXYZRGBA>(cloud, "OpenNICloud");
				cerr << "h" << endl;
			}

			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		}

		grabber->stop();

//		((pcl::EnsensoGrabber*)grabber)->closeDevice();
	}

//	pcl::visualization::CloudViewer viewer;
	pcl::visualization::ImageViewer image_viewer;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;
	boost::mutex cloud_mutex_;
};

int main()
{
	SimpleOpenNIViewer v;
	v.run();
	return 0;
}

