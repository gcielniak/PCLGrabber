#include <pcl/io/image_grabber.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/lzf_image_io.h>

#include "DeviceInput.h"
#include "PCDGrabberExt.h"
#include "PCDWriterExt.h"

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

using namespace pcl;

class SimpleOpenNIViewer
{
public:

	SimpleOpenNIViewer() : cloud_viewer("PCL Grabber") {}
//	SimpleOpenNIViewer() {}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
	{
		if (!cloud_viewer.wasStopped())
			cloud_viewer.showCloud(cloud);

		FPS_CALC("Cloud:: ");
	}

	void cd_images_cb_(const boost::shared_ptr<io::Image>& color_image, const boost::shared_ptr<io::DepthImage>& depth_image, float flength)
	{
		boost::mutex::scoped_lock lock(cd_mutex);
		depth_image_ = depth_image;
		color_image_ = color_image;

		FPS_CALC("Both:: ");
	}

	void save_images_cb_(const boost::shared_ptr<io::Image>& color_image, const boost::shared_ptr<io::DepthImage>& depth_image, float flength)
	{
		writer.WriteLZFFormat(depth_image, color_image);
	}

	void d_image_cb_(const boost::shared_ptr<io::DepthImage>& depth_image)
	{
		boost::mutex::scoped_lock lock(depth_mutex);
		depth_image_ = depth_image;

		FPS_CALC("Depth:: ");
	}

	void c_image_cb_(const boost::shared_ptr<io::Image>& color_image)
	{
		boost::mutex::scoped_lock lock(color_mutex);
		color_image_ = color_image;

		FPS_CALC("Color:: ");
	}

	void run()
	{
		pcl::DeviceInput device_input;
		device_input.ListAllDevices();
		Grabber* grabber = device_input.GetGrabber();

//		Grabber* grabber = new pcl::ImageGrabber<pcl::PointXYZRGBA>(".\\data\\20150616T143947\\", 30, false, true);
		
//		boost::function<void(const boost::shared_ptr<pcl::io::DepthImage>&)> f_3 =
//			boost::bind(&pcl::PCDWriterExt::WriteLZFDepthImage, &writer, _1);

//		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f_1 =
//			boost::bind(&pcl::PCDWriterExt::WritePCDCloud<pcl::PointXYZ>, &writer, _1);

		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f_2 =
			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
		grabber->registerCallback(f_2);

		/*
		boost::function<void(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float flength)> f_4 =
			boost::bind(&SimpleOpenNIViewer::cd_images_cb_, this, _1, _2, _3);

		boost::function<void(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float flength)> f_8 =
			boost::bind(&SimpleOpenNIViewer::save_images_cb_, this, _1, _2, _3);

		boost::function<void(const boost::shared_ptr<io::Image>&)> f_5 =
			boost::bind(&SimpleOpenNIViewer::c_image_cb_, this, _1);

		boost::function<void(const boost::shared_ptr<io::DepthImage>&)> f_6 =
			boost::bind(&SimpleOpenNIViewer::d_image_cb_, this, _1);
			*/

		//		grabber->registerCallback(f_6);
//		grabber->registerCallback(f_5);

		grabber->start();

		while (!depth_viewer.wasStopped() && !color_viewer.wasStopped() && !cloud_viewer.wasStopped())
		{
			boost::shared_ptr<pcl::io::DepthImage> depth_image;
			boost::shared_ptr<pcl::io::Image> color_image;

			if (cd_mutex.try_lock()){
				depth_image_.swap(depth_image);
				color_image_.swap(color_image);
				cd_mutex.unlock();
			}
			/*
			if (depth_mutex.try_lock()){
				depth_image_.swap(depth_image);
				depth_mutex.unlock();
			}

			if (color_mutex.try_lock()){
				color_image_.swap(color_image);
				color_mutex.unlock();
			}

			*/
			if (depth_image){
				depth_viewer.showShortImage(depth_image->getData(), depth_image->getWidth(), depth_image->getHeight());
				depth_viewer.spinOnce();
			}

			if (color_image){
				color_viewer.showRGBImage((unsigned char*)color_image->getData(), color_image->getWidth(), color_image->getHeight());
				color_viewer.spinOnce();
			}
		}

		grabber->stop();
	}

	pcl::visualization::CloudViewer cloud_viewer;
	pcl::visualization::ImageViewer depth_viewer, color_viewer;
	boost::mutex cd_mutex, depth_mutex, color_mutex;
	boost::shared_ptr<pcl::io::DepthImage> depth_image_;
	boost::shared_ptr<pcl::io::Image> color_image_;
	pcl::PCDWriterExt writer; 
};

int main()
{
	SimpleOpenNIViewer v;
	v.run();
	return 0;
}

