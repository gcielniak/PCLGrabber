#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h> //fps calculations
#include "DeviceInput.h"
#include "FileInput.h"
#include "FileOutput.h"

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

class SimpleViewer
{
	typedef PointXYZRGBA CloudType;
	int platform, device, file_format;
	bool vis_cloud, vis_images;
	Grabber* grabber;
	DeviceInput device_input;
	FileOutput file_output;
	visualization::CloudViewer *cloud_viewer;
	visualization::ImageViewer *depth_viewer, *color_viewer;
	boost::mutex cloud_mutex, image_mutex;
	boost::shared_ptr<io::DepthImage> depth_image_;
	boost::shared_ptr<io::Image> color_image_;
	PointCloud<CloudType>::ConstPtr cloud_;

public:

	SimpleViewer() : platform(0), device(0), grabber(0), file_format(-1),
		vis_cloud(false), vis_images(false),
		cloud_viewer(0), depth_viewer(0), color_viewer(0) {}

	void Platform(int _platform) { platform = _platform; }
	void Device(int _device) { device = _device; }
	void VisualiseCloudPoint(bool _vis_cloud) { vis_cloud = _vis_cloud; }
	void VisualiseImages(bool _vis_images) { vis_images = _vis_images; }
	void WriteFiles(int _file_format) { file_format = _file_format; }

	void cloud_cb_(const PointCloud<CloudType>::ConstPtr& cloud)
	{
		boost::mutex::scoped_lock lock(cloud_mutex);
		cloud_ = cloud;
	}

	void image_cb_(const boost::shared_ptr<io::Image>& color_image, const boost::shared_ptr<io::DepthImage>& depth_image)
	{
		boost::mutex::scoped_lock lock(image_mutex);
		depth_image_ = depth_image;
		color_image_ = color_image;
	}

	void run()
	{
		try
		{
			grabber = device_input.GetGrabber(platform, device);
		}
		catch (pcl::PCLException exc)
		{
			cerr << exc.detailedMessage() << endl;
			return;
		}

		if (vis_cloud)
		{
			cloud_viewer = new visualization::CloudViewer("PCLGrabber: point cloud");
			boost::function<void(const PointCloud<CloudType>::ConstPtr&)> f_viscloud =
				boost::bind(&SimpleViewer::cloud_cb_, this, _1);
			grabber->registerCallback(f_viscloud);
		}

		if (vis_images)
		{
			color_viewer = new visualization::ImageViewer("PCLGrabber: color image");
			depth_viewer = new visualization::ImageViewer("PCLGrabber: depth image");
			boost::function<void(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float flength)> f_image =
				boost::bind(&SimpleViewer::image_cb_, this, _1, _2);
			grabber->registerCallback(f_image);
		}

		if (file_format != -1)
		{
			switch (file_format)
			{
			case 0:
			{
				boost::function<void(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float flength)> f_write =
					boost::bind(&FileOutput::WriteImageLZF, file_output, _1, _2);
				grabber->registerCallback(f_write);
			}
				break;
			case 1:
			{
				boost::function<void(const pcl::PointCloud<CloudType>::ConstPtr&)> f_write =
					boost::bind(&FileOutput::WriteCloudPCD<CloudType>, &file_output, _1);
				grabber->registerCallback(f_write);
			}
				break;
			default:
				break;
			}

		}

		grabber->start();

		while (!((cloud_viewer && cloud_viewer->wasStopped()) || (depth_viewer && depth_viewer->wasStopped()) || (color_viewer && color_viewer->wasStopped())))
		{
			boost::shared_ptr<io::Image> color_image;
			boost::shared_ptr<io::DepthImage> depth_image;
			PointCloud<CloudType>::ConstPtr cloud;

			if (cloud_mutex.try_lock()){
				cloud_.swap(cloud);
				cloud_mutex.unlock();
			}

			if (image_mutex.try_lock()){
				depth_image_.swap(depth_image);
				color_image_.swap(color_image);
				image_mutex.unlock();
			}

			if (cloud_viewer && cloud) {
				cloud_viewer->showCloud(cloud);
			}
			
			if (depth_viewer && depth_image) {
				depth_viewer->showShortImage(depth_image->getData(), depth_image->getWidth(), depth_image->getHeight());
				depth_viewer->spinOnce();
			}
			
			if (color_viewer && color_image){
				color_viewer->showRGBImage((unsigned char*)color_image->getData(), color_image->getWidth(), color_image->getHeight());
				color_viewer->spinOnce();
			}
		}

		grabber->stop();
	}
};

void print_help()
{
	cerr << "PCLGrabber usage:" << endl;

	cerr << "  -l : list all available input platforms and devices" << endl;
	cerr << "  -p : specify the input platform" << endl;
	cerr << "  -d : specify the input device" << endl;
	cerr << " -vc : visualise the cloud point" << endl;
	cerr << " -vi : visualise images" << endl;
	cerr << "  -w : write files using a specific format:" << endl;
	cerr << "       0 - pclzf" << endl;
	cerr << "       1 - pcd" << endl;
	cerr << "  -h : print this message" << endl;
}

int main(int argc, char **argv)
{
	if (argc == 1)
	{
		print_help();
		exit(0);
	}

	int platform = 0;
	int device = 0;
	int write_format = -1;
	bool visualise_cloud = false;
	bool visualise_images = false;

	SimpleViewer viewer;

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-l") == 0)
		{
			DeviceInput device_input;
			device_input.ListAllDevices();
			return 0;
		}
		else if ((strcmp(argv[i], "-p") == 0) && (i < (argc - 1))) { viewer.Platform(atoi(argv[++i])); }
		else if ((strcmp(argv[i], "-d") == 0) && (i < (argc - 1))) { viewer.Device(atoi(argv[++i])); }
		else if ((strcmp(argv[i], "-w") == 0) && (i < (argc - 1))) { viewer.WriteFiles(atoi(argv[++i])); }
		else if (strcmp(argv[i], "-vc") == 0) { viewer.VisualiseCloudPoint(true); }
		else if (strcmp(argv[i], "-vi") == 0) { viewer.VisualiseImages(true); }
		else if (strcmp(argv[i], "-h") == 0) { print_help(); }
	}

	viewer.run();

	return 0;
}

