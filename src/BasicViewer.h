#pragma once
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image.h>
#include <pcl/io/openni_camera/openni_image.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/grabber.h>
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

namespace pcl
{
	template <typename PointT>
	class BasicViewer
	{
		bool vis_cloud, vis_images;
		visualization::PCLVisualizer *visualizer;

		visualization::ImageViewer *depth_viewer, *color_viewer;
		boost::mutex cloud_mutex, image_mutex, oni_image_mutex;
		boost::shared_ptr<io::DepthImage> depth_image_;
		boost::shared_ptr<io::Image> color_image_;
		boost::shared_ptr<openni_wrapper::DepthImage> oni_depth_image_;
		boost::shared_ptr<openni_wrapper::Image> oni_color_image_;
		boost::shared_ptr<const PointCloud<PointT> > cloud_;

	public:
		BasicViewer() : vis_cloud(false), vis_images(false),
			visualizer(0), depth_viewer(0), color_viewer(0)
		{
		}

		void VisualiseCloudPoint(bool value) { vis_cloud = value; }
		void VisualiseImages(bool value) { vis_images = value; }

		void cloud_cb_(const boost::shared_ptr<const PointCloud<PointT> >& cloud)
		{
			boost::mutex::scoped_lock lock(cloud_mutex);
			cloud_ = cloud;
			FPS_CALC("CLOUD_VIS");
		}

		void image_cb_(const boost::shared_ptr<io::Image>& color_image, const boost::shared_ptr<io::DepthImage>& depth_image)
		{
			boost::mutex::scoped_lock lock(image_mutex);
			depth_image_ = depth_image;
			color_image_ = color_image;
			FPS_CALC("IMG_VIS");
		}

		void image_cboni_(const boost::shared_ptr<openni_wrapper::Image>& color_image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image)
		{
			boost::mutex::scoped_lock lock(image_mutex);
			oni_depth_image_ = depth_image;
			oni_color_image_ = color_image;
		}

		void RegisterCallbacks(Grabber* grabber)
		{
			if (vis_cloud && grabber->providesCallback<void(const boost::shared_ptr<const PointCloud<PointT> >&)>())
			{
				visualizer = new visualization::PCLVisualizer("PCLGrabber: point cloud");
				visualizer->setCameraPosition(0.0, -0.0, -4.0, 0.0, -1.0, 0.0);

				boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_viscloud =
					boost::bind(&BasicViewer::cloud_cb_, this, _1);
				grabber->registerCallback(f_viscloud);
			}

			if (vis_images)
			{
				if (grabber->providesCallback<void(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float flength)>())
				{
					boost::function<void(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float flength)> f_image =
						boost::bind(&BasicViewer::image_cb_, this, _1, _2);
					grabber->registerCallback(f_image);

					color_viewer = new visualization::ImageViewer("PCLGrabber: color image");
					depth_viewer = new visualization::ImageViewer("PCLGrabber: depth image");
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float flength)>())
				{
					boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float flength)> f_image =
						boost::bind(&BasicViewer::image_cboni_, this, _1, _2);
					grabber->registerCallback(f_image);

					color_viewer = new visualization::ImageViewer("PCLGrabber: color image");
					depth_viewer = new visualization::ImageViewer("PCLGrabber: depth image");
				}
			}
		}

		bool SpinOnce()
		{
			if (!((visualizer && visualizer->wasStopped()) || (depth_viewer && depth_viewer->wasStopped()) || (color_viewer && color_viewer->wasStopped())))
			{
				boost::shared_ptr<io::Image> color_image;
				boost::shared_ptr<io::DepthImage> depth_image;
				boost::shared_ptr<openni_wrapper::Image> oni_color_image;
				boost::shared_ptr<openni_wrapper::DepthImage> oni_depth_image;
				boost::shared_ptr<const PointCloud<PointT> > cloud;

				if (visualizer)
				{
					if (cloud_mutex.try_lock()){
						cloud_.swap(cloud);
						cloud_mutex.unlock();
					}

					if (cloud)
					{
						visualization::PointCloudColorHandlerRGBField<PointT> color_h(cloud);
						if (!visualizer->updatePointCloud<PointT>(cloud, color_h))
							visualizer->addPointCloud<PointT>(cloud, color_h);
					}

					visualizer->spinOnce();
				}

				if (depth_viewer && color_viewer)
				{
					if (image_mutex.try_lock()){
						depth_image_.swap(depth_image);
						color_image_.swap(color_image);
						image_mutex.unlock();
					}

					if (oni_image_mutex.try_lock()){
						oni_depth_image_.swap(oni_depth_image);
						oni_color_image_.swap(oni_color_image);
						oni_image_mutex.unlock();
					}

					if (depth_image)
						depth_viewer->showShortImage(depth_image->getData(), depth_image->getWidth(), depth_image->getHeight());

					if (color_image)
					{
						if (color_image->getEncoding() != io::Image::Encoding::RGB)
						{
							vector<unsigned char> rgb_buffer(color_image->getWidth()*color_image->getHeight() * 3);
							color_image->fillRGB(color_image->getWidth(), color_image->getHeight(), &rgb_buffer[0]);
							color_viewer->showRGBImage(&rgb_buffer[0], color_image->getWidth(), color_image->getHeight());
						}
						else
							color_viewer->showRGBImage((unsigned char*)color_image->getData(), color_image->getWidth(), color_image->getHeight());
					}

					if (oni_depth_image)
						depth_viewer->showShortImage(oni_depth_image->getDepthMetaData().Data(), oni_depth_image->getWidth(), oni_depth_image->getHeight());

					if (oni_color_image)
					{
						if (oni_color_image->getEncoding() != openni_wrapper::Image::Encoding::RGB)
						{
							vector<unsigned char> rgb_buffer(oni_color_image->getWidth()*oni_color_image->getHeight() * 3);
							oni_color_image->fillRGB(oni_color_image->getWidth(), oni_color_image->getHeight(), &rgb_buffer[0]);
							color_viewer->showRGBImage(&rgb_buffer[0], oni_color_image->getWidth(), oni_color_image->getHeight());
						}
						else
							color_viewer->showRGBImage((unsigned char*)oni_color_image->getMetaData().Data(), oni_color_image->getWidth(), oni_color_image->getHeight());
					}

					depth_viewer->spinOnce();
					color_viewer->spinOnce();
				}

				return true;
			}
			else
				return false;
		}
	};
}
