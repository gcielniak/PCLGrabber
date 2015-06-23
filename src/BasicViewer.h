#pragma once
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
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
		visualization::CloudViewer *cloud_viewer;
		visualization::ImageViewer *depth_viewer, *color_viewer;
		boost::mutex cloud_mutex, image_mutex, oni_image_mutex;
		boost::shared_ptr<io::DepthImage> depth_image_;
		boost::shared_ptr<io::Image> color_image_;
		boost::shared_ptr<openni_wrapper::DepthImage> oni_depth_image_;
		boost::shared_ptr<openni_wrapper::Image> oni_color_image_;
		typename PointCloud<PointT>::ConstPtr cloud_;

	public:
		BasicViewer() : vis_cloud(false), vis_images(false),
			cloud_viewer(0), depth_viewer(0), color_viewer(0)
		{
		}

		void VisualiseCloudPoint(bool value) { vis_cloud = value; }
		void VisualiseImages(bool value) { vis_images = value; }

		void cloud_cb_(const typename PointCloud<PointT>::Ptr& cloud)
		{
			boost::mutex::scoped_lock lock(cloud_mutex);
			cloud_ = boost::static_pointer_cast<>;
		}

		void cloud_cb_const_(const typename PointCloud<PointT>::ConstPtr& cloud)
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

		void image_cboni_(const boost::shared_ptr<openni_wrapper::Image>& color_image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image)
		{
			boost::mutex::scoped_lock lock(image_mutex);
			oni_depth_image_ = depth_image;
			oni_color_image_ = color_image;
		}

		void RegisterCallbacks(Grabber* grabber)
		{
			if (vis_cloud)
			{
				cloud_viewer = new visualization::CloudViewer("PCLGrabber: point cloud");

				if (grabber->providesCallback<void(const PointCloud<PointT>::ConstPtr&)>())
				{
					boost::function<void(const PointCloud<PointT>::ConstPtr&)> f_viscloud =
						boost::bind(&BasicViewer::cloud_cb_, this, _1);
					grabber->registerCallback(f_viscloud);
				}
				else if (grabber->providesCallback<void(const PointCloud<PointT>::Ptr&)>())
				{
					boost::function<void(const PointCloud<PointT>::Ptr&)> f_viscloud =
						boost::bind(&BasicViewer::cloud_cb_, this, _1);
					grabber->registerCallback(f_viscloud);
				}
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
				else
				{
					cerr << "Warning: no suitable visualisation callback found!" << endl;
				}
			}
		}

		bool SpinOnce()
		{
			if (!((cloud_viewer && cloud_viewer->wasStopped()) || (depth_viewer && depth_viewer->wasStopped()) || (color_viewer && color_viewer->wasStopped())))
			{
				boost::shared_ptr<io::Image> color_image;
				boost::shared_ptr<io::DepthImage> depth_image;
				boost::shared_ptr<openni_wrapper::Image> oni_color_image;
				boost::shared_ptr<openni_wrapper::DepthImage> oni_depth_image;
				PointCloud<PointT>::ConstPtr cloud;

				if (cloud_viewer)
				{
					if (cloud_mutex.try_lock()){
						cloud_.swap(cloud);
						cloud_mutex.unlock();
					}

					if (cloud)
						cloud_viewer->showCloud(cloud);
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

					if (depth_image) {
						depth_viewer->showShortImage(depth_image->getData(), depth_image->getWidth(), depth_image->getHeight());
						depth_viewer->spinOnce();
					}

					if (color_image){
						color_viewer->showRGBImage((unsigned char*)color_image->getData(), color_image->getWidth(), color_image->getHeight());
						color_viewer->spinOnce();
					}

					if (oni_depth_image) {
						depth_viewer->showShortImage(oni_depth_image->getDepthMetaData().Data(), oni_depth_image->getWidth(), oni_depth_image->getHeight());
						depth_viewer->spinOnce();
					}

					if (oni_color_image){
						color_viewer->showRGBImage((unsigned char*)oni_color_image->getMetaData().Data(), oni_color_image->getWidth(), oni_color_image->getHeight());
						color_viewer->spinOnce();
					}

				}

				return true;
			}
			else
				return false;
		}
	};
}
