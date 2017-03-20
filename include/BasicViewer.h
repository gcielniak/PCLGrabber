#pragma once
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/grabber.h>
#include <pcl/common/time.h> //fps calculations
#include "ImageUtils.h"

namespace PCLGrabber {

	using namespace pcl;

	class BasicViewerBase {
	protected:
		bool vis_cloud, vis_images;
		double scale;
		boost::mutex mutex;

	public:
		void keyboardEvent(const pcl::visualization::KeyboardEvent &event) {
			if (event.getKeySym() == "plus" && event.keyDown()) {
				scale *= 1.5;
			}
			else if (event.getKeySym() == "minus" && event.keyDown()) {
				scale /= 1.5;
			}
		}

		virtual void RegisterCallbacks(Grabber* grabber) = 0;
		virtual bool SpinOnce() = 0;

		void VisualiseCloudPoint(bool value) { vis_cloud = value; }
		void VisualiseImages(bool value) { vis_images = value; }
	};

	template <typename PointT, typename ImageT, typename DepthT>
	class BasicViewer : public BasicViewerBase
	{
		visualization::PCLVisualizer *visualizer;

		visualization::ImageViewer *depth_viewer, *color_viewer;
		boost::mutex cloud_mutex, image_mutex;
		boost::shared_ptr<DepthT> depth_image_;
		boost::shared_ptr<ImageT> color_image_;
		boost::shared_ptr<const PointCloud<PointT> > cloud_;

	public:
		BasicViewer() : visualizer(0), depth_viewer(0), color_viewer(0) {
			this->scale = 0.25;
		}

		void cloud_cb_(const boost::shared_ptr<const PointCloud<PointT> >& cloud)
		{
			boost::mutex::scoped_lock lock(cloud_mutex);
			cloud_ = cloud;
			FPS_CALC("CLOUD_VIS");
		}

		void image_callback(const boost::shared_ptr<ImageT>& color_image, const boost::shared_ptr<DepthT>& depth_image)
		{
			boost::mutex::scoped_lock lock(image_mutex);
			depth_image_ = depth_image;
			color_image_ = color_image;
			FPS_CALC("IMG_VIS");
		}

		virtual void RegisterCallbacks(Grabber* grabber)
		{
			if (vis_cloud && grabber->providesCallback<void(const boost::shared_ptr<const PointCloud<PointT> >&)>())
			{
				visualizer = new visualization::PCLVisualizer("PCLGrabber: point cloud");
				visualizer->addCoordinateSystem(1.0);
				visualizer->setCameraPosition(0.0, 0.0, -5.0, 0.0, -1.0, 0.0);

				boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_viscloud =
					boost::bind(&BasicViewer::cloud_cb_, this, _1);
				grabber->registerCallback(f_viscloud);
			}

			if (vis_images)
			{
				if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, const boost::shared_ptr<ImageT>&)>()) {
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, const boost::shared_ptr<ImageT>&)> f_image =
						boost::bind(&BasicViewer::image_callback, this, _3, _2);
					grabber->registerCallback(f_image);
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, float flength)>()) {

					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, float flength)> f_image =
						boost::bind(&BasicViewer::image_callback, this, _1, _2);
					grabber->registerCallback(f_image);
				}
				else {
					return;
				}

				color_viewer = new visualization::ImageViewer();
				depth_viewer = new visualization::ImageViewer();
				color_viewer->setWindowTitle("PCLGrabber: color image");//the constructor does not seem to initialise the name correctly in all circumstances
				depth_viewer->setWindowTitle("PCLGrabber: depth image");
				depth_viewer->setPosition(0, 0);
				color_viewer->setPosition(800, 0);

				depth_viewer->registerKeyboardCallback(boost::bind(&BasicViewerBase::keyboardEvent, this, _1));
				color_viewer->registerKeyboardCallback(boost::bind(&BasicViewerBase::keyboardEvent, this, _1));
			}
		}

		virtual bool SpinOnce()
		{
			boost::shared_ptr<ImageT> color_image;
			boost::shared_ptr<DepthT> depth_image;
			boost::shared_ptr<const PointCloud<PointT> > cloud;

			if (!((visualizer && visualizer->wasStopped()) || (depth_viewer && depth_viewer->wasStopped()) || (color_viewer && color_viewer->wasStopped())))
			{
				if (visualizer)
				{
					if (cloud_mutex.try_lock()) {
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
					if (image_mutex.try_lock()) {
						depth_image_.swap(depth_image);
						color_image_.swap(color_image);
						image_mutex.unlock();
					}

					if (depth_image) {
						if (scale != 1.0) {
							cv::Mat orig(cv::Size(GetWidth(depth_image), GetHeight(depth_image)), CV_16UC1, GetDepthBuffer(depth_image));
							cv::Mat scaled;
							cv::resize(orig, scaled, cv::Size(GetWidth(depth_image)*scale, GetHeight(depth_image)*scale));
							depth_viewer->showShortImage((unsigned short*)scaled.data, scaled.cols, scaled.rows, 
								std::numeric_limits<unsigned short>::min(), std::numeric_limits<unsigned short>::max(), true);
						}
						else
							depth_viewer->showShortImage((unsigned short*)GetDepthBuffer(depth_image), GetWidth(depth_image), GetHeight(depth_image),
								std::numeric_limits<unsigned short>::min(), std::numeric_limits<unsigned short>::max(), true);
					}

					if (color_image) {
						if (scale != 1.0) {
							cv::Mat orig(cv::Size(GetWidth(color_image), GetHeight(color_image)), CV_8UC3, GetRGBBuffer(color_image));
							cv::Mat scaled;
							cv::resize(orig, scaled, cv::Size(GetWidth(color_image)*scale, GetHeight(color_image)*scale));
							color_viewer->showRGBImage(scaled.data, scaled.cols, scaled.rows);
						}
						else
							color_viewer->showRGBImage(GetRGBBuffer(color_image), GetWidth(color_image), GetHeight(color_image));
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
