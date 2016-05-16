#pragma once
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/grabber.h>
#include <pcl/common/time.h> //fps calculations
#include "ImageUtils.h"

namespace pcl
{
	template <typename PointT, typename ImageT, typename DepthImageT>
	class BasicViewer
	{
		bool vis_cloud, vis_images;
		visualization::PCLVisualizer *visualizer;

		visualization::ImageViewer *depth_viewer, *color_viewer;
		boost::mutex cloud_mutex, image_mutex;
		boost::shared_ptr<DepthImageT> depth_image_;
		boost::shared_ptr<ImageT> color_image_;
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

		void image_callback(const boost::shared_ptr<ImageT>& color_image, const boost::shared_ptr<DepthImageT>& depth_image)
		{
			boost::mutex::scoped_lock lock(image_mutex);
			depth_image_ = depth_image;
			color_image_ = color_image;
			FPS_CALC("IMG_VIS");
		}

		void RegisterCallbacks(Grabber* grabber)
		{
			if (vis_cloud && grabber->providesCallback<void(const boost::shared_ptr<const PointCloud<PointT> >&)>())
			{
				visualizer = new visualization::PCLVisualizer("PCLGrabber: point cloud");
				visualizer->addCoordinateSystem(1.0);
				visualizer->setCameraPosition(0.0, -20.0, 0.0, 0.0, 0.0, 1.0);
//for asus		visualizer->setCameraPosition(0.0, 0.0, -5.0, 0.0, -1.0, 0.0);

				boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_viscloud =
					boost::bind(&BasicViewer::cloud_cb_, this, _1);
				grabber->registerCallback(f_viscloud);
			}

			if (vis_images)
			{
				if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)> f_image =
						boost::bind(&BasicViewer::image_callback, this, _3, _2);
					grabber->registerCallback(f_image);

					color_viewer = new visualization::ImageViewer("PCLGrabber: color image");
					depth_viewer = new visualization::ImageViewer("PCLGrabber: depth image");
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float flength)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float flength)> f_image =
						boost::bind(&BasicViewer::image_callback, this, _1, _2);
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
				boost::shared_ptr<ImageT> color_image;
				boost::shared_ptr<DepthImageT> depth_image;
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

						/*
						Eigen::Quaternionf orient = cloud->sensor_orientation_;
						Eigen::Vector4f origin = cloud->sensor_origin_;

						cerr << "Orient x: " << orient.x() << ", y: " << orient.y() << ", z: " << orient.z() << ", w: " << orient.w() << endl;
						cerr << "Origin x: " << origin.x() << ", y: " << origin.y() << ", z: " << origin.z() << endl;
						*/
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

					if (depth_image)
						depth_viewer->showShortImage(GetDepthBuffer(depth_image), GetWidth(depth_image), GetHeight(depth_image));

					if (color_image)
						color_viewer->showRGBImage(GetRGBBuffer(color_image), GetWidth(color_image), GetHeight(color_image));

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
