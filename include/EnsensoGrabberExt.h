#pragma once
#include <pcl/io/ensenso_grabber.h>

namespace pcl
{
	class EnsensoGrabberExt : public EnsensoGrabber
	{
	public:
		typedef void (sig_cb_ensenso_const_point_cloud)(const PointCloud<PointXYZ>::ConstPtr &);
		typedef void (sig_cb_ensenso_point_cloud_color)(const PointCloud<PointXYZRGBA>::ConstPtr &);

		EnsensoGrabberExt(int device)
		{
			openTcpPort();
			openDevice(device);

			const_point_cloud_signal_ = createSignal<sig_cb_ensenso_const_point_cloud>();
			point_cloud_color_signal_ = createSignal<sig_cb_ensenso_point_cloud_color>();

			boost::function<void(const PointCloud<PointXYZ>::Ptr &)> f_cloud =
				boost::bind(&EnsensoGrabberExt::GenerateSignals, this, _1);
			this->registerCallback(f_cloud);
		}

		~EnsensoGrabberExt()
		{
			disconnect_all_slots<sig_cb_ensenso_const_point_cloud>();
			disconnect_all_slots<sig_cb_ensenso_point_cloud_color>();

			closeDevice();
		}

		void GenerateSignals(const PointCloud<PointXYZ>::Ptr & point_cloud)
		{
			if (const_point_cloud_signal_->num_slots() > 0)
				const_point_cloud_signal_->operator()(PointCloud<PointXYZ>::ConstPtr(point_cloud));
			if (point_cloud_color_signal_->num_slots() > 0)
				point_cloud_color_signal_->operator()(ToXYZRGBA(point_cloud));
		}

		PointCloud<PointXYZRGBA>::ConstPtr ToXYZRGBA(const PointCloud<PointXYZ>::Ptr & point_cloud)
		{
			PointCloud<PointXYZRGBA>::Ptr color_point_cloud(new PointCloud<PointXYZRGBA>(point_cloud->width, point_cloud->height));

			const PointXYZ* pt = &point_cloud->points[0];
			PointXYZRGBA* color_pt = &color_point_cloud->points[0];

			for (int i = 0; i < point_cloud->points.size(); i++, pt++, color_pt++)
			{
				color_pt->x = pt->x;
				color_pt->y = pt->y;
				color_pt->z = pt->z;
			}

			return color_point_cloud;
		}

	protected:
		boost::signals2::signal<sig_cb_ensenso_const_point_cloud>* const_point_cloud_signal_;
		boost::signals2::signal<sig_cb_ensenso_point_cloud_color>* point_cloud_color_signal_;

	};
}