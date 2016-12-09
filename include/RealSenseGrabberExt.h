#pragma once
#include "ImageUtils.h"
#include <pcl/io/real_sense_grabber.h>

namespace PCLGrabber {
	
	/** Extra support for RealSenseGrabber library. Adds image signals which are triggered on a point cloud signal.
	*/
	template <typename PointT, typename ImageT, typename DepthT>
	class RealSenseGrabberExt : public pcl::RealSenseGrabber {
	protected:
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, float focal_length);
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;

	public:
		RealSenseGrabberExt(const string& device) :
			RealSenseGrabber(device) {
			signal_ImageDepth = createSignal<Signal_ImageDepth>();
			
			//bind image signals to point cloud signal
			boost::function<void(const PointCloud<PointT>::ConstPtr&)> f_cloud =
				boost::bind(&RealSenseGrabberExt::SignalImages, this, _1);
			this->registerCallback(f_cloud);
		}

		//trigger image signals
		void SignalImages(const typename PointCloud<PointT>::ConstPtr& cloud)
		{
			if (signal_ImageDepth->num_slots())
				signal_ImageDepth->operator()(ToImage<PointT, ImageT>(cloud), ToDepthImage<PointT, DepthT>(cloud), 0.0);
		}

		~RealSenseGrabberExt() throw() {
			disconnect_all_slots<Signal_ImageDepth>();
		}
	};
}
