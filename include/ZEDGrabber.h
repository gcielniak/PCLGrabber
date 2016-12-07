#pragma once
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#include "ImageUtils.h"

namespace PCLGrabber {

	using namespace pcl;

	/** Support for ZED stereo camera.
	*/
	template <typename PointT, typename ImageT, typename DepthT>
	class ZEDGrabber : public Grabber {
	protected:
		boost::thread thread;
		mutable boost::mutex mutex;

		bool quit, running;
		typename PointCloud<PointT>::Ptr point_cloud;

		sl::zed::Camera* zed;

		typedef void (Signal_PointT)(const boost::shared_ptr<const PointCloud<PointT>>&);
		boost::signals2::signal<Signal_PointT>* signal_PointT;

		void threadFunction() {

			float *depth_buffer;
			static long long pcloud_counter = 0;

			while (!quit) {

				if (!zed->grab(sl::zed::STANDARD)) {
					depth_buffer = (float*)zed->retrieveMeasure(sl::zed::MEASURE::XYZRGBA).data; // Get the pointer			
//					point_cloud->header.stamp = zed->getCameraTimestamp();
					point_cloud->header.seq = pcloud_counter++;

					for (auto &it : point_cloud->points) {
						float X = *depth_buffer++;
						if (!isValidMeasure(X)) { // Checking if it's a valid point
							it.x = it.y = it.z = it.rgb = 0;
							depth_buffer += 3;
						}
						else {
							it.x = X;
							it.y = *depth_buffer++;
							it.z = *depth_buffer++;
							unsigned char* color = (unsigned char*)depth_buffer++;
							it.r = color[0];
							it.g = color[1];
							it.b = color[2];
						}
						if (typeid(PointT) == typeid(PointXYZRGBA))
							it.a = 255;
					}

					if (signal_PointT->num_slots())
						signal_PointT->operator()(point_cloud);
				}
			}
		}

	public:
		ZEDGrabber(const string& device = "") :
			quit(false) {
			zed = new sl::zed::Camera(sl::zed::HD720);

			sl::zed::InitParams params;
			params.mode = sl::zed::PERFORMANCE;
			params.unit = sl::zed::METER; // Scale to fit OpenGL world
			params.coordinate = sl::zed::IMAGE; // OpenGL compatible
			params.verbose = false;
			sl::zed::ERRCODE err = zed->init(params);

			point_cloud = boost::make_shared<PointCloud<PointT>>();
			point_cloud->width = static_cast<uint32_t>(zed->getImageSize().width);
			point_cloud->height = static_cast<uint32_t>(zed->getImageSize().height);
			point_cloud->is_dense = true;
			point_cloud->points.resize(zed->getImageSize().width*zed->getImageSize().height);

			signal_PointT = createSignal<Signal_PointT>();
		}

		~ZEDGrabber() throw() {
			stop();

			disconnect_all_slots<Signal_PointT>();

			if (zed)
				delete zed;
		}

		void start() {
			running = true;
			thread = boost::thread(&ZEDGrabber::threadFunction, this);
		}

		void stop() {
			boost::unique_lock<boost::mutex> lock(mutex);
			quit = true;
			running = false;
			lock.unlock();
		}

		bool isRunning() const {
			boost::unique_lock<boost::mutex> lock(mutex);
			return running;
			lock.unlock();
		}

		std::string getName() const {
			return std::string("ZEDGrabber");
		}

		float getFramesPerSecond() const {
			return 30.0f;
		}
	};
}
