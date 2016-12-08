#pragma once
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#include "ImageUtils.h"

namespace PCLGrabber {

	using namespace pcl;

	/** Support for ZED stereo camera. The base class that does not require template specialisation.
	*/
	class ZEDGrabberBase : public Grabber {
	protected:
		sl::zed::Camera* zed;
		boost::thread thread;
		mutable boost::mutex mutex;
		bool quit, running;

		virtual void threadFunction() = 0;

	public:
		ZEDGrabberBase(const string& device = "") :
			quit(false) {
			if (device == "")
				zed = new sl::zed::Camera(sl::zed::HD720);//use device (only one avaialble in Windows)
			else
				zed = new sl::zed::Camera(device);//use file

			sl::zed::InitParams params;
			params.mode = sl::zed::PERFORMANCE;
			params.unit = sl::zed::METER; // Scale to fit OpenGL world
			params.coordinate = sl::zed::IMAGE; // OpenGL compatible
			params.verbose = false;
			params.disableSelfCalib = true;
			sl::zed::ERRCODE err = zed->init(params);
		}

		~ZEDGrabberBase() throw() {
			stop();

			if (zed)
				delete zed;
		}

		void start() {
			running = true;
			thread = boost::thread(&ZEDGrabberBase::threadFunction, this);
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
			string name = "ZED Camera";

			boost::unique_lock<boost::mutex> lock(mutex);
			if (zed) {
				name += " ";
				name += zed->getZEDSerial();
			}
			lock.unlock();
			return name;
		}

		float getFramesPerSecond() const {
			float fps = std::numeric_limits<float>::quiet_NaN();

			boost::unique_lock<boost::mutex> lock(mutex);
			if (zed)
				fps = zed->getCurrentFPS();
			lock.unlock();

			return fps;
		}

		static int NumberDevices() {
			return sl::zed::Camera::isZEDconnected();
		}

	};

	/** Support for ZED stereo camera.
	*/
	template <typename PointT, typename ImageT, typename DepthT>
	class ZEDGrabber : public ZEDGrabberBase {
	protected:
		typename PointCloud<PointT>::Ptr cloud;

		typedef void (Signal_PointT)(const boost::shared_ptr<const PointCloud<PointT>>&);
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, float focal_length);

		boost::signals2::signal<Signal_PointT>* signal_PointT;
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;

		void threadFunction() {

			float *depth_buffer;
			static long long pcloud_counter = 0;

			while (!quit) {

				if (!zed->grab(sl::zed::STANDARD)) {
					cloud->header.stamp = zed->getCameraTimestamp();
					depth_buffer = (float*)zed->retrieveMeasure(sl::zed::MEASURE::XYZRGBA).data; // Get the pointer			
					cloud->header.seq = pcloud_counter++;

					for (auto &it : cloud->points) {
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
						signal_PointT->operator()(cloud);

					if (signal_ImageDepth->num_slots())
						signal_ImageDepth->operator()(ToImage<PointT, ImageT>(cloud), ToDepthImage<PointT, DepthT>(cloud), 0.0);
				}
			}
		}

	public:
		ZEDGrabber(const string& device = "") :
			ZEDGrabberBase(device) {

			cloud = boost::make_shared<PointCloud<PointT>>();
			cloud->width = static_cast<uint32_t>(zed->getImageSize().width);
			cloud->height = static_cast<uint32_t>(zed->getImageSize().height);
			cloud->is_dense = true;
			cloud->points.resize(zed->getImageSize().width*zed->getImageSize().height);

			signal_PointT = createSignal<Signal_PointT>();
			signal_ImageDepth = createSignal<Signal_ImageDepth>();
		}

		~ZEDGrabber() throw() {
			disconnect_all_slots<Signal_PointT>();
			disconnect_all_slots<Signal_ImageDepth>();
		}

	};
}
