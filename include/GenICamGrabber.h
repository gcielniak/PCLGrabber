#pragma once
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iCVCDriver.h>
#include <iCVCUtilities.h>
#include <iCVGenApi.h>
#include <CVCError.h>

#include "ImageUtils.h"

using namespace std;

namespace PCLGrabber {

	using namespace pcl;

	class GenICamera {
	protected:
		double t_start = 0.0, t_now;
		bool initialised = false;
		IMG hCamera = NULL;
		int type;

	public:
		GenICamera() {
		}

		~GenICamera() {
			ReleaseObject(hCamera);
		}

		cv::Mat image, image_2;

		void Init() {
			if (!initialised) {
				static const size_t DRIVERPATHSIZE = 256;
				// load the first camera
				char driverPath[DRIVERPATHSIZE] = { 0 };
				TranslateFileName("%CVB%\\Drivers\\GenICam.vin", driverPath, DRIVERPATHSIZE);
				if (!LoadImageFile(driverPath, hCamera)) {
					string message = "GenICamera::Init, Error loading \"" + std::string(driverPath) + "\" driver!";
					throw std::exception(message.c_str());
				}
				initialised = true;
			}
			if (G2Grab(hCamera) < 0) {
				throw std::exception("GenICamera::Capture, G2Grab function failure.");
			}
			if (ImageDimension(hCamera) == 3)
				type = CV_8UC3;
			else
				type = CV_8U;
			image = cv::Mat(cv::Size(ImageWidth(hCamera), ImageHeight(hCamera)), type);
			image_2 = cv::Mat(cv::Size(ImageWidth(hCamera), ImageHeight(hCamera)), type);
		}

		int GetNrDevices() {
			cvbval_t value = 0;

			if (CanCameraSelect2(hCamera))
				CS2GetNumPorts(hCamera, value);

			return value;
		}

		void SetCamera(int index) {
			if (CanCameraSelect2(hCamera))
				CS2SetCamPort(hCamera, index, 0, hCamera);
		}

		int GetCameraIndex() {
			cvbval_t port;
			CS2GetCamPort(hCamera, port);
			return port;
		}

		string GetNodeValue(string node_name) {
			NODE node;
			NODEMAP node_map;
			TNodeType node_type;
			char name[256] = { 0 };
			size_t name_size = sizeof(name);

			NMHGetNodeMap(hCamera, node_map);
			NMGetNode(node_map, node_name.c_str(), node);
			NType(node, node_type);
			if (node_type == NT_String)
				NGetAsString(node, name, name_size);
			ReleaseObject(node);
			ReleaseObject(node_map);

			return name;
		}

		string GetDeviceName(int index) {
			stringstream device_name;

			if (index < GetNrDevices()) {
				device_name << GetNodeValue("DeviceFamilyName");
				device_name << " " << GetNodeValue("DeviceModelName");
				device_name << " by " << GetNodeValue("DeviceVendorName");
			}

			return device_name.str();
		}

		long long GetTimeStamp() {
			//calculate timestamp in nanoseconds
			G2GetGrabStatus(hCamera, GRAB_INFO_CMD::GRAB_INFO_TIMESTAMP, t_now);
			if (t_start == 0.0)
				t_start = t_now;
			long long timestamp = (long long)(t_now - t_start);
		}

		cv::Mat& Capture() {
			if (G2Wait(hCamera) >= 0) {
				//assign newly captured buffer to OpenCV Mat object
				void* ppixels = nullptr; intptr_t xInc = 0; intptr_t yInc = 0;
				GetLinearAccess(hCamera, 0, &ppixels, &xInc, &yInc);
				image.data = (uchar*)ppixels;
				if (type == CV_8UC3) {
					cv::cvtColor(image, image_2, CV_BGR2RGB);
					image = image_2;
				}
			}
			return image;
		}
	};

	class GenICamGrabberBase : public Grabber {
	protected:
		boost::thread thread;
		mutable boost::mutex mutex;
		bool quit, running;
		GenICamera camera_1, camera_2;
		cv::Mat image_1, image_2;
		typedef void (Signal_ImageImage)(const boost::shared_ptr<cv::Mat>&, const boost::shared_ptr<cv::Mat>&);

		boost::signals2::signal<Signal_ImageImage>* signal_ImageImage;

		virtual void threadFunction() {
			while (!quit) {
				if (signal_ImageImage->num_slots()) {
					image_1 = camera_1.Capture();
					image_2 = camera_2.Capture();
					signal_ImageImage->operator()(boost::make_shared<cv::Mat>(image_1), boost::make_shared<cv::Mat>(image_2));
				}
			}
		}

	public:
		GenICamGrabberBase() :
			quit(false) {
			camera_1.Init();
			camera_2.Init();
			camera_2.SetCamera(1);
			signal_ImageImage = createSignal<Signal_ImageImage>();
		}

		~GenICamGrabberBase() throw() {
			stop();
			disconnect_all_slots<Signal_ImageImage>();
		}

		void start() {
			running = true;
			thread = boost::thread(&GenICamGrabberBase::threadFunction, this);
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
			return "GenICamera";
		}

		float getFramesPerSecond() const {
			float fps = std::numeric_limits<float>::quiet_NaN();
			return fps;
		}
	};
}
