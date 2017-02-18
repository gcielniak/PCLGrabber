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

		cv::Mat image, image_mono16;

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
		}

		void Start() {
			if (G2Grab(hCamera) < 0) {
				throw std::exception("GenICamera::Capture, G2Grab function failure.");
			}
			if (ImageDimension(hCamera) == 3)
				type = CV_8UC3;
			else
				type = CV_8U;
			image = cv::Mat(cv::Size(ImageWidth(hCamera), ImageHeight(hCamera)), type);
			image_mono16 = cv::Mat(cv::Size(ImageWidth(hCamera), ImageHeight(hCamera)), CV_16UC1);
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

			int current_index = GetCameraIndex();

			if (index < GetNrDevices()) {
				SetCamera(index);
				device_name << GetNodeValue("DeviceFamilyName");
				device_name << " " << GetNodeValue("DeviceModelName");
				device_name << " by " << GetNodeValue("DeviceVendorName");
				SetCamera(current_index);
			}

			return device_name.str();
		}

		long long GetTimeStamp() {
			//calculate timestamp in nanoseconds
			G2GetGrabStatus(hCamera, GRAB_INFO_CMD::GRAB_INFO_TIMESTAMP, t_now);
			if (t_start == 0.0)
				t_start = t_now;
			long long timestamp = (long long)(t_now - t_start);
			return timestamp;
		}

		cv::Mat& Capture() {
			if (G2Wait(hCamera) >= 0) {
				//assign newly captured buffer to OpenCV Mat object
				void* ppixels = nullptr; intptr_t xInc = 0; intptr_t yInc = 0;
				GetLinearAccess(hCamera, 0, &ppixels, &xInc, &yInc);
				image.data = (uchar*)ppixels;
				if (type == CV_8UC3) {
					cv::cvtColor(image, image, CV_BGR2RGB);
				}
				else {
					image.convertTo(image_mono16, CV_16UC1);
					image_mono16 *= 256;
					return image_mono16;
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
		CvMatExt image_1, image_2;
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<CvMatExt>&, const boost::shared_ptr<CvMatExt>&, float focal_length);

		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;

		virtual void threadFunction() {
			camera_1.Start();
			camera_2.Start();
			while (!quit) {
				if (signal_ImageDepth->num_slots()) {
					image_1.image = camera_1.Capture();
					image_2.image = camera_2.Capture();
					image_1.timestamp = camera_1.GetTimeStamp();
					image_2.timestamp = camera_2.GetTimeStamp();
					signal_ImageDepth->operator()(boost::make_shared<CvMatExt>(image_1), boost::make_shared<CvMatExt>(image_2), 0.0);
				}
			}
		}

	public:
		GenICamGrabberBase() :
			quit(false) {
			camera_1.Init();
			camera_2.Init();
			camera_2.SetCamera(1);
			signal_ImageDepth = createSignal<Signal_ImageDepth>();
		}

		~GenICamGrabberBase() throw() {
			stop();
			disconnect_all_slots<Signal_ImageDepth>();
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
