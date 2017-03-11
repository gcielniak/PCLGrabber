#pragma once
//#include <pcl/io/grabber.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

#include <iCVCDriver.h>
#include <iCVCUtilities.h>
#include <iCVGenApi.h>
#include <CVCError.h>

//#include "ImageUtils.h"

using namespace std;

namespace PCLGrabber {

	using namespace pcl;

	class GenICamera {
	protected:
		double t_start = 0.0, t_now;
		bool initialised = false;
		IMG hCamera = NULL;
		NODEMAP node_map = NULL;
		int type;

	public:
		GenICamera() {
		}

		~GenICamera() {
			ReleaseObject(node_map);
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
				NMHGetNodeMap(hCamera, node_map);
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
			if (CanCameraSelect2(hCamera)) {
				CS2SetCamPort(hCamera, index, 0, hCamera);
				NMHGetNodeMap(hCamera, node_map);
			}
		}

		int GetCameraIndex() {
			cvbval_t port;
			CS2GetCamPort(hCamera, port);
			return port;
		}

		string GetNodeValue(string node_name) {
			NODE node;
			TNodeType node_type;
			char name[256] = { 0 };
			size_t name_size = sizeof(name);

			NMGetNode(node_map, node_name.c_str(), node);
			NType(node, node_type);
			NGetAsString(node, name, name_size);
			ReleaseObject(node);

			return name;
		}

		void SetNodeValue(string node_name, string value) {
			NODE node;

			NMGetNode(node_map, node_name.c_str(), node);
			NSetAsString(node, value.c_str());
			ReleaseObject(node);
		}

		void SetNodeValue(string node_name, double value) {
			NODE node;

			NMGetNode(node_map, node_name.c_str(), node);
			NSetAsFloat(node, value);
			ReleaseObject(node);
		}

		void SetNodeValue(string node_name, int value) {
			NODE node;

			NMGetNode(node_map, node_name.c_str(), node);
			NSetAsInteger(node, value);
			ReleaseObject(node);
		}

		void SetNodeValue(string node_name, bool value) {
			NODE node;

			NMGetNode(node_map, node_name.c_str(), node);
			NSetAsBoolean(node, value);
			ReleaseObject(node);
		}

		double GetNodeValueDouble(string node_name) {
			NODE node;
			double value;

			NMGetNode(node_map, node_name.c_str(), node);
			NGetAsFloat(node, value);
			ReleaseObject(node);

			return value;
		}

		bool GetNodeValueBoolean(string node_name) {
			NODE node;
			cvbbool_t value;

			NMGetNode(node_map, node_name.c_str(), node);
			NGetAsBoolean(node, value);
			ReleaseObject(node);

			return value ? true : false;
		}

		int GetNodeValueInteger(string node_name) {
			NODE node;
			cvbint64_t value;

			NMGetNode(node_map, node_name.c_str(), node);
			NGetAsInteger(node, value);
			ReleaseObject(node);

			return (int)value;
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

		void Gain(double value) { SetNodeValue("Gain", value); }
		double Gain() { return GetNodeValueDouble("Gain"); }

		void ExposureTime(double value) { SetNodeValue("ExposureTime", value); }
		double ExposureTime() { return GetNodeValueDouble("ExposureTime"); }

		void AutoBrightness(bool value) { SetNodeValue("autoBrightnessMode", value ? 1 : 0); }
		bool AutoBrightness() { return (GetNodeValue("autoBrightnessMode") == "Active") ? true : false; }

		void BalanceRatioRed(double value) {
			SetNodeValue("BalanceRatioSelector", "Red");
			SetNodeValue("BalanceRatio", value);
		}

		double BalanceRatioRed() {
			SetNodeValue("BalanceRatioSelector", "Red");
			return GetNodeValueDouble("BalanceRatio");
		}

		void BalanceRatioGreen(double value) {
			SetNodeValue("BalanceRatioSelector", "Green");
			SetNodeValue("BalanceRatio", value);
		}

		double BalanceRatioGreen() {
			SetNodeValue("BalanceRatioSelector", "Green");
			return GetNodeValueDouble("BalanceRatio");
		}

		void BalanceRatioBlue(double value) {
			SetNodeValue("BalanceRatioSelector", "Blue");
			SetNodeValue("BalanceRatio", value);
		}

		double BalanceRatioBlue() {
			SetNodeValue("BalanceRatioSelector", "Blue");
			return GetNodeValueDouble("BalanceRatio");
		}

		void TriggerMode(bool value) { SetNodeValue("TriggerMode", value ? 1 : 0); }
		bool TriggerMode() { return (GetNodeValueInteger("TriggerMode") == 1) ? true : false; }

		void TriggerSoftware() { SetNodeValue("TriggerSoftware", true); }

		long long GetTimeStamp() {
			//calculate timestamp in nanoseconds
			G2GetGrabStatus(hCamera, GRAB_INFO_CMD::GRAB_INFO_TIMESTAMP, t_now);
			if (t_start == 0.0)
				t_start = t_now;
			long long timestamp = (long long)(t_now - t_start);
			return timestamp;
		}

		cv::Mat& Capture() {
			while (G2Wait(hCamera) < 0);
//			if (G2Wait(hCamera) >= 0) {
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
//			}
			return image;
		}
	};
}