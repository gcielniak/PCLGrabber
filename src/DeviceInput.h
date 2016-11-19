#pragma once
#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef HAVE_OPENNI2
#include <pcl/io/openni2_grabber.h>
#endif
#ifdef HAVE_OPENNI
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#endif
#ifdef HAVE_ENSENSO
#include "EnsensoGrabberExt.h"
#endif
#ifdef HAVE_KINECT2_NATIVE
#include "Kinect2NativeGrabber.h"
#endif
#ifdef HAVE_REAL_SENSE
#include "RealSenseGrabberExt.h"
#endif

using namespace std;

namespace PCLGrabber {

	using namespace pcl;

	enum PlatformType
	{
		OPENNI2_PLATFORM,
		OPENNI_PLATFORM,
		ENSENSO_PLATFORM,
		KINECT2_NATIVE_PLATFORM,
		REALSENSE_PLATFORM,
		NO_PLATFORM
	};

	using namespace pcl::io;
	using namespace pcl::io::openni2;

	//OpenNI2Grabber with extended functionality which makes Asus sensors compatible with mirrored image from Kinect
	// TODO: fix the first frame on Asus sensor which is not flipped
	class OpenNI2GrabberExt : public io::OpenNI2Grabber {
	private:
		bool mirrored_color, mirrored_depth;

	public:
		OpenNI2GrabberExt(const string& device_id = "") :
			io::OpenNI2Grabber(device_id), mirrored_color(false), mirrored_depth(false) {

			// callbacks from the sensor to the grabber
			device_->setColorCallback(boost::bind(&OpenNI2GrabberExt::processColorFrameExt, this, _1));
			device_->setDepthCallback(boost::bind(&OpenNI2GrabberExt::processDepthFrameExt, this, _1));
		}

		void processColorFrameExt(openni::VideoStream& stream) {
			if (!mirrored_color) {
				stream.setMirroringEnabled(true);
				mirrored_color = true;
			}

			this->processColorFrame(stream);
		}

		void processDepthFrameExt(openni::VideoStream& stream) {
			if (!mirrored_depth) {
				stream.setMirroringEnabled(true);
				mirrored_depth = true;
			}
			this->processDepthFrame(stream);
		}
	};

	class DeviceInput
	{
	private:
		Grabber* grabber;
		PlatformType platform_type;

	public:
		DeviceInput() : grabber(0), platform_type(NO_PLATFORM) {
		}

		~DeviceInput() {
			if (grabber) {

#ifdef HAVE_ENSENSO
				if (platform_type == ENSENSO_PLATFORM)
					((EnsensoGrabber*)grabber)->closeDevice();
#endif
				delete grabber;
			}
		}

		static void GetSupportedPlatforms(vector<PlatformType>& supported_platforms) {
#ifdef HAVE_REAL_SENSE
			supported_platforms.push_back(REALSENSE_PLATFORM);
#endif
#ifdef HAVE_OPENNI2
			supported_platforms.push_back(OPENNI2_PLATFORM);
#endif
#ifdef HAVE_OPENNI
			supported_platforms.push_back(OPENNI_PLATFORM);
#endif
#ifdef HAVE_ENSENSO
			supported_platforms.push_back(ENSENSO_PLATFORM);
#endif
#ifdef HAVE_KINECT2_NATIVE
			supported_platforms.push_back(KINECT2_NATIVE_PLATFORM);
#endif
		}

		static void GetPresentPlatforms(vector<PlatformType>& supported_platforms) {
#ifdef HAVE_REAL_SENSE
			try {
				RealSenseGrabber* rsgrabber = new RealSenseGrabber("");
				delete rsgrabber;
				supported_platforms.push_back(REALSENSE_PLATFORM);
			}
			catch (pcl::io::IOException) {}
#endif
#ifdef HAVE_OPENNI2
			io::openni2::OpenNI2DeviceManager device_manager_oni2;
			if (device_manager_oni2.getNumOfConnectedDevices())
				supported_platforms.push_back(OPENNI2_PLATFORM);
#endif
#ifdef HAVE_OPENNI
			openni_wrapper::OpenNIDriver& device_manager_oni = openni_wrapper::OpenNIDriver::getInstance();
			if (device_manager_oni.getNumberDevices())
				supported_platforms.push_back(OPENNI_PLATFORM);
#endif
#ifdef HAVE_ENSENSO
			supported_platforms.push_back(ENSENSO_PLATFORM);
#endif
#ifdef HAVE_KINECT2_NATIVE
			BOOLEAN sensor_present;
			IKinectSensor* sensor;
			GetDefaultKinectSensor(&sensor);
			sensor->get_IsAvailable(&sensor_present);
			if (sensor_present)
				supported_platforms.push_back(KINECT2_NATIVE_PLATFORM);
#endif
		}

		PlatformType GetPlatformType() {
			return platform_type;
		}

		static void ListAllDevices() {
			vector<PlatformType> supported_platforms;
			GetSupportedPlatforms(supported_platforms);

			cerr << "Supported platforms:";

			for (unsigned int i = 0; i < supported_platforms.size(); i++) {
				switch (supported_platforms[i]) {
				case REALSENSE_PLATFORM:
					cerr << " RealSense";
					break;
				case OPENNI2_PLATFORM:
					cerr << " OpenNI2";
					break;
				case OPENNI_PLATFORM:
					cerr << " OpenNI";
					break;
				case ENSENSO_PLATFORM:
					cerr << " Ensenso SDK";
					break;
				case KINECT2_NATIVE_PLATFORM:
					cerr << " Kinect2 Native";
					break;
				default:
					break;
				}
				if (i < (supported_platforms.size() - 1))
					cerr << ",";
			}

			cerr << endl;

			supported_platforms.clear();

			GetPresentPlatforms(supported_platforms);

			cerr << "Present platforms:";

			for (unsigned int i = 0; i < supported_platforms.size(); i++) {
				switch (supported_platforms[i]) {
				case REALSENSE_PLATFORM:
					cerr << " RealSense";
					break;
				case OPENNI2_PLATFORM:
					cerr << " OpenNI2";
					break;
				case OPENNI_PLATFORM:
					cerr << " OpenNI";
					break;
				case ENSENSO_PLATFORM:
					cerr << " Ensenso SDK";
					break;
				case KINECT2_NATIVE_PLATFORM:
					cerr << " Kinect2 Native";
					break;
				default:
					break;
				}
				if (i < (supported_platforms.size() - 1))
					cerr << ",";
			}

			cerr << endl;

			for (unsigned int i = 0; i < supported_platforms.size(); i++) {

#ifdef HAVE_REAL_SENSE
				if (supported_platforms[i] == REALSENSE_PLATFORM) {
					try {
						RealSenseGrabber* rsgrabber = new RealSenseGrabber("");
						cerr << " Device " << 0 << ": ";
						cerr << rsgrabber->getName() << endl;
						delete rsgrabber;
					}
					catch (pcl::io::IOException) {}
				}
#endif

#ifdef HAVE_OPENNI2
				if (supported_platforms[i] == OPENNI2_PLATFORM) {
					cerr << "Platform " << i << ": OpenNI2" << endl;

					io::openni2::OpenNI2DeviceManager device_manager;
					size_t nr_of_devices = device_manager.getNumOfConnectedDevices();

					for (size_t j = 0; j < nr_of_devices; j++) {
						boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = device_manager.getDeviceByIndex(j);

						cerr << " Device " << j << ": ";
						cerr << device->getStringID() << endl;
						cerr << "  depth FL: " << device->getDepthFocalLength() << endl;
						cerr << "  color FL: " << device->getColorFocalLength() << endl;
						cerr << "  baseline: " << device->getBaseline() << endl;
					}
				}
#endif

#ifdef HAVE_OPENNI
				if (supported_platforms[i] == OPENNI_PLATFORM) {
					cerr << "Platform " << i << ": OpenNI" << endl;

					openni_wrapper::OpenNIDriver& device_manager = openni_wrapper::OpenNIDriver::getInstance();

					unsigned int nr_of_devices = device_manager.getNumberDevices();

					for (unsigned int j = 0; j < nr_of_devices; j++) {
						boost::shared_ptr<openni_wrapper::OpenNIDevice> device = device_manager.getDeviceByIndex(j);
						cerr << " Device " << j << ": ";
						cerr << device->getProductName() << endl;
						cerr << "  depth FL: " << device->getDepthFocalLength() << endl;
						cerr << "  color FL: " << device->getImageFocalLength() << endl;
					}
				}
#endif

#ifdef HAVE_ENSENSO
				if (supported_platforms[i] == ENSENSO_PLATFORM)
				{
					cerr << "Platform " << i << ": Ensenso SDK" << endl;

					pcl::EnsensoGrabber device_manager;

					int nr_of_devices = device_manager.enumDevices();

					for (int j = 0; j < nr_of_devices; j++)
					{
					}
				}
#endif

#ifdef HAVE_KINECT2_NATIVE
				if (supported_platforms[i] == KINECT2_NATIVE_PLATFORM) {
					BOOLEAN sensor_present;
					IKinectSensor* sensor;
					GetDefaultKinectSensor(&sensor);
					sensor->get_IsAvailable(&sensor_present);
					if (sensor_present) {
						cerr << "Platform " << i << ": Kinect2 Native" << endl;
						cerr << " Device " << 0 << ": ";
						cerr << "Kinect2" << endl;
					}
				}
#endif
			}
		}

		Grabber* GetGrabber(int platform = 0, int device = 0)
		{
#ifdef HAVE_OPENCV
			typedef CvMatExt ImageT;
			typedef CvMatExt DepthT;
#elif HAVE_OPENNI2
			typedef io::Image ImageT;
			typedef io::DepthImage DepthT;
#elif HAVE_OPENNI
			typedef openni_wrapper::Image ImageT;
			typedef openni_wrapper::DepthImage DepthT;
#endif

			vector<PlatformType> supported_platforms;
			GetSupportedPlatforms(supported_platforms);

			if (grabber)
				throw pcl::PCLException("DeviceInput::GetGrabber, deviced already initalised.");

			if (platform >= supported_platforms.size())
				throw pcl::PCLException("DeviceInput::GetGrabber, wrong platform number.");

#ifdef HAVE_OPENNI2
			if (!grabber && (supported_platforms[platform] == OPENNI2_PLATFORM))
			{
				io::openni2::OpenNI2DeviceManager device_manager;
				size_t nr_of_devices = device_manager.getNumOfConnectedDevices();
				if (device >= nr_of_devices)
					throw pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");
				ostringstream device_str;
				device_str << "#" << device + 1;
				io::OpenNI2Grabber* g = new OpenNI2GrabberExt(device_str.str());
				grabber = g;
			}
#endif

#ifdef HAVE_OPENNI
			if (!grabber && (supported_platforms[platform] == OPENNI_PLATFORM))
			{
				openni_wrapper::OpenNIDriver& device_manager = openni_wrapper::OpenNIDriver::getInstance();
				unsigned int nr_of_devices = device_manager.getNumberDevices();

				if (device >= nr_of_devices)
					throw pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");
				ostringstream device_str;
				device_str << "#" << device + 1;
				grabber = new OpenNIGrabber(device_str.str());
			}
#endif

#ifdef HAVE_ENSENSO
			if (!grabber && (supported_platforms[platform] == ENSENSO_PLATFORM))
			{
				//freopen("out2.txt", "a", stdout);

				pcl::EnsensoGrabber device_manager;
				int nr_of_devices = device_manager.enumDevices();

				pcl::EnsensoGrabber device_manager2;

				if (device >= nr_of_devices)
					throw pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");

				grabber = new EnsensoGrabberExt(device);
			}
#endif

#ifdef HAVE_KINECT2_NATIVE
			if (!grabber && (supported_platforms[platform] == KINECT2_NATIVE_PLATFORM))
				grabber = new Kinect2Grabber<PointXYZRGBA, ImageT, DepthT>();
#endif

#ifdef HAVE_REAL_SENSE
			if (!grabber && (supported_platforms[platform] == REALSENSE_PLATFORM))
				grabber = new RealSenseGrabberExt<PointXYZRGBA, ImageT, DepthT>("");
#endif

			if (!grabber)
				throw pcl::PCLException("DeviceInput::GetGrabber, could not initalise the specified device.");

			platform_type = supported_platforms[platform];
			return grabber;
		}

	};
}
