#pragma once
#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef HAVE_OPENNI2
#define HAVE_ONI
#include <pcl/io/openni2_grabber.h>
#endif
#ifdef HAVE_OPENNI
#define HAVE_ONI
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#endif
#ifdef HAVE_ONI
#ifdef HAVE_ENSENSO
#include "EnsensoGrabberExt.h"
#endif
#ifdef HAVE_KINECT2_NATIVE
#include "kinect2_grabber.h"
#endif
#endif

using namespace std;

namespace pcl
{
	enum PlatformType
	{
		OPENNI2_PLATFORM,
		OPENNI_PLATFORM,
		ENSENSO_PLATFORM,
		KINECT2_NATIVE_PLATFORM,
		NO_PLATFORM
	};

	class DeviceInput
	{
	private:
		Grabber* grabber;
		PlatformType platform_type;
		vector<PlatformType> supported_platforms;

	public:
		DeviceInput() : grabber(0)
		{
#ifdef HAVE_OPENNI2
			supported_platforms.push_back(OPENNI2_PLATFORM);
#endif
#ifdef HAVE_OPENNI
			supported_platforms.push_back(OPENNI_PLATFORM);
#endif
#ifdef HAVE_ONI
#ifdef HAVE_ENSENSO
			supported_platforms.push_back(ENSENSO_PLATFORM);
#endif
#ifdef HAVE_KINECT2_NATIVE
			supported_platforms.push_back(KINECT2_NATIVE_PLATFORM);
#endif
#endif
		}

		~DeviceInput()
		{
			if (grabber)
			{

#ifdef HAVE_ENSENSO
				if (platform_type == ENSENSO_PLATFORM)
					((EnsensoGrabber*)grabber)->closeDevice();
#endif
				delete grabber;
			}
		}

		void ListAllDevices()
		{
			for (unsigned int i = 0; i < supported_platforms.size(); i++)
			{

#ifdef HAVE_OPENNI2
				if (supported_platforms[i] == OPENNI2_PLATFORM)
				{
					cerr << "Platform " << i << ": OpenNI2" << endl;

					io::openni2::OpenNI2DeviceManager device_manager;
					size_t nr_of_devices = device_manager.getNumOfConnectedDevices();

					for (size_t j = 0; j < nr_of_devices; j++)
					{
						cerr << " Device " << j << ": ";
						cerr << device_manager.getDeviceByIndex(j)->getStringID() << endl;
						cerr << "  depth FL: " << device_manager.getDeviceByIndex(j)->getDepthFocalLength() << endl;
						cerr << "  color FL: " << device_manager.getDeviceByIndex(j)->getColorFocalLength() << endl;
						cerr << "  IR FL: " << device_manager.getDeviceByIndex(j)->getIRFocalLength() << endl;
						cerr << "  baseline: " << device_manager.getDeviceByIndex(j)->getBaseline() << endl;
						cerr << "  depth reg: " << device_manager.getDeviceByIndex(j)->isDepthRegistered() << endl;
					}
				}
#endif

#ifdef HAVE_OPENNI
				if (supported_platforms[i] == OPENNI_PLATFORM)
				{
					cerr << "Platform " << i << ": OpenNI" << endl;

					openni_wrapper::OpenNIDriver& device_manager = openni_wrapper::OpenNIDriver::getInstance();

					unsigned int nr_of_devices = device_manager.getNumberDevices();

					for (unsigned int j = 0; j < nr_of_devices; j++)
					{
						cerr << " Device " << j << ": ";
						cerr << device_manager.getProductName(j) << endl;
					}
				}
#endif

#ifdef HAVE_ONI
#ifdef HAVE_ENSENSO
				if (supported_platforms[i] == ENSENSO_PLATFORM)
				{
					cerr << "Platform " << i << ": Ensenso" << endl;

					pcl::EnsensoGrabber device_manager;

					int nr_of_devices = device_manager.enumDevices();

					for (int j = 0; j < nr_of_devices; j++)
					{
					}
				}
#endif
#ifdef HAVE_KINECT2_NATIVE
				if (supported_platforms[i] == KINECT2_NATIVE_PLATFORM)
				{
					cerr << "Platform " << i << ": Kinect2 Native" << endl;
				}
#endif
#endif
			}
		}

		Grabber* GetGrabber(int platform = 0, int device = 0)
		{
			if (grabber)
				throw new pcl::PCLException("DeviceInput::GetGrabber, deviced already initalised.");

			if (platform >= supported_platforms.size())
				throw new pcl::PCLException("DeviceInput::GetGrabber, wrong platform number.");

#ifdef HAVE_OPENNI2
			if (!grabber && (supported_platforms[platform] == OPENNI2_PLATFORM))
			{
				io::openni2::OpenNI2DeviceManager device_manager;
				size_t nr_of_devices = device_manager.getNumOfConnectedDevices();
				if (device > nr_of_devices)
					throw new pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");
				ostringstream device_str;
				device_str << "#" << device + 1;
				grabber = new io::OpenNI2Grabber(device_str.str());
			}
#endif

#ifdef HAVE_OPENNI
			if (!grabber && (supported_platforms[platform] == OPENNI_PLATFORM))
			{
				openni_wrapper::OpenNIDriver& device_manager = openni_wrapper::OpenNIDriver::getInstance();
				unsigned int nr_of_devices = device_manager.getNumberDevices();

				if (device > nr_of_devices)
					throw new pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");
				ostringstream device_str;
				device_str << "#" << device + 1;
				grabber = new OpenNIGrabber(device_str.str());
			}
#endif

#ifdef HAVE_ONI
#ifdef HAVE_ENSENSO
			if (!grabber && (supported_platforms[platform] == ENSENSO_PLATFORM))
			{
				//freopen("out2.txt", "a", stdout);

				pcl::EnsensoGrabber device_manager;
				int nr_of_devices = device_manager.enumDevices();

				pcl::EnsensoGrabber device_manager2;

				if (device > nr_of_devices)
					throw new pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");

				grabber = new EnsensoGrabberExt(device);
			}
#endif

#ifdef HAVE_KINECT2_NATIVE
			if (!grabber && (supported_platforms[platform] == KINECT2_NATIVE_PLATFORM))
			{
#ifdef HAVE_OPENNI2
				grabber = new Kinect2Grabber<PointXYZRGBA, io::Image, io::DepthImage>();
#elif HAVE_OPENNI
				grabber = new Kinect2Grabber<PointXYZRGBA, openni_wrapper::Image, openni_wrapper::DepthImage>();
#endif
			}
#endif
#endif			
			if (!grabber)
				throw new pcl::PCLException("DeviceInput::GetGrabber, could not initalise the specified device.");

			platform_type = supported_platforms[platform];
			return grabber;
		}

	};
}