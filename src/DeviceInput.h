#pragma once
#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>

#ifdef HAVE_OPENNI2
#include <pcl/io/openni2_grabber.h>
#endif
#ifdef HAVE_OPENNI
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#endif
#ifdef HAVE_ENSENSO
#include <pcl/io/ensenso_grabber.h>
#endif

using namespace std;

namespace pcl
{
	enum PlatformType
	{
		OPENNI2_DEVICE,
		OPENNI_DEVICE,
		ENSENSO_DEVICE,
		KINECT2NATIVE_DEVICE,
		NO_DEVICE
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
			supported_platforms.push_back(OPENNI2_DEVICE);
#endif
#ifdef HAVE_OPENNI
			supported_platforms.push_back(OPENNI_DEVICE);
#endif
#ifdef HAVE_ENSENSO
			supported_platforms.push_back(ENSENSO_DEVICE);
#endif
		}

		~DeviceInput()
		{
			if (grabber)
			{
				#ifdef HAVE_ENSENSO
				if (platform_type == ENSENSO_DEVICE)
					((EnsensoGrabber*)grabber)->closeDevice();
				#endif

				delete grabber;
			}
		}

		void ListAllDevices()
		{
#ifdef HAVE_OPENNI2
			//OpenNI2
			{
				io::openni2::OpenNI2DeviceManager device_manager;

				size_t nr_of_devices = device_manager.getNumOfConnectedDevices();

				cerr << "- OpenNI2 devices -" << endl;

				for (size_t i = 0; i < nr_of_devices; i++)
				{
					cerr << " Device " << i << ": ";
					cerr << device_manager.getDeviceByIndex(i)->getStringID() << endl;
				}
			}
#endif

#ifdef HAVE_OPENNI
			//OpenNI
			{
				openni_wrapper::OpenNIDriver& device_manager = openni_wrapper::OpenNIDriver::getInstance();

				unsigned int nr_of_devices = device_manager.getNumberDevices();

				cerr << "- OpenNI devices -" << endl;

				for (unsigned int i = 0; i < nr_of_devices; i++)
				{
					cerr << " Device " << i << ": ";
					cerr << device_manager.getProductName(i) << endl;
				}
			}
#endif

#ifdef HAVE_ENSENSO
			//Ensenso SDK
			{
				pcl::EnsensoGrabber device_manager;

				int nr_of_devices = device_manager.enumDevices();

				cerr << "- Ensenso devices -" << endl;

				for (int i = 0; i < nr_of_devices; i++)
				{
				}
			}
#endif
		}

		const Grabber* GetGrabber(int platform = 0, int device = 0)
		{
			if (grabber)
				throw new pcl::PCLException("DeviceInput::GetGrabber, deviced already initalised.");

			if (platform >= supported_platforms.size())
				throw new pcl::PCLException("DeviceInput::GetGrabber, wrong platform number.");

#ifdef HAVE_OPENNI2
			if (!grabber && (supported_platforms[platform] == OPENNI2_DEVICE))
			{
				io::openni2::OpenNI2DeviceManager device_manager;
				size_t nr_of_devices = device_manager.getNumOfConnectedDevices();
				if (device > nr_of_devices)
					throw new pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");
				ostringstream device_str;
				device_str << device;
				grabber = new io::OpenNI2Grabber(device_str.str());
			}
#endif

#ifdef HAVE_OPENNI
				if (!grabber && (supported_platforms[platform] == OPENNI_DEVICE))
				{
					openni_wrapper::OpenNIDriver& device_manager = openni_wrapper::OpenNIDriver::getInstance();
					unsigned int nr_of_devices = device_manager.getNumberDevices();

					if (device > nr_of_devices)
						throw new pcl::PCLException("DeviceInput::GetGrabber, wrong device number.");
					ostringstream device_str;
					device_str << device;
					grabber = new OpenNIGrabber(device_str.str());
				}
#endif

#ifdef HAVE_ENSENSO
				if (!grabber && (supported_platforms[platform] == ENSENSO_DEVICE))
				{
					grabber = new EnsensoGrabber();
					((EnsensoGrabber*)grabber)->openTcpPort();
					((EnsensoGrabber*)grabber)->openDevice(device);
				}
#endif
				if (!grabber)
					throw new pcl::PCLException("DeviceInput::GetGrabber, could not initalise the specified device.");

				platform_type = supported_platforms[platform];
				return grabber;
			}
	
	};
}