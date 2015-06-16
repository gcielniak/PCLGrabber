#pragma once
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/ensenso_grabber.h>
using namespace std;

namespace pcl
{
	class DeviceInput
	{
	public:
		void ListAllDevices()
		{
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

			//Ensenso SDK
			{
				pcl::EnsensoGrabber device_manager;

				int nr_of_devices = device_manager.enumDevices();

				cerr << "- Ensenso devices -" << endl;

				for (int i = 0; i < nr_of_devices; i++)
				{
				}
			}
		}
	};
}