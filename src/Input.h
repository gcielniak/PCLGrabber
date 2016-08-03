#pragma once
#include <pcl/io/openni2_grabber.h>

class Input {

public:
};

class OpenNI2Input : public Input, pcl::io::OpenNI2Grabber {
public:
	OpenNI2Input(string device) : pcl::io::OpenNI2Grabber(device) {
	};
	
	static void ListAllDevices() {
		pcl::io::openni2::OpenNI2DeviceManager device_manager;
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
};


