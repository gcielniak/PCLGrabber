#pragma once
#include "DeviceInput.h"
#include "FileInput.h"

namespace pcl
{
	class BasicGrabber
	{
		int platform, device;
		string file_name;
		Grabber* grabber;
		DeviceInput device_input;
		FileInput file_input;
		double fps;
		bool repeat;

	public:
		BasicGrabber() : platform(0), device(0), grabber(0), fps(0.), repeat(false) 
		{
		}

		void Platform(int value) { platform = value; }

		void Device(int value) { device = value; }

		void File(string value) { file_name = value; }

		void FPS(double value) { fps = value; }

		void Repeat(bool value) { repeat = value; }

		Grabber* GetGrabber() { return grabber; }

		void Init()
		{
			if (file_name.empty())
				grabber = device_input.GetGrabber(platform, device);
			else
				grabber = file_input.GetGrabber(file_name, fps, repeat);
		}

		void Start()
		{
			if (grabber)
				grabber->start();
		}

		void Stop()
		{
			if (grabber)
				grabber->stop();
		}
	};
}