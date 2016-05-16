#pragma once
#include "DeviceInput.h"
#include "FileInput.h"

namespace pcl
{

	template <typename PointT>
	class BasicGrabber
	{
		int platform, device;
		string file_name;
		Grabber* grabber;
		DeviceInput device_input;
		FileInput<PointT> file_input;
		double fps;
		bool repeat;

	public:
		BasicGrabber() : platform(0), device(0), grabber(0), fps(30.), repeat(false) 
		{
		}

		~BasicGrabber()
		{
			Stop();
		}

		void Platform(int value) { platform = value; }

		void Device(int value) { device = value; }

		void File(string value) { file_name = value; device = -1; }

		void FPS(double value) { fps = value; }

		void Repeat(bool value) { repeat = value; }

		Grabber* GetGrabber() { return grabber; }

		void Init()
		{
			if (device == -1)
				grabber = file_input.GetGrabber(file_name, fps, repeat);
			else
				grabber = device_input.GetGrabber(platform, device);
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