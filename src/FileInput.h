#pragma once
#include <pcl/pcl_config.h>
#include <pcl/exceptions.h>
#include <pcl/visualization/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image.h>
#include <string>
#include <sstream>

using namespace std;

namespace pcl
{
	class FileInput
	{
	private:
		Grabber* grabber;

	public:
		FileInput() : grabber(0) {}

		Grabber* GetGrabber(const string& file_name)
		{
			return grabber;
		}
	};
}