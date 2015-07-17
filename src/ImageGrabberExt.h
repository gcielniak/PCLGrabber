#pragma once
#include <pcl/io/image_grabber.h>

namespace pcl
{
	template <typename T> class PointCloud;
	template <typename PointT>
	class ImageGrabberExt : public ImageGrabber < PointT >
	{
	public:
		ImageGrabberExt(const std::string& dir,
			float frames_per_second = 0,
			bool repeat = false,
			bool pclzf_mode = false) : ImageGrabber(dir, frames_per_second, repeat, pclzf_mode)
		{
		}

		ImageGrabberExt(const std::string& depth_dir,
			const std::string& rgb_dir,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber(depth_dir, rgb_dir, frames_per_second, repeat)
		{
		}

		ImageGrabberExt(const std::vector<std::string>& depth_image_files,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber(depth_image_files, frames_per_second, repeat)
		{
		}
	};
}