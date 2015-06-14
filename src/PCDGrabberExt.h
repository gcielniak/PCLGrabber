#pragma once

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <pcl/io/pcd_grabber.h>

namespace pcl
{
	template <typename PointT>
	Grabber* PCDGrabberExt(const std::string& pcd_path, float frames_per_second = 0, bool repeat = false)
	{
		namespace fs = boost::filesystem;
		fs::path dir(pcd_path);

		//check if pcd_path is a folder
		if (fs::is_directory(dir))
		{
			if (dir.empty() || !fs::exists(dir))
				PCL_THROW_EXCEPTION(pcl::IOException, "No valid PCD directory given!\n");

			std::vector<std::string> pcd_files;
			fs::directory_iterator pos(dir);
			fs::directory_iterator end;

			for (; pos != end; ++pos)
				if (fs::is_regular_file(pos->status()))
					if (fs::extension(*pos) == ".pcd")
					{
#if BOOST_FILESYSTEM_VERSION == 3
				pcd_files.push_back(pos->path().string());
#else
				pcd_files.push_back(pos->path());
#endif
					}
			return new PCDGrabber<PointT>(pcd_files, frames_per_second, repeat);
		}
		else
		{
			return new PCDGrabber<PointT>(pcd_path, frames_per_second, repeat);
		}
	}

	template <typename PointT>
	Grabber* PCDGrabberExt(const std::vector<std::string>& pcd_files, float frames_per_second = 0, bool repeat = false)
	{
		return new PCDGrabber<PointT>(pcd_files, frames_per_second, repeat);
	}
}