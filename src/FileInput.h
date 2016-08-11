#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <pcl/exceptions.h>
#include "GrabberExt.h"

using namespace std;

namespace pcl
{
	template <typename PointT, typename ImageT, typename DepthT>
	class FileInput
	{
	private:
		Grabber* grabber;

	public:
		FileInput() : grabber(0) {}

		Grabber* GetGrabber(const string& file_name, float frames_per_second = 0, bool repeat = false, bool swap_rb_channels=false)
		{
			if (grabber)
				return grabber;

			//first check if the file is a directory
			boost::filesystem::path dir(file_name);

			//check if pcd_path is a folder
			if (boost::filesystem::is_directory(dir))
			{
				if (dir.empty() || !boost::filesystem::exists(dir))
					PCL_THROW_EXCEPTION(pcl::IOException, "No valid directory given!\n");

				if (FilesInDir(dir, ".pcd"))
				{
					grabber = new PCDGrabberExt<PointT, ImageT, DepthT>(file_name, frames_per_second, repeat);
					return grabber;
				}
				else if (FilesInDir(dir, ".pclzf"))
				{
					grabber = new ImageGrabberExt<PointT, ImageT, DepthT>(file_name, frames_per_second, repeat, true, swap_rb_channels);
					return grabber;
				}
				else if (FilesInDir(dir, ".png"))
				{
					grabber = new ImageGrabberExt<PointT, ImageT, DepthT>(file_name, frames_per_second, repeat, false, swap_rb_channels);
					return grabber;
				}
				PCL_THROW_EXCEPTION(pcl::IOException, "No recognised files in the directory given!\n");
			}
			else if (boost::filesystem::extension(dir) == ".pcd") //single file
			{
				grabber = new PCDGrabberExt<PointT, ImageT, DepthT>(file_name, frames_per_second, repeat);
				return grabber;
			}

			PCL_THROW_EXCEPTION(pcl::IOException, "No recognised files in the directory given!\n");

			return grabber;
		}

		static bool FilesInDir(boost::filesystem::path& dir, string ext)
		{
			boost::filesystem::directory_iterator pos(dir);
			boost::filesystem::directory_iterator end;

			for (; pos != end; ++pos)
				if (boost::filesystem::is_regular_file(pos->status()))
					if (boost::filesystem::extension(*pos) == ext)
						return true;

			return false;
		}

		static vector<string> GetFileNames(boost::filesystem::path& dir, string ext)
		{
			vector<string> files;
			boost::filesystem::directory_iterator pos(dir);
			boost::filesystem::directory_iterator end;

			for (; pos != end; ++pos)
			{
				if (boost::filesystem::is_regular_file(pos->status()))
				{
					if (boost::filesystem::extension(*pos) == ext)
					{
#if BOOST_FILESYSTEM_VERSION == 3
						files.push_back(pos->path().string());
#else
						files.push_back(pos->path());
#endif
					}
				}
			}

			return files;
		}
	};
}