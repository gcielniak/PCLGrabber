#pragma once
#include <pcl/visualization/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/io/png_io.h>
#include <string>
#include <sstream>
#include <typeinfo>
#include "ImageUtils.h"

namespace pcl
{
	using namespace std;

	// Get current date/time, format is YYYYMMDDTHHmmss
	const std::string currentDateTime()
	{
		ostringstream ss;
		ss.imbue(std::locale(ss.getloc(), new boost::posix_time::time_facet("%Y%m%dT%H%M%S")));
		ss << boost::posix_time::second_clock::local_time();
		return ss.str();
	}

	template <typename PointT, typename ImageT, typename DepthImageT>
	class FileOutput
	{
		int image_counter, format;
		string output_data_path;

	public:
		FileOutput() : image_counter(0), output_data_path(".\\data\\" + currentDateTime() + "\\"), format(-1) {}

		void Format(int value) { format = value; }
		int Format() { return format; }

		string OutputDir() { return output_data_path; }
		void OutputDir(const string new_path) { output_data_path = new_path; }

		void WritePCD(const boost::shared_ptr<const PointCloud<PointT> >& cloud)
		{
			string timestamp = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream file_name;
			file_name << "frame_" << timestamp << "_depth.pcd";
			io::savePCDFile<PointT>(output_data_path + file_name.str(), *cloud, true);

			FPS_CALC("WRITE PCD");
		}

		void WriteImage(const boost::shared_ptr<ImageT>& color_image, const boost::shared_ptr<DepthImageT>& depth_image, const boost::shared_ptr<ImageT>& orig_image, bool pclzf_on)
		{
			io::LZFDepth16ImageWriter depth_writer;
			io::LZFRGB24ImageWriter color_writer;

			string time_string = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream depth_file_name;
			std::stringstream color_file_name;
			std::stringstream orig_file_name;
			std::stringstream xml_file_name;
			if (pclzf_on)
			{
				depth_file_name << "frame_" << time_string << "_depth.pclzf";
				color_file_name << "frame_" << time_string << "_rgb.pclzf";
				orig_file_name << "frame_" << time_string << "_orig.pclzf_";
			}
			else
			{
				depth_file_name << "frame_" << time_string << "_depth.png";
				color_file_name << "frame_" << time_string << "_rgb.png";
				orig_file_name << "frame_" << time_string << "_orig.png_";
			}

			xml_file_name << "frame_" << time_string << ".xml";

			if (depth_image != nullptr)
			{
				io::CameraParameters depth_parameters;
				depth_parameters.focal_length_x = depth_parameters.focal_length_y = depth_image->getFocalLength();
				depth_parameters.principal_point_x = (depth_image->getWidth() - 1.f) / 2.f;
				depth_parameters.principal_point_y = (depth_image->getHeight() - 1.f) / 2.f;
				depth_writer.writeParameters(depth_parameters, output_data_path + xml_file_name.str());

				if (pclzf_on)
					depth_writer.write((const char*)GetDepthBuffer(depth_image), depth_image->getWidth(), depth_image->getHeight(), output_data_path + depth_file_name.str());
				else
					io::saveShortPNGFile(output_data_path + depth_file_name.str(), GetDepthBuffer(depth_image), depth_image->getWidth(), depth_image->getHeight(), 1);
			}

			if (color_image != nullptr)
			{
				if (pclzf_on)
					color_writer.write((const char*)GetRGBBuffer(color_image), color_image->getWidth(), color_image->getHeight(), output_data_path + color_file_name.str());
				else
					io::saveRgbPNGFile(output_data_path + color_file_name.str(), GetRGBBuffer(color_image), color_image->getWidth(), color_image->getHeight());
			}
			if (orig_image != nullptr)
			{
				if (pclzf_on)
					color_writer.write((const char*)GetRGBBuffer(orig_image), orig_image->getWidth(), orig_image->getHeight(), output_data_path + orig_file_name.str());
				else
					io::saveRgbPNGFile(output_data_path + orig_file_name.str(), GetRGBBuffer(orig_image), orig_image->getWidth(), orig_image->getHeight());
			}

			FPS_CALC("WRITE IMAGE");
		}

		void RegisterCallbacks(Grabber* grabber)
		{
			switch (format)
			{
			case 0:
			{
				if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)> f_image =
						boost::bind(&FileOutput::WriteImage, this, _1, _2, _3, true);
					grabber->registerCallback(f_image);
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float flength)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)> f_write =
						boost::bind(&FileOutput::WriteImage, this, _1, _2, boost::shared_ptr<io::Image>(), true);
					grabber->registerCallback(f_write);
				}
			}
				break;
			case 1:
			{
				boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_write =
					boost::bind(&FileOutput::WritePCD, this, _1);
				grabber->registerCallback(f_write);
			}
				break;
			case 2:
			{
				if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)> f_image =
						boost::bind(&FileOutput::WriteImage, this, _1, _2, _3, false);
					grabber->registerCallback(f_image);
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float flength)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)> f_write =
						boost::bind(&FileOutput::WriteImage, this, _1, _2, boost::shared_ptr<io::Image>(), false);
					grabber->registerCallback(f_write);
				}
			}
				break;
			default:
				break;
			}
		}
	};
}
