#pragma once
#include <pcl/visualization/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/image.h>
#include <string>
#include <sstream>

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

	class FileOutput
	{
		int image_counter;
		string output_data_path;
	public:
		FileOutput() : image_counter(0), output_data_path(".\\data\\" + currentDateTime() + "\\") {}

		template <typename T> class PointCloud;
		template <typename PointT>
		void WriteCloudPCD(const boost::shared_ptr<const pcl::PointCloud<PointT> >& cloud)
		{
			string timestamp = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream file_name;
			file_name << "frame_" << timestamp << "_depth.pcd";
			pcl::io::savePCDFile(output_data_path + file_name.str(), *cloud, true);
		}

		void WriteDepthImageLZF(const boost::shared_ptr<pcl::io::DepthImage>& depth_image)
		{
			pcl::io::LZFDepth16ImageWriter writer;

			string time_string = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream depth_file_name;
			std::stringstream xml_file_name;
			depth_file_name << "frame_" << time_string << "_depth.pclzf";
			xml_file_name << "frame_" << time_string << ".xml";

			io::CameraParameters parameters_depth;
			parameters_depth.focal_length_x = parameters_depth.focal_length_y = depth_image->getFocalLength();
			parameters_depth.principal_point_x = (depth_image->getWidth() - 1.f) / 2.f;
			parameters_depth.principal_point_y = (depth_image->getHeight() - 1.f) / 2.f;

			writer.write(reinterpret_cast<const char*> (depth_image->getData()), depth_image->getWidth(), depth_image->getHeight(), output_data_path + depth_file_name.str());
			writer.writeParameters(parameters_depth, output_data_path + xml_file_name.str());
		}

		void WriteImageLZF(const boost::shared_ptr<pcl::io::Image>& color_image, const boost::shared_ptr<pcl::io::DepthImage>& depth_image)
		{
			pcl::io::LZFDepth16ImageWriter depth_writer;
			pcl::io::LZFRGB24ImageWriter color_writer;

			string time_string = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream depth_file_name;
			std::stringstream color_file_name;
			std::stringstream xml_file_name;
			depth_file_name << "frame_" << time_string << "_depth.pclzf";
			color_file_name << "frame_" << time_string << "_rgb.pclzf";
			xml_file_name << "frame_" << time_string << ".xml";

			io::CameraParameters depth_parameters;
			depth_parameters.focal_length_x = depth_parameters.focal_length_y = depth_image->getFocalLength();
			depth_parameters.principal_point_x = (depth_image->getWidth() - 1.f) / 2.f;
			depth_parameters.principal_point_y = (depth_image->getHeight() - 1.f) / 2.f;
			depth_writer.writeParameters(depth_parameters, output_data_path + xml_file_name.str());

			depth_writer.write(reinterpret_cast<const char*> (depth_image->getData()), depth_image->getWidth(), depth_image->getHeight(), output_data_path + depth_file_name.str());
			color_writer.write(reinterpret_cast<const char*> (color_image->getData()), color_image->getWidth(), color_image->getHeight(), output_data_path + color_file_name.str());
		}
	};
}
