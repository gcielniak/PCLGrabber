#pragma once
#include <pcl/visualization/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/io/image_depth.h>

using namespace std;

namespace pcl
{
	// Get current date/time, format is YYYYMMDDTHHmmss
	const std::string currentDateTime()
	{
		ostringstream ss;
		ss.imbue(std::locale(ss.getloc(), new boost::posix_time::time_facet("%Y%m%dT%H%M%S")));
		ss << boost::posix_time::second_clock::local_time();
		return ss.str();
	}

	class PCDWriterExt
	{
		int image_counter;
		string output_data_path;
	public:
		PCDWriterExt() : image_counter(0), output_data_path(".\\data\\" + currentDateTime() + "\\") {}

		template <typename T> class PointCloud;
		template <typename PointT>	
		void WritePCDCloud(const boost::shared_ptr<const pcl::PointCloud<PointT> >& cloud)
		{
			string timestamp = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream file_name;
			file_name << "frame_" << timestamp << "_depth.pcd";
			pcl::io::savePCDFile(output_data_path + file_name.str(), *cloud, true);
		}

		void WriteLZFDepthImage(const boost::shared_ptr<pcl::io::DepthImage>& depth_image)
		{
			pcl::io::LZFDepth16ImageWriter writer;

			string time_string = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream depth_file_name;
			std::stringstream xml_file_name;
			depth_file_name << "frame_" << time_string << "_depth.pclzf";
			xml_file_name << "frame_" << time_string << ".xml";
			writer.write(reinterpret_cast<const char*> (depth_image->getData()), depth_image->getWidth(), depth_image->getHeight(), output_data_path + depth_file_name.str());
			writer.writeParameters(parameters_depth, output_data_path + xml_file_name.str());
		}

		io::CameraParameters parameters_depth;
	};
}