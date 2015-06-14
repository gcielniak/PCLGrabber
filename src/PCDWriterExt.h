#pragma once
#include <pcl/visualization/boost.h>
#include <pcl/io/pcd_io.h>

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

	template <typename T> class PointCloud;
	template <typename PointT>	class PCDWriterExt
	{
		int image_counter;
		string output_data_path;

	public:
		PCDWriterExt() : image_counter(0), output_data_path(".\\data\\" + currentDateTime() + "\\") {}

		void WritePCDCloud(const boost::shared_ptr<const pcl::PointCloud<PointT> >& cloud)
		{
			string timestamp = boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time());

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream file_name;
			file_name << std::setw(7) << std::setfill('0') << image_counter++ << ".pcd";
			pcl::io::savePCDFile(output_data_path + file_name.str(), *cloud, true);
			//log file
			ofstream myfile;
			myfile.open(output_data_path + "log.txt", ios::app | ios::out);
			myfile << file_name.str() << " " << timestamp << endl;
			myfile.close();
		}
	};
}