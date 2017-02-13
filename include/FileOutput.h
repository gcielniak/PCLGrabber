#pragma once
#include <pcl/visualization/boost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/io/png_io.h>
#include <string>
#include <sstream>
#include <typeinfo>
#include "ImageUtils.h"

namespace PCLGrabber
{
	using namespace std;
	using namespace pcl;

	// Get current date/time, format is YYYYMMDDTHHmmss
	const std::string currentDateTime()
	{
		ostringstream ss;
		ss.imbue(std::locale(ss.getloc(), new boost::posix_time::time_facet("%Y%m%dT%H%M%S")));
		ss << boost::posix_time::second_clock::local_time();
		return ss.str();
	}

	class FileOutputBase {
	protected:
		int format;
		string output_data_path;
		bool simulated_time;

	public:
		FileOutputBase() :
			output_data_path(".\\data\\" + currentDateTime() + "\\"), format(-1), simulated_time(false) {
		}

		virtual void RegisterCallbacks(Grabber* grabber) = 0;

		void Format(int value) { format = value; }
		int Format() { return format; }

		string OutputDir() { return output_data_path; }
		void OutputDir(const string new_path) { output_data_path = new_path; }

		void SimulatedTime(bool value) { simulated_time = value; }
		bool SimulatedTime() { return simulated_time; }
	};

	template <typename PointT, typename ImageT, typename DepthImageT>
	class FileOutput : public FileOutputBase {
		boost::posix_time::ptime sensor_time_start, sensor_timestamp_first;

	public:
		FileOutput() : sensor_timestamp_first(boost::posix_time::not_a_date_time) {}

		void WritePCD(const boost::shared_ptr<const PointCloud<PointT> >& cloud) {

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			//cloud timestamp: can be simulated from filename or timestamp from the sensor
			boost::posix_time::ptime cloud_timestamp = from_us(cloud->header.stamp);

			//if timestamp from sensor then calculate offset from the current time
			if (!simulated_time) {
				if (sensor_timestamp_first.is_not_a_date_time()) {//first frame
					sensor_time_start = boost::posix_time::microsec_clock::local_time();
					sensor_timestamp_first = cloud_timestamp;
				}
				//correct timestamp
				cloud_timestamp += sensor_time_start - sensor_timestamp_first;
			}

			std::stringstream file_name;

			file_name << "frame_" << boost::posix_time::to_iso_string(cloud_timestamp) << ".pcd";

			io::savePCDFileBinaryCompressed<PointT>(output_data_path + file_name.str(), *cloud);

			FPS_CALC("WRITE PCD");
		}

		boost::posix_time::ptime from_us(long long us)
		{
			return boost::posix_time::from_time_t(us / 1000000) + boost::posix_time::microseconds(us % 1000000);
		}

		void WriteImage(const boost::shared_ptr<ImageT>& color_image, const boost::shared_ptr<DepthImageT>& depth_image, const boost::shared_ptr<ImageT>& orig_image)
		{
			io::LZFDepth16ImageWriter depth_writer;
			io::LZFRGB24ImageWriter color_writer;

			//cloud timestamp: can be simulated from filename or timestamp from the sensor
			boost::posix_time::ptime depth_timestamp = from_us(GetTimeStamp(depth_image));

			//if timestamp from sensor then calculate offset from the current time
			if (!simulated_time) {
				if (sensor_timestamp_first.is_not_a_date_time()) {//first frame
					sensor_time_start = boost::posix_time::microsec_clock::local_time();
					sensor_timestamp_first = depth_timestamp;
				}
				//correct
				depth_timestamp += sensor_time_start - sensor_timestamp_first;
			}

			string time_string = boost::posix_time::to_iso_string(depth_timestamp);

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream depth_file_name;
			std::stringstream color_file_name;
			std::stringstream orig_file_name;
			std::stringstream xml_file_name;
			if (format == 0)
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

			cout << "Writing out the frame \"" << output_data_path << "frame_" << time_string << "\"" << endl;
			if (depth_image)
			{
				cout << "Writing the depth image to " << output_data_path + depth_file_name.str() << endl;
				io::CameraParameters depth_parameters;
				depth_parameters.focal_length_x = depth_parameters.focal_length_y = GetFocalLength(depth_image);
				depth_parameters.principal_point_x = (GetWidth(depth_image) - 1.f) / 2.f;
				depth_parameters.principal_point_y = (GetHeight(depth_image) - 1.f) / 2.f;
				depth_writer.writeParameters(depth_parameters, output_data_path + xml_file_name.str());

				if (format == 0)
					depth_writer.write((const char*)GetDepthBuffer(depth_image), GetWidth(depth_image), GetHeight(depth_image), output_data_path + depth_file_name.str());
				else
#ifdef HAVE_OPENCV
					cv::imwrite(output_data_path + depth_file_name.str(), cv::Mat(GetHeight(depth_image), GetWidth(depth_image), CV_16UC1, (void*)GetDepthBuffer(depth_image), GetWidth(depth_image) * 2));
#else
					io::saveShortPNGFile(output_data_path + depth_file_name.str(), GetDepthBuffer(depth_image), GetWidth(depth_image), GetHeight(depth_image), 1);
#endif
			}

			if (color_image)
			{
				cout << "Writing the color image to " << output_data_path + color_file_name.str() << endl;
				if (format == 0)
					color_writer.write((const char*)GetRGBBuffer(color_image), GetWidth(color_image), GetHeight(color_image), output_data_path + color_file_name.str());
				else
#ifdef HAVE_OPENCV
					cv::imwrite(output_data_path + color_file_name.str(), cv::Mat(GetHeight(color_image), GetWidth(color_image), CV_8UC3, (void*)GetRGBBuffer(color_image), GetWidth(color_image) * 3));
#else
					io::saveRgbPNGFile(output_data_path + color_file_name.str(), GetRGBBuffer(color_image), GetWidth(color_image), GetHeight(color_image));
#endif
			}
			if (orig_image)
			{
				cout << "Writing the orig image to " << output_data_path + orig_file_name.str() << endl;
				if (format == 0)
					color_writer.write((const char*)GetRGBBuffer(orig_image), GetWidth(orig_image), GetHeight(orig_image), output_data_path + orig_file_name.str());
				else
#ifdef HAVE_OPENCV
					cv::imwrite(output_data_path + orig_file_name.str(), cv::Mat(GetHeight(orig_image), GetWidth(orig_image), CV_8UC3, (void*)GetRGBBuffer(orig_image), GetWidth(orig_image) * 3));
#else
					io::saveRgbPNGFile(output_data_path + orig_file_name.str(), GetRGBBuffer(orig_image), GetWidth(orig_image), GetHeight(orig_image));
#endif
			}

			FPS_CALC("WRITE IMAGE");
		}

		void WriteImage2(const boost::shared_ptr<CvMatExt>& color_image, const boost::shared_ptr<CvMatExt>& ir_image) {
			io::LZFBayer8ImageWriter ir_writer;
			io::LZFRGB24ImageWriter color_writer;

			//cloud timestamp: can be simulated from filename or timestamp from the sensor
			boost::posix_time::ptime ir_timestamp = from_us(GetTimeStamp(ir_image));

			//if timestamp from sensor then calculate offset from the current time
			if (!simulated_time) {
				if (sensor_timestamp_first.is_not_a_date_time()) {//first frame
					sensor_time_start = boost::posix_time::microsec_clock::local_time();
					sensor_timestamp_first = ir_timestamp;
				}
				//correct
				ir_timestamp += sensor_time_start - sensor_timestamp_first;
			}

			string time_string = boost::posix_time::to_iso_string(ir_timestamp);

			if (!boost::filesystem::exists(boost::filesystem::path(output_data_path)))
				boost::filesystem::create_directories(boost::filesystem::path(output_data_path));

			std::stringstream ir_file_name;
			std::stringstream color_file_name;
			std::stringstream xml_file_name;

			if (format == 0)
			{
				ir_file_name << "frame_" << time_string << "_depth.pclzf";
				color_file_name << "frame_" << time_string << "_rgb.pclzf";
			}
			else
			{
				ir_file_name << "frame_" << time_string << "_depth.png";
				color_file_name << "frame_" << time_string << "_rgb.png";
			}

			xml_file_name << "frame_" << time_string << ".xml";

			cout << "Writing out the frame \"" << output_data_path << "frame_" << time_string << "\"" << endl;
			if (ir_image)
			{
				cout << "Writing the ir image to " << output_data_path + ir_file_name.str() << endl;
				if (format == 0)
					ir_writer.write((const char*)ir_image->image.data, GetWidth(ir_image), GetHeight(ir_image), output_data_path + ir_file_name.str());
				else
					cv::imwrite(output_data_path + ir_file_name.str(), cv::Mat(GetHeight(ir_image), GetWidth(ir_image), CV_8UC1, (void*)ir_image->image.data, GetWidth(ir_image)));
				std::ofstream stream(output_data_path + xml_file_name.str());
			}

			if (color_image)
			{
				cout << "Writing the color image to " << output_data_path + color_file_name.str() << endl;
				if (format == 0)
					color_writer.write((const char*)GetRGBBuffer(color_image), GetWidth(color_image), GetHeight(color_image), output_data_path + color_file_name.str());
				else
					cv::imwrite(output_data_path + color_file_name.str(), cv::Mat(GetHeight(color_image), GetWidth(color_image), CV_8UC3, (void*)GetRGBBuffer(color_image), GetWidth(color_image) * 3));
			}

			FPS_CALC("WRITE IMAGE 2");
		}

		virtual void RegisterCallbacks(Grabber* grabber)
		{
			if ((format == 0) || (format == 2))
			{
				if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&)> f_image =
						boost::bind(&FileOutput::WriteImage, this, _1, _2, _3);
					grabber->registerCallback(f_image);
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float flength)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float flength)> f_write =
						boost::bind(&FileOutput::WriteImage, this, _1, _2, boost::shared_ptr<ImageT>());
					grabber->registerCallback(f_write);
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<CvMatExt>&, const boost::shared_ptr<CvMatExt>&)>()) {
					boost::function<void(const boost::shared_ptr<CvMatExt>&, const boost::shared_ptr<CvMatExt>&)> f_image =
						boost::bind(&FileOutput::WriteImage2, this, _1, _2);
					grabber->registerCallback(f_image);
				}
			}
			else if (format == 1)
			{
				boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_write =
					boost::bind(&FileOutput::WritePCD, this, _1);
				grabber->registerCallback(f_write);

			}
			else if (format == 3)
			{
				if (grabber->providesCallback<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&)>())
				{
					boost::function<void(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&)> f_image =
						boost::bind(&FileOutput::WriteImage, this, _3, _4, boost::shared_ptr<ImageT>());
					grabber->registerCallback(f_image);
				}
				else if (grabber->providesCallback<void(const boost::shared_ptr<CvMatExt>&, const boost::shared_ptr<CvMatExt>&)>()) {
					boost::function<void(const boost::shared_ptr<CvMatExt>&, const boost::shared_ptr<CvMatExt>&)> f_image =
						boost::bind(&FileOutput::WriteImage2, this, _1, _2);
					grabber->registerCallback(f_image);
				}
			}
		}

	};
}
