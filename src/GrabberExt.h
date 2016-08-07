#pragma once
#include <pcl/io/image_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include "ImageReaderExt.h"

#ifdef HAVE_KINECT2_NATIVE
#include "kinect2_grabber.h"
#endif

namespace pcl
{
	using namespace std;

	template <typename PointT, typename ImageT, typename DepthImageT>
	class ImageGrabberExt : public ImageGrabber < PointT >
	{
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float focal_length);
		typedef void (Signal_ImageDepthImage)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&);
		typedef void (Signal_ImageDepthImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&);

	protected:
		string dir;
		bool pclzf_mode;
		vector<unsigned char> color_buffer, orig_buffer;
		vector<unsigned short> depth_buffer;
		int file_index;
		bool swap_rb_channels;

	protected:
		using Grabber::createSignal;
		using Grabber::disconnect_all_slots;
		
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;
		boost::signals2::signal<Signal_ImageDepthImage>* signal_ImageDepthImage;
		boost::signals2::signal<Signal_ImageDepthImageDepth>* signal_ImageDepthImageDepth;
#ifdef HAVE_KINECT2_NATIVE
		Kinect2Grabber<PointT, ImageT, DepthImageT> grabber;
#endif

	public:
		ImageGrabberExt(const std::string& dir_,
			float frames_per_second = 0,
			bool repeat = false,
			bool pclzf_mode_ = false, bool swap_rb_channels_=false) : ImageGrabber<PointT>(dir_, frames_per_second, repeat, pclzf_mode_), dir(dir_), pclzf_mode(pclzf_mode_), file_index(0)
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr), signal_ImageDepthImageDepth(nullptr), swap_rb_channels(swap_rb_channels_)
		{
			boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_cloud =
				boost::bind(&ImageGrabberExt<PointT, ImageT, DepthImageT>::GetImage, this, _1);
			this->registerCallback(f_cloud);

			signal_ImageDepth = createSignal<Signal_ImageDepth>();
			signal_ImageDepthImage = createSignal<Signal_ImageDepthImage>();
			signal_ImageDepthImageDepth = createSignal<Signal_ImageDepthImageDepth>();
		}

		ImageGrabberExt(const std::string& depth_dir,
			const std::string& rgb_dir,
			float frames_per_second = 0,
			bool repeat = false, bool swap_rb_channels_ = false) : ImageGrabber<PointT>(depth_dir, rgb_dir, frames_per_second, repeat), pclzf_mode(false), file_index(0)
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr), signal_ImageDepthImageDepth(nullptr), swap_rb_channels(swap_rb_channels_)
		{
		}

		ImageGrabberExt(const std::vector<std::string>& depth_image_files,
			float frames_per_second = 0,
			bool repeat = false, bool swap_rb_channels_ = false) : ImageGrabber<PointT>(depth_image_files, frames_per_second, repeat), pclzf_mode(false), file_index(0)
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr), signal_ImageDepthImageDepth(nullptr), swap_rb_channels(swap_rb_channels_)
		{
		}

		virtual ~ImageGrabberExt() throw()
		{
			disconnect_all_slots<Signal_ImageDepth>();
			disconnect_all_slots<Signal_ImageDepthImage>();
			disconnect_all_slots<Signal_ImageDepthImageDepth>();
		}

		void GetImage(const boost::shared_ptr<const PointCloud<PointT> >&)
		{
			if (signal_ImageDepth->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getDepthFileNameAtIndex(file_index);
				signal_ImageDepth->operator()(ToRGB24Image(file_name), ToDepthImage(file_name), 1.0);
			}
			if (signal_ImageDepthImage->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getDepthFileNameAtIndex(file_index);
				signal_ImageDepthImage->operator()(ToRGB24Image(file_name), ToDepthImage(file_name), ToRGB24OrigImage(file_name));
			}
#ifdef HAVE_KINECT2_NATIVE
			if ((signal_ImageDepthImageDepth->num_slots() > 0) && grabber.IsAvailable())
			{
				string file_name = dir + "\\" + this->getDepthFileNameAtIndex(file_index);
				signal_ImageDepthImageDepth->operator()(ToRGB24Image(file_name), ToDepthImage(file_name), ToRGB24OrigImage(file_name), ToDepthImageReg(file_name));
			}
#endif
			file_index++;
		}

		boost::shared_ptr<ImageT> ToRGB24Image(const string& file_name)
		{
			string color_file_name = file_name;
			color_file_name.replace(color_file_name.end() - 5, color_file_name.end(), "rgb");
			if (pclzf_mode)
				color_file_name += ".pclzf";
			else
				color_file_name += ".png";

			pcl::LZFRGB24ImageReaderExt<ImageT> rgb_reader;

			pcl::LZFRGB24ImageReaderExt<ImageT>::ColorFormat color_format = pcl::LZFRGB24ImageReaderExt<ImageT>::CF_BGR;

			if (swap_rb_channels)
				color_format = pcl::LZFRGB24ImageReaderExt<ImageT>::CF_RGB;

			return rgb_reader.read(color_file_name, color_buffer, color_format);
		}

		boost::shared_ptr<ImageT> ToRGB24OrigImage(const string& file_name)
		{
			string orig_file_name = file_name;
			orig_file_name.replace(orig_file_name.end() - 5, orig_file_name.end(), "orig");
			if (pclzf_mode)
				orig_file_name += ".pclzf_";
			else
				orig_file_name += ".png_";

			pcl::LZFRGB24ImageReaderExt<ImageT> rgb_reader;

			pcl::LZFRGB24ImageReaderExt<ImageT>::ColorFormat color_format = pcl::LZFRGB24ImageReaderExt<ImageT>::CF_BGR;

			if (swap_rb_channels)
				color_format = pcl::LZFRGB24ImageReaderExt<ImageT>::CF_RGB;

			return rgb_reader.read(orig_file_name, orig_buffer, color_format);
		}

		boost::shared_ptr<DepthImageT> ToDepthImage(const string& file_name)
		{
			string depth_file_name = file_name;
			if (pclzf_mode)
				depth_file_name += ".pclzf";
			else
				depth_file_name += ".png";

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(), ".xml");

			pcl::LZFDepth16ImageReaderExt<DepthImageT> depth_reader;
			io::CameraParameters params;

			depth_reader.readParameters(xml_file_name);
			params = depth_reader.getParameters();

			return depth_reader.read(depth_file_name, depth_buffer, params.focal_length_x);
		}

#ifdef HAVE_KINECT2_NATIVE
		boost::shared_ptr<DepthImageT> ToDepthImageReg(const string& file_name)
		{
			string depth_file_name = file_name;
			if (pclzf_mode)
				depth_file_name += ".pclzf";
			else
				depth_file_name += ".png";

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(), ".xml");

			pcl::LZFDepth16ImageReaderExt<DepthImageT> depth_reader;
			io::CameraParameters params;

			depth_reader.readParameters(xml_file_name);
			params = depth_reader.getParameters();
			boost::shared_ptr<DepthImageT> depth_orig = depth_reader.read(depth_file_name, depth_buffer, params.focal_length_x);

			//perform depth2rgb registration

			grabber.mapping_updated = false;
			return grabber.convertDepthImageReg(depth_buffer, GetTimeStamp(depth_orig));
		}
#endif
	};

	template <typename PointT>
	class PCDGrabberExt : public PCDGrabber<PointT> {
	public:
		PCDGrabberExt(const std::vector<std::string>& pcd_files, float frames_per_second = 0, bool repeat = false) :
			PCDGrabber(pcd_files, frames_per_second, repeat) {}

		PCDGrabberExt(const std::string& pcd_path, float frames_per_second = 0, bool repeat = false) :
			PCDGrabber(pcd_path, frames_per_second, repeat) {}

	protected:
		/**
		 * Implements a new publish() method that also updates the cloud timestamp based on the pcd filename.
		 */
		virtual void publish(const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation, const std::string& file_name) const {
			//
			pcl::uint64_t timestamp;
			getTimestampFromFilepath(file_name, timestamp);

			pcl::PCLPointCloud2& my_blob = const_cast<pcl::PCLPointCloud2&>(blob);//very naughty!
			my_blob.header.stamp = timestamp;
			PCDGrabber::publish(blob, origin, orientation, file_name);
		}

	public:
		static bool getTimestampFromFilepath(const std::string &filepath, pcl::uint64_t &timestamp) {

			// For now, we assume the file is of the form frame_[22-char POSIX timestamp]_*
			char timestamp_str[256];
			int result = std::sscanf(boost::filesystem::basename(filepath).c_str(),
				"frame_%22s_%*s", timestamp_str);
			if (result > 0) {
				// Convert to pcl::uint64_t, microseconds since 1970-01-01
				boost::posix_time::ptime cur_date = boost::posix_time::from_iso_string(timestamp_str);
				boost::posix_time::ptime zero_date(
					boost::gregorian::date(1970, boost::gregorian::Jan, 1));
				timestamp = (cur_date - zero_date).total_microseconds();
				return (true);
			}
			return (false);
		}
	};
}