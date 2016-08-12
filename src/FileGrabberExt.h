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

	class FileGrabberUtils {
	public:
		static void FileNamebyExt(boost::filesystem::path const& dir, string const& ext, vector<string>& file_names) {
			boost::filesystem::directory_iterator pos(dir);
			boost::filesystem::directory_iterator end;

			for (; pos != end; ++pos) {
				if (boost::filesystem::is_regular_file(pos->status())) {
					if (boost::filesystem::extension(*pos) == ext) {
#if BOOST_FILESYSTEM_VERSION == 3
						file_names.push_back(pos->path().string());
#else
						file_names.push_back(pos->path());
#endif
					}
				}
			}
		}

		static bool FileExtInDir(boost::filesystem::path& dir, string ext)
		{
			boost::filesystem::directory_iterator pos(dir);
			boost::filesystem::directory_iterator end;

			for (; pos != end; ++pos)
				if (boost::filesystem::is_regular_file(pos->status()))
					if (boost::filesystem::extension(*pos) == ext)
						return true;

			return false;
		}

		static bool TimestampFromFilepath(const std::string &filepath, pcl::uint64_t &timestamp) {

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

	template <typename ImageT, typename DepthT>
	class ImageSignals {
	protected:
		typedef void (Signal_Depth)(const boost::shared_ptr<DepthT>&);
		typedef void (Signal_Image)(const boost::shared_ptr<ImageT>&);
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, float focal_length);

		boost::signals2::signal<Signal_Image>* signal_Image;
		boost::signals2::signal<Signal_Depth>* signal_Depth;
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;
	};

	template <typename PointT, typename ImageT, typename DepthT>
	class ImageGrabberExt : public ImageGrabber < PointT >, public ImageSignals<ImageT, DepthT>
	{
		//extra signals for Kinect2
		typedef void (Signal_ImageDepthImage)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, const boost::shared_ptr<ImageT>&);
		typedef void (Signal_ImageDepthImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&);

	protected:
		bool pclzf_mode;
		string dir;
		vector<unsigned char> color_buffer, orig_buffer;
		vector<unsigned short> depth_buffer;
		int file_index;
		bool swap_rb_channels;

	protected:
		boost::signals2::signal<Signal_ImageDepthImage>* signal_ImageDepthImage;
		boost::signals2::signal<Signal_ImageDepthImageDepth>* signal_ImageDepthImageDepth;
#ifdef HAVE_KINECT2_NATIVE
		Kinect2Grabber<PointT, ImageT, DepthT> grabber;
#endif

	protected:
		/**
		* Implements a new publish() method that generates also images
		*/
		virtual void publish(const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation) {
			ImageGrabber::publish(blob, origin, orientation);

			boost::shared_ptr<DepthT> depth, depth_reg;
			boost::shared_ptr<ImageT> image, image_orig;
			string file_name = dir + "\\" + getDepthFileNameAtIndex(file_index);

			if (signal_Depth->num_slots() || signal_ImageDepth->num_slots() || signal_ImageDepthImage->num_slots() || signal_ImageDepthImageDepth->num_slots())
				depth = ToDepthImage(file_name);

			if (signal_Image->num_slots() || signal_ImageDepth->num_slots() || signal_ImageDepthImage->num_slots() || signal_ImageDepthImageDepth->num_slots())
				image = ToRGB24Image(file_name);

			if (signal_ImageDepthImage->num_slots() || signal_ImageDepthImageDepth->num_slots())
				image_orig = ToRGB24Image(file_name,"orig");

			if (signal_ImageDepthImageDepth->num_slots())
				depth_reg = ToDepthImage(file_name, true);

			if (signal_Depth->num_slots())
				signal_Depth->operator()(depth);

			if (signal_Image->num_slots())
				signal_Image->operator()(image);

			if (signal_ImageDepth->num_slots())
				signal_ImageDepth->operator()(image, depth, 0.0);

			if (signal_ImageDepthImage->num_slots())
				signal_ImageDepthImage->operator()(image, depth, image_orig);

			if (signal_ImageDepthImageDepth->num_slots())
				signal_ImageDepthImageDepth->operator()(image, depth, image_orig, depth_reg);

			file_index++;
		}

	public:
		ImageGrabberExt(const std::string& dir_, float frames_per_second = 0, bool repeat = false, bool swap_rb_channels_ = false) :
			ImageGrabber<PointT>(GetFileNames(dir_), frames_per_second, repeat), dir(dir_), file_index(0), swap_rb_channels(swap_rb_channels_)
		{
			signal_Depth = createSignal<Signal_Depth>();
			signal_Image = createSignal<Signal_Image>();
			signal_ImageDepth = createSignal<Signal_ImageDepth>();

			signal_ImageDepthImage = createSignal<Signal_ImageDepthImage>();
			signal_ImageDepthImageDepth = createSignal<Signal_ImageDepthImageDepth>();
		}

		virtual ~ImageGrabberExt() throw()
		{
			disconnect_all_slots<Signal_Depth>();
			disconnect_all_slots<Signal_Image>();
			disconnect_all_slots<Signal_ImageDepth>();

			disconnect_all_slots<Signal_ImageDepthImage>();
			disconnect_all_slots<Signal_ImageDepthImageDepth>();
		}

		boost::shared_ptr<ImageT> ToRGB24Image(const string& file_name, const string& postfix="rgb")
		{
			string color_file_name = file_name;
			color_file_name.replace(color_file_name.end() - 5, color_file_name.end(), postfix);
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

		boost::shared_ptr<DepthT> ToDepthImage(const string& file_name, bool registered=false)
		{
			string depth_file_name = file_name;
			if (pclzf_mode)
				depth_file_name += ".pclzf";
			else
				depth_file_name += ".png";

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(), ".xml");

			pcl::LZFDepth16ImageReaderExt<DepthT> depth_reader;
			io::CameraParameters params;

			depth_reader.readParameters(xml_file_name);
			params = depth_reader.getParameters();
			boost::shared_ptr<DepthT> depth = depth_reader.read(depth_file_name, depth_buffer, params.focal_length_x);

			if (registered) {
#ifdef HAVE_KINECT2_NATIVE
				//perform depth2rgb registration
				if (grabber.IsAvailable()) {
					grabber.mapping_updated = false;
					return grabber.convertDepthImageReg(depth_buffer, GetTimeStamp(depth));
				}
				else {
					return nullptr;
				}
#elif
				return nullptr;
#endif
			}
			else {
				return depth;
			}
		}

	protected:
		vector<string> GetFileNames(std::string const& path_name) {
			pclzf_mode = true;
			vector<string> file_names;
			boost::filesystem::path path(path_name);

			//check if the path is valid
			if (!boost::filesystem::exists(path))
				PCL_THROW_EXCEPTION(pcl::IOException, "No valid file name given!\n");

			if (boost::filesystem::is_directory(path))
				FileGrabberUtils::FileNamebyExt(path, ".pclzf", file_names); //scan for pclzf files

			if (!file_names.size()) {
				FileGrabberUtils::FileNamebyExt(path, ".png", file_names); //scan for png files
				pclzf_mode = false;
			}

			if (!file_names.size())
				PCL_THROW_EXCEPTION(pcl::IOException, "No PCLZF/PNG files in the specified directory!\n");

			return file_names;
		}
	};

	template <typename PointT, typename ImageT, typename DepthT>
	class PCDGrabberExt : public PCDGrabber<PointT>, public ImageSignals<ImageT, DepthT> {
	public:
		PCDGrabberExt(const std::string& pcd_path, float frames_per_second = 0, bool repeat = false) :
			PCDGrabber(GetPCDFileNames(pcd_path), frames_per_second, repeat) {
			//disconnect the original OpenNI signals
#ifdef HAVE_OPENNI
			remove_signal<void(const boost::shared_ptr<openni_wrapper::DepthImage>&)>();
			remove_signal<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float)>();
			remove_signal<void(const boost::shared_ptr<openni_wrapper::Image>&)>();
#endif

			//create new universal signals
			signal_Depth = createSignal <Signal_Depth>();
			signal_Image = createSignal <Signal_Image>();
			signal_ImageDepth = createSignal <Signal_ImageDepth>();
		}

		virtual ~PCDGrabberExt() throw() {
			disconnect_all_slots<Signal_Depth>();
			disconnect_all_slots<Signal_Image>();
			disconnect_all_slots<Signal_ImageDepth>();
		}

	protected:
		template<typename T>
		void remove_signal() {
			std::map<std::string, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find(typeid (T).name());
			if (signal_it != signals_.end())
				signals_.erase(signal_it);
		}

		/**
		 * Implements a new publish() method that also updates the cloud timestamp based on the pcd filename.
		 * Adds new image/depth signal based on templates so OpenNI/OpenNI2/OpenCV image formats are accepted.
		 */
		virtual void publish(const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation, const std::string& file_name) const {
			typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

			pcl::fromPCLPointCloud2(blob, *cloud);
			cloud->sensor_origin_ = origin;
			cloud->sensor_orientation_ = orientation;
			pcl::uint64_t timestamp;
			FileGrabberUtils::TimestampFromFilepath(file_name, timestamp);
			cloud->header.stamp = timestamp;

			if (signal_->num_slots())
				signal_->operator () (cloud);

			if (file_name_signal_->num_slots())
				file_name_signal_->operator()(file_name);

			boost::shared_ptr<DepthT> depth;
			boost::shared_ptr<ImageT> image;

			if (signal_Depth->num_slots() || signal_ImageDepth->num_slots())
				depth = ToDepthImage<PointT, DepthT>(cloud);

			if (signal_Image->num_slots() || signal_ImageDepth->num_slots())
				image = ToImage<PointT, ImageT>(cloud);

			if (signal_Depth->num_slots())
				signal_Depth->operator()(depth);

			if (signal_Image->num_slots())
				signal_Image->operator()(image);

			if (signal_ImageDepth->num_slots())
				signal_ImageDepth->operator()(image, depth, 0.0);
		}

		static vector<string> GetPCDFileNames(std::string const& path_name) {
			vector<string> file_names;
			boost::filesystem::path path(path_name);

			//check if the path is valid
			if (!boost::filesystem::exists(path))
				PCL_THROW_EXCEPTION(pcl::IOException, "No valid file name given!\n");

			if (boost::filesystem::is_directory(path)) //scan for pcd files if directory given
				FileGrabberUtils::FileNamebyExt(path, ".pcd", file_names);
			else //add a single pcd file
				file_names.push_back(path_name);

			if (!file_names.size())
				PCL_THROW_EXCEPTION(pcl::IOException, "No PCD files in the specified directory!\n");

			return file_names;
		}
	};
}