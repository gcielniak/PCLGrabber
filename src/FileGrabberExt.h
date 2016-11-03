#pragma once
#include <pcl/io/image_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include "ImageReaderExt.h"

#ifdef HAVE_KINECT2_NATIVE
#include "Kinect2NativeGrabber.h"
#endif

namespace PCLGrabber
{
	using namespace std;

	//A set of utilities for handling input files
	namespace FileGrabberUtils {
		//return all file names matching the extension ext in a given directory dir
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

		//check if there is at least one file name with extension ext in a given directory dir
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

		//extract timestamp from a file name in microseconds
		//the timestamp format is used by ImageGrabber
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
	}

	//interface class which defines common grabber signals in a templated form
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

	//Extended ImageGrabber class.
	//Provides support for Kinect2 grabber, including storing original colour images in full resolution
	//and a colour-registered depth image (requires an active Kinect2 device connected to PC) 
	template <typename PointT, typename ImageT, typename DepthT>
	class ImageGrabberExt : public pcl::ImageGrabber < PointT >, public ImageSignals<ImageT, DepthT>
	{
		//extra signals for Kinect2
		typedef void (Signal_ImageDepthImage)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, const boost::shared_ptr<ImageT>&);
		typedef void (Signal_ImageDepthImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&, const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthT>&);

	protected:
		bool pclzf_mode;
		string dir;

	protected:
		boost::signals2::signal<Signal_ImageDepthImage>* signal_ImageDepthImage;
		boost::signals2::signal<Signal_ImageDepthImageDepth>* signal_ImageDepthImageDepth;

	public:
		ImageGrabberExt(const std::string& dir_, float frames_per_second = 0, bool repeat = false) :
			ImageGrabber<PointT>(CheckFiles(dir_), frames_per_second, repeat, pclzf_mode), dir(dir_), signal_ImageDepthImage(nullptr)
		{
			signal_Depth = createSignal<Signal_Depth>();
			signal_Image = createSignal<Signal_Image>();
			signal_ImageDepth = createSignal<Signal_ImageDepth>();

			//check if the folder contains original colour images (used with Kinect2)
			if (FileGrabberUtils::FileExtInDir(boost::filesystem::path(dir_), ".pclzf_"))
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

		boost::shared_ptr<ImageT> ToRGB24Image(const string& file_name, vector<unsigned char>& color_buffer, const string& postfix = "rgb") const
		{
			string color_file_name = file_name;
			color_file_name.replace(color_file_name.end() - 5, color_file_name.end(), postfix);
			if (pclzf_mode) {
				color_file_name += ".pclzf";
				if (postfix == "orig")
					color_file_name += "_";
			}
			else
				color_file_name += ".png";

			pcl::LZFRGB24ImageReaderExt<ImageT> rgb_reader;

			return rgb_reader.read(color_file_name, color_buffer, pcl::LZFRGB24ImageReaderExt<ImageT>::CF_RGB);
		}

		boost::shared_ptr<DepthT> ToDepthImage(const string& file_name, vector<unsigned short>& depth_buffer, bool registered = false) const
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
				Kinect2Grabber<PointT, ImageT, DepthT> grabber;
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
		/**
		* Implements a new publish() method that generates also standard images and extra for Kinect2
		*/
		virtual void publish(const pcl::PCLPointCloud2& blob, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orientation) const {
			//standard publish method genereates points clouds only
			ImageGrabber::publish(blob, origin, orientation);

			static int file_index = 0;

			static vector<unsigned char> image_buffer, image_orig_buffer;
			static vector<unsigned short> depth_buffer, depth_reg_buffer;

			boost::shared_ptr<DepthT> depth, depth_reg;
			boost::shared_ptr<ImageT> image, image_orig;
			string file_name = dir + "\\" + getDepthFileNameAtIndex(file_index);

			if (signal_Depth->num_slots() || signal_ImageDepth->num_slots() ||
				(signal_ImageDepthImage && signal_ImageDepthImage->num_slots()) || signal_ImageDepthImageDepth->num_slots())
				depth = ToDepthImage(file_name, depth_buffer);

			if (signal_Image->num_slots() || signal_ImageDepth->num_slots() ||
				(signal_ImageDepthImage && signal_ImageDepthImage->num_slots()) || signal_ImageDepthImageDepth->num_slots())
				image = ToRGB24Image(file_name, image_buffer);

			if ((signal_ImageDepthImage && signal_ImageDepthImage->num_slots()) || signal_ImageDepthImageDepth->num_slots())
				image_orig = ToRGB24Image(file_name, image_orig_buffer, "orig");

			if (signal_ImageDepthImageDepth->num_slots())
				depth_reg = ToDepthImage(file_name, depth_reg_buffer, true);

			if (signal_Depth->num_slots())
				signal_Depth->operator()(depth);

			if (signal_Image->num_slots())
				signal_Image->operator()(image);

			if (signal_ImageDepth->num_slots())
				signal_ImageDepth->operator()(image, depth, 0.0);

			if (signal_ImageDepthImage && signal_ImageDepthImage->num_slots())
				signal_ImageDepthImage->operator()(image, depth, image_orig);

			if (signal_ImageDepthImageDepth->num_slots())
				signal_ImageDepthImageDepth->operator()(image, depth, image_orig, depth_reg);

			cerr << file_name << endl;

			file_index++;
		}

		string const& CheckFiles(std::string const& path_name) {
			pclzf_mode = true;
			boost::filesystem::path path(path_name);

			//check if the path is valid
			if (!boost::filesystem::exists(path))
				PCL_THROW_EXCEPTION(pcl::IOException, "No valid file name given!\n");

			if (boost::filesystem::is_directory(path)) {
				if (FileGrabberUtils::FileExtInDir(path, ".pclzf")) {
				}
				else if (FileGrabberUtils::FileExtInDir(path, ".png")) {
					pclzf_mode = false;
				}
				else {
					PCL_THROW_EXCEPTION(pcl::IOException, "No PCLZF/PNG files in the specified directory!\n");
				}
			}

			return path_name;
		}
	};

	//Extended PCDGrabber class.
	//adds universal image signals which work with OpenNI, OpenNI2, OpenCV, etc.
	template <typename PointT, typename ImageT, typename DepthT>
	class PCDGrabberExt : public pcl::PCDGrabber<PointT>, public ImageSignals<ImageT, DepthT> {
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