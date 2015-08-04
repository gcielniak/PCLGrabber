#pragma once
#include <pcl/io/image_grabber.h>
#include "ImageReaderExt.h"

namespace pcl
{
	using namespace std;

	template <typename T> class PointCloud;
	template <typename PointT>
	class ImageGrabberExt : public ImageGrabber < PointT >
	{
#ifdef HAVE_OPENNI2
		typedef void (Signal_ImageDepth)(const io::Image::Ptr&, const io::DepthImage::Ptr&, float reciprocalFocalLength);
		typedef void (Signal_ImageDepthImage)(const io::Image::Ptr&, const io::DepthImage::Ptr&, const io::Image::Ptr&);
#endif
#ifdef HAVE_OPENNI
		typedef void (Signal_ImageDepthOni)(const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float reciprocalFocalLength);
		typedef void (Signal_ImageDepthImageOni)(const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, openni_wrapper::Image::Ptr&);
#endif

	protected:
		string dir;
		bool pclzf_mode;
		vector<unsigned char> color_buffer, orig_buffer;
		vector<unsigned short> depth_buffer;

	protected:
#ifdef HAVE_OPENNI2
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;
		boost::signals2::signal<Signal_ImageDepthImage>* signal_ImageDepthImage;
#endif

#ifdef HAVE_OPENNI
		boost::signals2::signal<Signal_ImageDepthOni>* signal_ImageDepthOni;
		boost::signals2::signal<Signal_ImageDepthImageOni>* signal_ImageDepthImageOni;
#endif

	public:
		ImageGrabberExt(const std::string& dir_,
			float frames_per_second = 0,
			bool repeat = false,
			bool pclzf_mode_ = false) : ImageGrabber<PointT>(dir_, frames_per_second, repeat, pclzf_mode_), dir(dir_), pclzf_mode(pclzf_mode_)
#ifdef HAVE_OPENNI2
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr)
#endif
#ifdef HAVE_OPENNI
			, signal_ImageDepthOni(nullptr), signal_ImageDepthImageOni(nullptr)
#endif
		{
#ifdef HAVE_OPENNI2
			boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_cloud =
				boost::bind(&ImageGrabberExt<PointT>::GetImage, this, _1);
			this->registerCallback(f_cloud);

			signal_ImageDepth = createSignal<Signal_ImageDepth>();
			signal_ImageDepthImage = createSignal<Signal_ImageDepthImage>();
#endif
#ifdef HAVE_OPENNI
			boost::function<void(boost::shared_ptr<const PointCloud<PointT> >&)> f_cloud_oni =
				boost::bind(&ImageGrabberExt<PointT>::GetImageOni, this, _1);
			this->registerCallback(f_cloud_oni);

			signal_ImageDepthOni = createSignal<Signal_ImageDepthOni>();
			signal_ImageDepthImageOni = createSignal<Signal_ImageDepthImageOni>();
#endif
		}

		ImageGrabberExt(const std::string& depth_dir,
			const std::string& rgb_dir,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber<PointT>(depth_dir, rgb_dir, frames_per_second, repeat), pclzf_mode(false)
#ifdef HAVE_OPENNI2
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr)
#endif
#ifdef HAVE_OPENNI
			, signal_ImageDepthOni(nullptr), signal_ImageDepthImageOni(nullptr)
#endif
		{
		}

		ImageGrabberExt(const std::vector<std::string>& depth_image_files,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber<PointT>(depth_image_files, frames_per_second, repeat), pclzf_mode(false)
#ifdef HAVE_OPENNI2
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr)
#endif
#ifdef HAVE_OPENNI
			, signal_ImageDepthOni(nullptr), signal_ImageDepthImageOni(nullptr)
#endif
		{
		}

		~ImageGrabberExt() throw()
		{
#ifdef HAVE_OPENNI2
			disconnect_all_slots<Signal_ImageDepth>();
			disconnect_all_slots<Signal_ImageDepthImage>();
#endif
#ifdef HAVE_OPENNI
			disconnect_all_slots<Signal_ImageDepthOni>();
			disconnect_all_slots<Signal_ImageDepthImageOni>();
#endif
		}

#ifdef HAVE_OPENNI2
		void GetImage(const boost::shared_ptr<const PointCloud<PointT> >&)
		{
			if (signal_ImageDepth->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_ImageDepth->operator()(ToRGB24Image(file_name), ToDepthImage(file_name), 1.0);
			}
			if (signal_ImageDepthImage->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_ImageDepthImage->operator()(ToRGB24Image(file_name), ToDepthImage(file_name), ToRGB24OrigImage(file_name));
			}
		}

		io::Image::Ptr ToRGB24Image(const string& file_name)
		{
			string color_file_name = file_name;
			color_file_name.replace(color_file_name.end() - 5, color_file_name.end(), "rgb");
			if (pclzf_mode)
				color_file_name += ".pclzf";
			else
				color_file_name += ".png";

			pcl::LZFRGB24ImageReaderExt rgb_reader;

			return rgb_reader.read(color_file_name, color_buffer);
		}

		io::Image::Ptr ToRGB24OrigImage(const string& file_name)
		{
			string orig_file_name = file_name;
			orig_file_name.replace(orig_file_name.end() - 5, orig_file_name.end(), "orig");
			if (pclzf_mode)
				orig_file_name += ".pclzf_";
			else
				orig_file_name += ".png_";

			pcl::LZFRGB24ImageReaderExt rgb_reader;

			return rgb_reader.read(orig_file_name, orig_buffer);
		}

		io::DepthImage::Ptr ToDepthImage(const string& file_name)
		{
			string depth_file_name = file_name;
			if (pclzf_mode)
				depth_file_name += ".pclzf";
			else
				depth_file_name += ".png";

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(), ".xml");

			pcl::LZFDepth16ImageReaderExt depth_reader;
			io::CameraParameters params;

			depth_reader.readParameters(xml_file_name);
			params = depth_reader.getParameters();

			return depth_reader.read(depth_file_name, depth_buffer, params.focal_length_x);
		}
#endif

#ifdef HAVE_OPENNI
		void GetImageOni(const boost::shared_ptr<const PointCloud<PointT> >&)
		{
			if (signal_ImageDepthOni->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_ImageDepthOni->operator()(ToRGB24ImageOni(file_name), ToDepthImageOni(file_name), 1.0);
			}
			if (signal_ImageDepthImageOni->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_ImageDepthImageOni->operator()(ToRGB24ImageOni(file_name), ToDepthImageOni(file_name), ToRGB24OrigImageOni(file_name));
			}
		}

		openni_wrapper::Image::Ptr ToRGB24ImageOni(const string& file_name)
		{
			string color_file_name = file_name;
			color_file_name.replace(color_file_name.end() - 5, color_file_name.end(), "rgb");
			if (pclzf_mode)
				color_file_name += ".pclzf";
			else
				color_file_name += ".png";

			pcl::LZFRGB24ImageReaderExt rgb_reader;

			return rgb_reader.readOni(color_file_name, color_buffer);
		}

		openni_wrapper::Image::Ptr ToRGB24OrigImageOni(const string& file_name)
		{
			string orig_file_name = file_name;
			orig_file_name.replace(orig_file_name.end() - 5, orig_file_name.end(), "orig");
			if (pclzf_mode)
				orig_file_name += ".pclzf_";
			else
				orig_file_name += ".png_";

			pcl::LZFRGB24ImageReaderExt rgb_reader;

			return rgb_reader.readOni(orig_file_name, orig_buffer);
		}

		openni_wrapper::DepthImage::Ptr ToDepthImageOni(const string& file_name)
		{
			string depth_file_name = file_name;
			if (pclzf_mode)
				depth_file_name += ".pclzf";
			else
				depth_file_name += ".png";

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(), ".xml");

			pcl::LZFDepth16ImageReaderExt depth_reader;
			io::CameraParameters params;

			depth_reader.readParameters(xml_file_name);
			params = depth_reader.getParameters();

			return depth_reader.readOni(depth_file_name, depth_buffer, params.focal_length_x);
		}
#endif

	};
}