#pragma once
#include <pcl/io/image_grabber.h>
#include "ImageUtils.h"

#ifdef HAVE_OPENNI2
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#endif

namespace pcl
{
	using namespace std;

	template <typename T> class PointCloud;
	template <typename PointT>
	class ImageGrabberExt : public ImageGrabber < PointT >
	{
#ifdef HAVE_OPENNI2
		typedef void (Signal_PointCloud)(const boost::shared_ptr<const PointCloud<PointT> >&, bool fake);
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float reciprocalFocalLength);
		typedef void (Signal_ImageDepthImage)(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, const boost::shared_ptr<io::Image>&);
#endif

	protected:
		string dir;
		bool pclzf_mode;

	protected:
#ifdef HAVE_OPENNI2
		boost::signals2::signal<Signal_PointCloud>* signal_PointCloud;
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;
		boost::signals2::signal<Signal_ImageDepthImage>* signal_ImageDepthImage;
#endif

	public:
		ImageGrabberExt(const std::string& dir_,
			float frames_per_second = 0,
			bool repeat = false,
			bool pclzf_mode_ = false) : ImageGrabber<PointT>(dir_, frames_per_second, repeat, pclzf_mode_), dir(dir_), pclzf_mode(pclzf_mode_)
#ifdef HAVE_OPENNI2
			, signal_ImageDepth(nullptr), signal_PointCloud(nullptr), signal_ImageDepthImage(nullptr)
#endif
		{
#ifdef HAVE_OPENNI2
			boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_cloud =
				boost::bind(&ImageGrabberExt<PointT>::GetImage, this, _1);
			this->registerCallback(f_cloud);

			signal_ImageDepth = createSignal<Signal_ImageDepth>();
			//signal_ImageDepthImage = createSignal<Signal_ImageDepthImage>();
			signal_PointCloud = createSignal<Signal_PointCloud>();
#endif
		}

		ImageGrabberExt(const std::string& depth_dir,
			const std::string& rgb_dir,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber<PointT>(depth_dir, rgb_dir, frames_per_second, repeat), pclzf_mode(false)
#ifdef HAVE_OPENNI2
			, signal_ImageDepth(nullptr), signal_PointCloud(nullptr), signal_ImageDepthImage(nullptr)
#endif
		{
		}

		ImageGrabberExt(const std::vector<std::string>& depth_image_files,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber<PointT>(depth_image_files, frames_per_second, repeat), pclzf_mode(false)
#ifdef HAVE_OPENNI2
			, signal_ImageDepth(nullptr), signal_PointCloud(nullptr), signal_ImageDepthImage(nullptr)
#endif
		{
		}

		~ImageGrabberExt() throw()
		{
#ifdef HAVE_OPENNI2
			disconnect_all_slots<Signal_ImageDepth>();
			disconnect_all_slots<Signal_ImageDepthImage>();
			disconnect_all_slots<Signal_PointCloud>();
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
			if (signal_ImageDepthImage && signal_ImageDepthImage->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_ImageDepthImage->operator()(ToRGB24Image(file_name), ToDepthImage(file_name), ToRGB24OrigImage(file_name));
			}
			if (signal_PointCloud->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_PointCloud->operator()(ToPointCloud(ToRGB24Image(file_name), ToDepthImage(file_name), RGBCameraParameters(file_name), DepthCameraParameters(file_name)), false);
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

			io::Image::Ptr image;

			rgb_reader.read(color_file_name, image);

			return image;
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

			io::Image::Ptr image;

			rgb_reader.read(orig_file_name, image);

			return image;
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

			io::DepthImage::Ptr image;

			depth_reader.read(depth_file_name, image);

			return image;
		}

		boost::shared_ptr<io::CameraParameters> RGBCameraParameters(const string& file_name)
		{
			io::CameraParameters params;

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(), ".xml");

			io::LZFRGB24ImageReader rgb_reader;
			rgb_reader.readParameters(xml_file_name);
			params = rgb_reader.getParameters();
			return boost::make_shared<io::CameraParameters>(params);
		}

		boost::shared_ptr<io::CameraParameters> DepthCameraParameters(const string& file_name)
		{
			io::CameraParameters params;

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(), ".xml");

			io::LZFDepth16ImageReader depth_reader;
			depth_reader.readParameters(xml_file_name);
			params = depth_reader.getParameters();
			return boost::make_shared<io::CameraParameters>(params);
		}

		boost::shared_ptr<PointCloud<PointT> > ToPointCloud(const boost::shared_ptr<io::Image>& cimage, const boost::shared_ptr<io::DepthImage>& dimage,
			const boost::shared_ptr<io::CameraParameters>& cparams, const boost::shared_ptr<io::CameraParameters>& dparams)
		{
			//reproject depth image
			PointCloud<PointT>::Ptr point_cloud(new PointCloud<PointT>());

			point_cloud->width = dimage->getWidth();
			point_cloud->height = dimage->getHeight();
			point_cloud->is_dense = false;
			point_cloud->points.resize(point_cloud->height * point_cloud->width);

			PointT* pt = &point_cloud->points[0];
			for (int y = 0; y < point_cloud->height; y++){
				for (int x = 0; x < point_cloud->width; x++, pt++){
					unsigned short depth = dimage->getData()[y * point_cloud->width + x];
					pt->z = depth * 0.001;
					pt->x = (x - dparams->principal_point_x)*pt->z / dparams->focal_length_x;
					pt->y = (y - dparams->principal_point_y)*pt->z / dparams->focal_length_y;
					int cx = (int)(cparams->focal_length_x*(pt->x + 0.052) / pt->z + cparams->principal_point_x + 0.5);
					int cy = (int)(cparams->focal_length_y*(pt->y - 0.020) / pt->z + cparams->principal_point_y + 0.5);
					if ((cx >= 0) && (cx < cimage->getWidth()) && (cy >= 0) && (cy < cimage->getHeight()))
					{
						unsigned char* color_ptr = &((unsigned char*)cimage->getData())[(cy * cimage->getWidth() + cx) * 3];
						pt->r = *color_ptr++;
						pt->g = *color_ptr++;
						pt->b = *color_ptr++;
						pt->a = 0;
					}
				}
			}

			return point_cloud;
		}
#endif

	};
}