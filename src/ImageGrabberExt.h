#pragma once
#include <pcl/io/image_grabber.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>

namespace pcl
{
	using namespace std;

	template <typename T> class PointCloud;
	template <typename PointT>
	class ImageGrabberExt : public ImageGrabber < PointT >
	{
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float reciprocalFocalLength);
		typedef void (Signal_PointCloud)(const boost::shared_ptr<const PointCloud<PointT> >&, bool fake);

	protected:
		string dir;
		vector<unsigned char> color_data;
		vector<unsigned short> depth_data;
		bool pclzf_mode;
		OniFrame oni_depth_frame, oni_color_frame;
		boost::shared_ptr<io::DepthImage> depth_image;
		boost::shared_ptr<io::Image> color_image;
		io::FrameWrapper::Ptr depth_frameWrapper, color_frameWrapper;

	protected:
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;
		boost::signals2::signal<Signal_PointCloud>* signal_PointCloud;

	public:
		ImageGrabberExt(const std::string& dir_,
			float frames_per_second = 0,
			bool repeat = false,
			bool pclzf_mode_ = false) : ImageGrabber(dir_, frames_per_second, repeat, pclzf_mode_), dir(dir_), pclzf_mode(pclzf_mode_), 
			signal_ImageDepth(nullptr), signal_PointCloud(nullptr)
		{
			boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_cloud =
				boost::bind(&ImageGrabberExt<PointT>::GetImage, this, _1);
			this->registerCallback(f_cloud);

			signal_ImageDepth = createSignal<Signal_ImageDepth>();
		}

		ImageGrabberExt(const std::string& depth_dir,
			const std::string& rgb_dir,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber(depth_dir, rgb_dir, frames_per_second, repeat), pclzf_mode(false),
			signal_ImageDepth(nullptr), signal_PointCloud(nullptr)
		{
		}

		ImageGrabberExt(const std::vector<std::string>& depth_image_files,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber(depth_image_files, frames_per_second, repeat), pclzf_mode(false),
			signal_ImageDepth(nullptr), signal_PointCloud(nullptr)
		{
		}

		~ImageGrabberExt()
		{
			disconnect_all_slots<Signal_ImageDepth>();
			disconnect_all_slots<Signal_PointCloud>();
		}

		void GetImage(const boost::shared_ptr<const PointCloud<PointT> >&)
		{
			if (signal_ImageDepth->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_ImageDepth->operator()(ToRGB24Image(file_name), ToDepthImage(file_name), 1.0);
			}
			if (signal_PointCloud->num_slots() > 0)
			{
				string file_name = dir + "\\" + this->getCurrentDepthFileName();
				signal_PointCloud->operator()(ToPointCloud(ToRGB24Image(file_name), ToDepthImage(file_name)), false);
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

			pcl::io::LZFRGB24ImageReader rgb;
			PointCloud<PointT> pcloud;

			rgb.read(color_file_name, pcloud);

			color_data.resize(pcloud.height*pcloud.width * 3);

			for (uint32_t i = 0; i < pcloud.width; i++)
			{
				for (uint32_t j = 0; j < pcloud.height; j++)
				{
					PointT p = pcloud.at(i, j);
					color_data[(i + j*pcloud.width) * 3 + 0] = p.r;
					color_data[(i + j*pcloud.width) * 3 + 1] = p.g;
					color_data[(i + j*pcloud.width) * 3 + 2] = p.b;
				}
			}

			oni_color_frame.data = (void*)&color_data[0];
			oni_color_frame.dataSize = color_data.size();
			oni_color_frame.height = pcloud.height;
			oni_color_frame.width = pcloud.width;
			oni_color_frame.stride = pcloud.width * 3;

			openni::VideoFrameRef frame;
			frame._setFrame(&oni_color_frame);
			color_frameWrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

			color_image =
				boost::make_shared<io::ImageRGB24>(color_frameWrapper);

			return color_image;
		}

		io::DepthImage::Ptr ToDepthImage(const string& file_name)
		{
			string depth_file_name = file_name;
			if (pclzf_mode)
				depth_file_name += ".pclzf";
			else
				depth_file_name += ".png";

			string xml_file_name = file_name;
			xml_file_name.replace(xml_file_name.end() - 6, xml_file_name.end(),".xml");

			pcl::io::LZFDepth16ImageReader depth;
			PointCloud<PointT> pcloud;

			depth.readParameters(xml_file_name);
			depth.read(depth_file_name, pcloud);

			depth_data.resize(pcloud.height*pcloud.width);

			for (uint32_t i = 0; i < pcloud.width; i++)
			{
				for (uint32_t j = 0; j < pcloud.height; j++)
				{
					PointT p = pcloud.at(i, j);
					depth_data[i + j*pcloud.width] = p.z*1000;
				}
			}

			oni_depth_frame.data = (void*)&depth_data[0];
			oni_depth_frame.dataSize = depth_data.size()*sizeof(unsigned short);
			oni_depth_frame.height = pcloud.height;
			oni_depth_frame.width = pcloud.width;
			oni_depth_frame.stride = pcloud.width*sizeof(unsigned short);

			openni::VideoFrameRef frame;
			frame._setFrame(&oni_depth_frame);
			depth_frameWrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

			float focalLength = 0;
			float baseline = 0;
			pcl::uint64_t no_sample_value = 0;
			pcl::uint64_t shadow_value = 0;

			depth_image =
				boost::make_shared<io::DepthImage>(depth_frameWrapper, baseline, focalLength, shadow_value, no_sample_value);

			return depth_image;
		}

		boost::shared_ptr<PointCloud<PointT> > ToPointCloud(const boost::shared_ptr<io::Image>& cimage, const boost::shared_ptr<io::DepthImage>& dimage)
		{
			//reproject depth image
			PointCloud<PointT>::Ptr point_cloud(new PointCloud<PointT>());

			point_cloud->width = dimage->getWidth();
			point_cloud->height = dimage->getHeight();
			point_cloud->is_dense = false;
			point_cloud->points.resize(point_cloud->height * point_cloud->width);
			//depth camera paramters
			float fl = dimage->getFocalLength();
			float cx = point_cloud->width / 2;
			float cy = point_cloud->height / 2;

			PointT* pt = &point_cloud->points[0];
			for (int y = 0; y < point_cloud->height; y++){
				for (int x = 0; x < point_cloud->width; x++, pt++){
					unsigned short depth = dimage->getData()[y * point_cloud->width + x];
					pt->z = depth * 0.001;
					pt->x = (x - cx)*pt->z / fl;
					pt->y = (y - cy)*pt->z / fl;
				}
			}

			return point_cloud;
		}

	};
}