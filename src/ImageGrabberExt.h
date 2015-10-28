#pragma once
#include <pcl/io/image_grabber.h>
#include "ImageReaderExt.h"

#ifdef HAVE_KINECT2_NATIVE
#include "kinect2_grabber.h"
#endif

namespace pcl
{
	enum RegistrationMode {
		R_NONE,
		R_DEPTH2RGB,
		R_RGB2DEPTH
	};

	using namespace std;

	template <typename PointT, typename ImageT, typename DepthImageT>
	class ImageGrabberExt : public ImageGrabber < PointT >
	{
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float focal_length);
		typedef void (Signal_ImageDepthImage)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&);

	protected:
		string dir;
		bool pclzf_mode;
		vector<unsigned char> color_buffer, orig_buffer;
		vector<unsigned short> depth_buffer;
		int file_index;
		RegistrationMode registration_mode;

	protected:
		using Grabber::createSignal;
		using Grabber::disconnect_all_slots;
		
		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;
		boost::signals2::signal<Signal_ImageDepthImage>* signal_ImageDepthImage;

	public:
		ImageGrabberExt(const std::string& dir_,
			float frames_per_second = 0,
			bool repeat = false,
			bool pclzf_mode_ = false) : ImageGrabber<PointT>(dir_, frames_per_second, repeat, pclzf_mode_), dir(dir_), pclzf_mode(pclzf_mode_), file_index(0)
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr), registration_mode(R_NONE)
		{
			boost::function<void(const boost::shared_ptr<const PointCloud<PointT> >&)> f_cloud =
				boost::bind(&ImageGrabberExt<PointT, ImageT, DepthImageT>::GetImage, this, _1);
			this->registerCallback(f_cloud);

			signal_ImageDepth = createSignal<Signal_ImageDepth>();
			signal_ImageDepthImage = createSignal<Signal_ImageDepthImage>();
		}

		ImageGrabberExt(const std::string& depth_dir,
			const std::string& rgb_dir,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber<PointT>(depth_dir, rgb_dir, frames_per_second, repeat), pclzf_mode(false), file_index(0)
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr), registration_mode(R_NONE)
		{
		}

		ImageGrabberExt(const std::vector<std::string>& depth_image_files,
			float frames_per_second = 0,
			bool repeat = false) : ImageGrabber<PointT>(depth_image_files, frames_per_second, repeat), pclzf_mode(false), file_index(0)
			, signal_ImageDepth(nullptr), signal_ImageDepthImage(nullptr), registration_mode(R_NONE)
		{
		}
		/*
		void SetRegistrationMode(RegistrationMode mode) 
		{
			registration_mode = mode;

			IKinectSensor* sensor;
			ICoordinateMapper* mapper;
			HRESULT result;
			vector<ColorSpacePoint> color_space_points;

			//this is currently only possible with Kinect attached to the PC
			// Create Sensor Instance
			result = GetDefaultKinectSensor(&sensor);
			if (FAILED(result)) {
				throw std::exception("Exception : GetDefaultKinectSensor()");
			}

			// Open Sensor
			result = sensor->Open();
			if (FAILED(result)){
				throw std::exception("Exception : IKinectSensor::Open()");
			}

			// Retrieved Coordinate Mapper
			result = sensor->get_CoordinateMapper(&mapper);
			if (FAILED(result)){
				throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
			}

			if (color_space_points.size() != depth_buffer.size())
				color_space_points.resize(depth_buffer.size());

//			mapper->MapDepthFrameToColorSpace(depth_size, &depthBuffer[0], color_space_points.size(), &color_space_points[0]);

		}
		*/

		virtual ~ImageGrabberExt() throw()
		{
			disconnect_all_slots<Signal_ImageDepth>();
			disconnect_all_slots<Signal_ImageDepthImage>();
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

			return rgb_reader.read(color_file_name, color_buffer);
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

			return rgb_reader.read(orig_file_name, orig_buffer);
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
	};
}