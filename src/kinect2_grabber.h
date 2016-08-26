// Kinect2Grabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ImageUtils.h"

namespace pcl
{
	template<class Interface>
	inline void SafeRelease(Interface *& IRelease)
	{
		if (IRelease != NULL){
			IRelease->Release();
			IRelease = NULL;
		}
	}


	template<typename PointT, typename ImageT, typename DepthImageT>
	class Kinect2Grabber : public Grabber
	{
	public:
		typedef void (signal_Kinect2_PointT)(const boost::shared_ptr<const PointCloud<PointT>>&);
		typedef void (signal_Kinect2_ImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, float reciprocalFocalLength);
		typedef void (signal_Kinect2_ImageDepthImage)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&);
		typedef void (signal_Kinect2_ImageDepthImageDepth)(const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&, const boost::shared_ptr<ImageT>&, const boost::shared_ptr<DepthImageT>&);

		Kinect2Grabber()
			: sensor(nullptr)
			, mapper(nullptr)
			, colorSource(nullptr)
			, colorReader(nullptr)
			, depthSource(nullptr)
			, depthReader(nullptr)
			, result(S_OK)
			, colorWidth(1920)
			, colorHeight(1080)
			, colorBuffer()
			, depthWidth(512)
			, depthHeight(424)
			, depthBuffer()
			, running(false)
			, quit(false)
			, signal_PointT(nullptr)
			, data_ready(false)
			, color_timestamp(0)
			, depth_timestamp(0)
		{
			// Create Sensor Instance
			result = GetDefaultKinectSensor(&sensor);
			if (FAILED(result)){
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

			// Retrieved Color Frame Source
			result = sensor->get_ColorFrameSource(&colorSource);
			if (FAILED(result)){
				throw std::exception("Exception : IKinectSensor::get_ColorFrameSource()");
			}

			// Retrieved Depth Frame Source
			result = sensor->get_DepthFrameSource(&depthSource);
			if (FAILED(result)){
				throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
			}

			// Retrieved Color Frame Size
			IFrameDescription* colorDescription;
			result = colorSource->get_FrameDescription(&colorDescription);
			if (FAILED(result)){
				throw std::exception("Exception : IColorFrameSource::get_FrameDescription()");
			}

			result = colorDescription->get_Width(&colorWidth); // 1920
			if (FAILED(result)){
				throw std::exception("Exception : IFrameDescription::get_Width()");
			}

			result = colorDescription->get_Height(&colorHeight); // 1080
			if (FAILED(result)){
				throw std::exception("Exception : IFrameDescription::get_Height()");
			}

			SafeRelease(colorDescription);

			// To Reserve Color Frame Buffer
			colorBuffer.resize(colorWidth * colorHeight);

			// Retrieved Depth Frame Size
			IFrameDescription* depthDescription;
			result = depthSource->get_FrameDescription(&depthDescription);
			if (FAILED(result)){
				throw std::exception("Exception : IDepthFrameSource::get_FrameDescription()");
			}

			result = depthDescription->get_Width(&depthWidth); // 512
			if (FAILED(result)){
				throw std::exception("Exception : IFrameDescription::get_Width()");
			}

			result = depthDescription->get_Height(&depthHeight); // 424
			if (FAILED(result)){
				throw std::exception("Exception : IFrameDescription::get_Height()");
			}

			SafeRelease(depthDescription);

			// To Reserve Depth Frame Buffer
			depthBuffer.resize(depthWidth * depthHeight);

			signal_PointT = createSignal<signal_Kinect2_PointT>();
			signal_ImageDepth = createSignal<signal_Kinect2_ImageDepth>();
			signal_ImageDepthImage = createSignal<signal_Kinect2_ImageDepthImage>();
			signal_ImageDepthImageDepth = createSignal<signal_Kinect2_ImageDepthImageDepth>();
		}

		~Kinect2Grabber() throw()
		{
			stop();

			disconnect_all_slots<signal_Kinect2_PointT>();
			disconnect_all_slots<signal_Kinect2_ImageDepth>();
			disconnect_all_slots<signal_Kinect2_ImageDepthImage>();
			disconnect_all_slots<signal_Kinect2_ImageDepthImageDepth>();

			// End Processing
			if (sensor){
				sensor->Close();
			}
			SafeRelease(sensor);
			SafeRelease(mapper);
			SafeRelease(colorSource);
			SafeRelease(colorReader);
			SafeRelease(depthSource);
			SafeRelease(depthReader);

			thread.join();
		}

		bool IsAvailable() {
			BOOLEAN value;
			sensor->get_IsAvailable(&value);
			return (value == 1);
		}

		void start()
		{
			// Open Color Frame Reader
			result = colorSource->OpenReader(&colorReader);
			if (FAILED(result)){
				throw std::exception("Exception : IColorFrameSource::OpenReader()");
			}

			// Open Depth Frame Reader
			result = depthSource->OpenReader(&depthReader);
			if (FAILED(result)){
				throw std::exception("Exception : IDepthFrameSource::OpenReader()");
			}

			running = true;

			thread = boost::thread(&Kinect2Grabber::threadFunction, this);
		}

		void stop()
		{
			boost::unique_lock<boost::mutex> lock(mutex);

			quit = true;
			running = false;

			lock.unlock();
		}

		bool isRunning() const
		{
			boost::unique_lock<boost::mutex> lock(mutex);

			return running;

			lock.unlock();
		}

		std::string getName() const{
			return std::string("Kinect2Grabber");
		}

		float getFramesPerSecond() const {
			return 30.0f;
		}

		void threadFunction()
		{

			while (!quit){
				boost::unique_lock<boost::mutex> lock(mutex);

				// Acquire Latest Color Frame
				IColorFrame* colorFrame = nullptr;
				result = colorReader->AcquireLatestFrame(&colorFrame);
				if (SUCCEEDED(result)){
					// Retrieved Color Data
					result = colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);

					if (FAILED(result)){
						throw std::exception("Exception : IColorFrame::CopyConvertedFrameDataToArray()");
					}

					colorFrame->get_RelativeTime(&color_timestamp);
				}


				SafeRelease(colorFrame);

				// Acquire Latest Depth Frame
				IDepthFrame* depthFrame = nullptr;
				result = depthReader->AcquireLatestFrame(&depthFrame);
				if (SUCCEEDED(result)){
					// Retrieved Depth Data
					result = depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);

					if (FAILED(result)){
						throw std::exception("Exception : IDepthFrame::CopyFrameDataToArray()");
					}

					depthFrame->get_RelativeTime(&depth_timestamp);
					data_ready = true;
				}
				else
					data_ready = false;

				SafeRelease(depthFrame);

				lock.unlock();

				if (data_ready)//fire signals only if there is new depth data
				{
					mapping_updated = false;

					if (signal_PointT->num_slots() > 0) {
						signal_PointT->operator()(convertToPointCloud(&colorBuffer[0]));
					}
					
					if (signal_ImageDepth->num_slots() > 0)
						signal_ImageDepth->operator()(convertColorImageReg(colorBuffer), convertDepthImage(depthBuffer), 1.0);
					
					if (signal_ImageDepthImage->num_slots() > 0)
						signal_ImageDepthImage->operator()(convertColorImageReg(colorBuffer), convertDepthImage(depthBuffer), convertColorImageOrig(colorBuffer));

					if (signal_ImageDepthImageDepth->num_slots() > 0)
						signal_ImageDepthImageDepth->operator()(convertColorImageOrig(colorBuffer), convertDepthImage(depthBuffer), convertColorImageReg(colorBuffer), convertDepthImageReg(depthBuffer, depth_timestamp / 10));
				}
			}
		}

		void PrintCalib()
		{
			if (mapper)
			{
				CameraIntrinsics intrinsics;

				mapper->GetDepthCameraIntrinsics(&intrinsics);

				cerr << "FocalLengthX: " << intrinsics.FocalLengthX << endl;
				cerr << "FocalLengthY: " << intrinsics.FocalLengthY << endl;
				cerr << "PrincipalPointX: " << intrinsics.PrincipalPointX << endl;
				cerr << "PrincipalPointY: " << intrinsics.PrincipalPointY << endl;
				cerr << "RadialDistortionFourthOrder: " << intrinsics.RadialDistortionFourthOrder << endl;
				cerr << "RadialDistortionSecondOrder: " << intrinsics.RadialDistortionSecondOrder << endl;
				cerr << "RadialDistortionSixthOrder: " << intrinsics.RadialDistortionSixthOrder << endl;

			}
		}

		bool mapping_updated;

	protected:
		boost::signals2::signal<signal_Kinect2_PointT>* signal_PointT;
		boost::signals2::signal<signal_Kinect2_ImageDepthImage>* signal_ImageDepthImage;
		boost::signals2::signal<signal_Kinect2_ImageDepth>* signal_ImageDepth;
		boost::signals2::signal<signal_Kinect2_ImageDepthImageDepth>* signal_ImageDepthImageDepth;//original + registered images

		vector<unsigned char> color_buffer, orig_buffer;
		vector<unsigned short> depth_buffer;

		boost::thread thread;
		mutable boost::mutex mutex;

		bool quit;
		bool running;

		HRESULT result;
		IKinectSensor* sensor;
		ICoordinateMapper* mapper;
		IColorFrameSource* colorSource;
		IColorFrameReader* colorReader;
		IDepthFrameSource* depthSource;
		IDepthFrameReader* depthReader;

		int colorWidth;
		int colorHeight;
		std::vector<RGBQUAD> colorBuffer;
		TIMESPAN color_timestamp, depth_timestamp;

		int depthWidth;
		int depthHeight;
		std::vector<UINT16> depthBuffer;

		vector<DepthSpacePoint> depth_space_points;
		vector<CameraSpacePoint> camera_space_points;
		vector<ColorSpacePoint> color_space_points;
		bool data_ready;

		boost::shared_ptr<ImageT> convertColorImageOrig(const std::vector<RGBQUAD>& buffer)
		{
			//convert to rgb buffer
			orig_buffer.resize(colorHeight*colorWidth * 3);
			const RGBQUAD* cbuffer = &buffer[0];
			unsigned char* convbuffer = &orig_buffer[0];
			for (unsigned int i = 0; i < buffer.size(); i++, cbuffer++)
			{
				*convbuffer++ = cbuffer->rgbRed;
				*convbuffer++ = cbuffer->rgbGreen;
				*convbuffer++ = cbuffer->rgbBlue;
			}

			if (!color_timestamp)
				color_timestamp = depth_timestamp;

			return ToImageRGB24<ImageT>(&orig_buffer[0], colorWidth, colorHeight, color_timestamp / 10);
		}

		boost::shared_ptr<ImageT> convertColorImageReg(const std::vector<RGBQUAD>& buffer)
		{
			UpdateMapping(depthBuffer);

			int color_size = colorWidth*colorHeight;
			int depth_size = depthWidth*depthHeight;

			color_buffer.resize(depth_size * 3);

			unsigned char* pt = &color_buffer[0];
			ColorSpacePoint* colsp = &color_space_points[0];

			for (int i = 0; i < depth_size; i++, colsp++)
			{
				int color_ind = (int)(colsp->X + 0.5) + (int)(colsp->Y + 0.5)*colorWidth;

				if ((color_ind >= 0) && (color_ind < color_size))
				{
					RGBQUAD* cp = &colorBuffer[color_ind];
					*pt++ = cp->rgbRed;
					*pt++ = cp->rgbGreen;
					*pt++ = cp->rgbBlue;
				}
				else
				{
					*pt++ = 0;
					*pt++ = 0;
					*pt++ = 0;
				}
			}

			if (!color_timestamp)
				color_timestamp = depth_timestamp;

			return ToImageRGB24<ImageT>(&color_buffer[0], depthWidth, depthHeight, color_timestamp / 10);
		}

		boost::shared_ptr<DepthImageT> convertDepthImage(const std::vector<UINT16>& buffer)
		{
			CameraIntrinsics intrinsics;
			mapper->GetDepthCameraIntrinsics(&intrinsics);

			//parameters are not available during first couple of seconds
			if (!intrinsics.FocalLengthX)
				intrinsics.FocalLengthX = 364.82281494140625;

			return ToDepthImage<DepthImageT>(&buffer[0], depthWidth, depthHeight, intrinsics.FocalLengthX, depth_timestamp / 10);
		}

		boost::shared_ptr<const PointCloud<PointT> > convertToPointCloud(RGBQUAD* colorBuffer)
		{
			static long long pcloud_counter = 0;

			UpdateMapping(depthBuffer);

			int color_size = colorWidth*colorHeight;
			int cloud_size = depthWidth*depthHeight;

			PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>());

			cloud->width = static_cast<uint32_t>(depthWidth);
			cloud->height = static_cast<uint32_t>(depthHeight);
			cloud->is_dense = false;
			cloud->points.resize(cloud_size);
			cloud->header.stamp = depth_timestamp;
			cloud->header.seq = pcloud_counter++;

			PointT* pt = &cloud->points[0];
			CameraSpacePoint* csp = &camera_space_points[0];
			ColorSpacePoint* colsp = &color_space_points[0];

			for (int i = 0; i < cloud_size; i++, pt++, csp++, colsp++)
			{
				int color_ind = (int)(colsp->X + 0.5) + (int)(colsp->Y + 0.5)*colorWidth;

				if ((color_ind >= 0) && (color_ind < color_size))
				{
					RGBQUAD* cp = &colorBuffer[color_ind];
					pt->x = csp->X;
					pt->y = -csp->Y;//--X, Y makes it compatible with OpenNI point clouds and image readers
					pt->z = csp->Z;
					if ((typeid(PointT) == typeid(PointXYZRGB)) || (typeid(PointT) == typeid(PointXYZRGBA)))
					{
						pt->b = cp->rgbBlue;
						pt->g = cp->rgbGreen;
						pt->r = cp->rgbRed;
					}
					if (typeid(PointT) == typeid(PointXYZRGBA))
						pt->a = 255;
				}
				else
				{
					pt->x = pt->y = pt->z = pt->x = std::numeric_limits<float>::quiet_NaN();
				}
			}

			return cloud;
		}

		void UpdateMapping(const std::vector<unsigned short>& depth_buffer)
		{
			if (mapping_updated)
				return;

			int depth_size = depth_buffer.size();
			int color_size = colorWidth*colorHeight;

			if (camera_space_points.size() != depth_size)
				camera_space_points.resize(depth_size);

			if (color_space_points.size() != depth_size)
				color_space_points.resize(depth_size);

			if (depth_space_points.size() != color_size)
				depth_space_points.resize(color_size);

			mapper->MapDepthFrameToCameraSpace(depth_size, &depth_buffer[0], camera_space_points.size(), &camera_space_points[0]);
			mapper->MapDepthFrameToColorSpace(depth_size, &depth_buffer[0], color_space_points.size(), &color_space_points[0]);
			mapper->MapColorFrameToDepthSpace(depth_size, &depth_buffer[0], depth_space_points.size(), &depth_space_points[0]);

			mapping_updated = true;
		}

		public:
			boost::shared_ptr<DepthImageT> convertDepthImageReg(const std::vector<UINT16>& buffer, long long timestamp)
			{
				UpdateMapping(buffer);

				int color_size = colorWidth*colorHeight;
				int depth_size = depthWidth*depthHeight;

				depth_buffer.resize(color_size);

				unsigned short* pt = &depth_buffer[0];
				DepthSpacePoint* depthsp = &depth_space_points[0];

				for (int i = 0; i < color_size; i++, depthsp++)
				{
					int depth_ind = (int)(depthsp->X + 0.5) + (int)(depthsp->Y + 0.5)*depthWidth;

					if ((depth_ind >= 0) && (depth_ind < depth_size))
						*pt++ = buffer[depth_ind];
					else
						*pt++ = 0;
				}

				CameraIntrinsics intrinsics;
				mapper->GetDepthCameraIntrinsics(&intrinsics);

				//parameters are not available during first couple of seconds
				if (!intrinsics.FocalLengthX)
					intrinsics.FocalLengthX = 364.82281494140625;

				return ToDepthImage<DepthImageT>(&depth_buffer[0], colorWidth, colorHeight, intrinsics.FocalLengthX, timestamp);
			}

	};

}

#endif KINECT2_GRABBER

