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
	struct PointXYZ;
	struct PointXYZRGB;
	struct PointXYZRGBA;
	template <typename T> class PointCloud;

	template<class Interface>
	inline void SafeRelease( Interface *& IRelease )
	{
		if( IRelease != NULL ){
			IRelease->Release();
			IRelease = NULL;
		}
	}

	class Kinect2Grabber : public Grabber
	{
		public:
			Kinect2Grabber();
			virtual ~Kinect2Grabber() throw ();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;

			typedef void (signal_Kinect2_PointXYZ)( const boost::shared_ptr<const PointCloud<PointXYZ>>& );
			typedef void (signal_Kinect2_PointXYZRGB)(const boost::shared_ptr<const PointCloud<PointXYZRGB>>&);
			typedef void (signal_Kinect2_PointXYZRGBA)(const boost::shared_ptr<const PointCloud<PointXYZRGBA>>&);
#ifdef HAVE_OPENNI2
			typedef void (signal_Kinect2_ImageDepth)(const io::Image::Ptr&, const io::DepthImage::Ptr&, float reciprocalFocalLength);
			typedef void (signal_Kinect2_ImageDepthImage)(const io::Image::Ptr&, const io::DepthImage::Ptr&, const io::Image::Ptr&);
#endif
#ifdef HAVE_OPENNI
			typedef void (signal_Kinect2_ImageDepthOni)(const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, float reciprocalFocalLength);
			typedef void (signal_Kinect2_ImageDepthImageOni)(const openni_wrapper::Image::Ptr&, const openni_wrapper::DepthImage::Ptr&, const openni_wrapper::Image::Ptr&);
#endif

			void PrintCalib();
			void UpdateMapping();

	protected:
			boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;
			boost::signals2::signal<signal_Kinect2_PointXYZRGBA>* signal_PointXYZRGBA;

#ifdef HAVE_OPENNI2
			boost::signals2::signal<signal_Kinect2_ImageDepth>* signal_ImageDepth;
			boost::signals2::signal<signal_Kinect2_ImageDepthImage>* signal_ImageDepthImage;
			io::Image::Ptr convertColorImage(const std::vector<RGBQUAD>& colorBuffer);
			io::Image::Ptr convertColorImageOrig(const std::vector<RGBQUAD>& colorBuffer);
			io::DepthImage::Ptr convertDepthImage(const std::vector<UINT16>& depthBuffer);
			vector<unsigned char> color_buffer, orig_buffer;
			vector<unsigned short> depth_buffer;
#endif
#ifdef HAVE_OPENNI
			boost::signals2::signal<signal_Kinect2_ImageDepthOni>* signal_ImageDepthOni;
			boost::signals2::signal<signal_Kinect2_ImageDepthImageOni>* signal_ImageDepthImageOni;
			openni_wrapper::Image::Ptr convertColorImageOni(const std::vector<RGBQUAD>& colorBuffer);
			openni_wrapper::Image::Ptr convertColorImageOrigOni(const std::vector<RGBQUAD>& colorBuffer);
			openni_wrapper::DepthImage::Ptr convertDepthImageOni(const std::vector<UINT16>& depthBuffer);
#endif

			PointCloud<PointXYZ>::Ptr convertDepthToPointXYZ( UINT16* depthBuffer );
			PointCloud<PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB(RGBQUAD* colorBuffer, UINT16* depthBuffer);
			PointCloud<PointXYZRGBA>::Ptr convertRGBDepthToPointXYZRGBA(RGBQUAD* colorBuffer, UINT16* depthBuffer);

			boost::thread thread;
			mutable boost::mutex mutex;

			void threadFunction();

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

			int depthWidth;
			int depthHeight;
			std::vector<UINT16> depthBuffer;

			vector<DepthSpacePoint> depth_space_points;
			vector<CameraSpacePoint> camera_space_points;
			vector<ColorSpacePoint> color_space_points;
			bool data_ready;
			bool mapping_updated;
	};

	Kinect2Grabber::Kinect2Grabber()
		: sensor( nullptr )
		, mapper( nullptr )
		, colorSource( nullptr )
		, colorReader( nullptr )
		, depthSource( nullptr )
		, depthReader( nullptr )
		, result( S_OK )
		, colorWidth( 1920 )
		, colorHeight( 1080 )
		, colorBuffer()
		, depthWidth( 512 )
		, depthHeight( 424 )
		, depthBuffer()
		, running( false )
		, quit( false )
		, signal_PointXYZ( nullptr )
		, signal_PointXYZRGB( nullptr )
		, signal_PointXYZRGBA(nullptr)
		, data_ready(false)
	{
		// Create Sensor Instance
		result = GetDefaultKinectSensor( &sensor );
		if( FAILED( result ) ){
			throw std::exception( "Exception : GetDefaultKinectSensor()" );
		}

		// Open Sensor
		result = sensor->Open();
		if( FAILED( result ) ){
			throw std::exception( "Exception : IKinectSensor::Open()" );
		}

		// Retrieved Coordinate Mapper
		result = sensor->get_CoordinateMapper( &mapper );
		if( FAILED( result ) ){
			throw std::exception( "Exception : IKinectSensor::get_CoordinateMapper()" );
		}

		// Retrieved Color Frame Source
		result = sensor->get_ColorFrameSource( &colorSource );
		if( FAILED( result ) ){
			throw std::exception( "Exception : IKinectSensor::get_ColorFrameSource()" );
		}

		// Retrieved Depth Frame Source
		result = sensor->get_DepthFrameSource( &depthSource );
		if( FAILED( result ) ){
			throw std::exception( "Exception : IKinectSensor::get_DepthFrameSource()" );
		}

		// Retrieved Color Frame Size
		IFrameDescription* colorDescription;
		result = colorSource->get_FrameDescription( &colorDescription );
		if( FAILED( result ) ){
			throw std::exception( "Exception : IColorFrameSource::get_FrameDescription()" );
		}
		
		result = colorDescription->get_Width( &colorWidth ); // 1920
		if( FAILED( result ) ){
			throw std::exception( "Exception : IFrameDescription::get_Width()" );
		}

		result = colorDescription->get_Height( &colorHeight ); // 1080
		if( FAILED( result ) ){
			throw std::exception( "Exception : IFrameDescription::get_Height()" );
		}
		
		SafeRelease( colorDescription );

		// To Reserve Color Frame Buffer
		colorBuffer.resize( colorWidth * colorHeight );

		// Retrieved Depth Frame Size
		IFrameDescription* depthDescription;
		result = depthSource->get_FrameDescription( &depthDescription );
		if( FAILED( result ) ){
			throw std::exception( "Exception : IDepthFrameSource::get_FrameDescription()" );
		}

		result = depthDescription->get_Width( &depthWidth ); // 512
		if( FAILED( result ) ){
			throw std::exception( "Exception : IFrameDescription::get_Width()" );
		}

		result = depthDescription->get_Height( &depthHeight ); // 424
		if( FAILED( result ) ){
			throw std::exception( "Exception : IFrameDescription::get_Height()" );
		}

		SafeRelease( depthDescription );

		// To Reserve Depth Frame Buffer
		depthBuffer.resize( depthWidth * depthHeight );

		signal_PointXYZ = createSignal<signal_Kinect2_PointXYZ>();
		signal_PointXYZRGB = createSignal<signal_Kinect2_PointXYZRGB>();
		signal_PointXYZRGBA = createSignal<signal_Kinect2_PointXYZRGBA>();
#ifdef HAVE_OPENNI2
		signal_ImageDepth = createSignal<signal_Kinect2_ImageDepth>();
		signal_ImageDepthImage = createSignal<signal_Kinect2_ImageDepthImage>();
#endif
#ifdef HAVE_OPENNI
		signal_ImageDepthOni = createSignal<signal_Kinect2_ImageDepthOni>();
		signal_ImageDepthImageOni = createSignal<signal_Kinect2_ImageDepthImageOni>();
#endif
	}

	Kinect2Grabber::~Kinect2Grabber() throw( )
	{
		stop();

		disconnect_all_slots<signal_Kinect2_PointXYZ>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGB>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGBA>();
#ifdef HAVE_OPENNI2
		disconnect_all_slots<signal_Kinect2_ImageDepth>();
		disconnect_all_slots<signal_Kinect2_ImageDepthImage>();
#endif
#ifdef HAVE_OPENNI
		disconnect_all_slots<signal_Kinect2_ImageDepthOni>();
		disconnect_all_slots<signal_Kinect2_ImageDepthImageOni>();
#endif

		// End Processing
		if( sensor ){
			sensor->Close();
		}
		SafeRelease( sensor );
		SafeRelease( mapper );
		SafeRelease( colorSource );
		SafeRelease( colorReader );
		SafeRelease( depthSource );
		SafeRelease( depthReader );

		thread.join();
	}

	void Kinect2Grabber::start()
	{
		// Open Color Frame Reader
		result = colorSource->OpenReader( &colorReader );
		if( FAILED( result ) ){
			throw std::exception( "Exception : IColorFrameSource::OpenReader()" );
		}

		// Open Depth Frame Reader
		result = depthSource->OpenReader( &depthReader );
		if( FAILED( result ) ){
			throw std::exception( "Exception : IDepthFrameSource::OpenReader()" );
		}

		running = true;

		thread = boost::thread( &Kinect2Grabber::threadFunction, this );
	}

	void Kinect2Grabber::stop()
	{
		boost::unique_lock<boost::mutex> lock( mutex );
		
		quit = true;
		running = false;

		lock.unlock();
	}

	bool Kinect2Grabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock( mutex );

		return running;

		lock.unlock();
	}

	std::string Kinect2Grabber::getName() const{
		return std::string( "Kinect2Grabber" );
	}

	float Kinect2Grabber::getFramesPerSecond() const {
		return 30.0f;
	}

	void Kinect2Grabber::threadFunction()
	{

		while( !quit ){
			boost::unique_lock<boost::mutex> lock( mutex );

			// Acquire Latest Color Frame
			IColorFrame* colorFrame = nullptr;
			result = colorReader->AcquireLatestFrame( &colorFrame );
			if( SUCCEEDED( result ) ){
				// Retrieved Color Data
				result = colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
				if (FAILED(result)){
					throw std::exception( "Exception : IColorFrame::CopyConvertedFrameDataToArray()" );
				}
			}


			SafeRelease(colorFrame);

			// Acquire Latest Depth Frame
			IDepthFrame* depthFrame = nullptr;
			result = depthReader->AcquireLatestFrame( &depthFrame );
			if (SUCCEEDED(result)){
				// Retrieved Depth Data
				result = depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
				if (FAILED(result)){
					throw std::exception("Exception : IDepthFrame::CopyFrameDataToArray()");
				}
				data_ready = true;
			}
			else
				data_ready = false;

			SafeRelease( depthFrame );

			lock.unlock();

			if (data_ready)//fire signals only if there is new depth data
			{			
				mapping_updated = false;

				if (signal_PointXYZ->num_slots() > 0) {
					signal_PointXYZ->operator()( convertDepthToPointXYZ( &depthBuffer[0] ) );
				}

				if( signal_PointXYZRGB->num_slots() > 0 ) {
					signal_PointXYZRGB->operator()( convertRGBDepthToPointXYZRGB( &colorBuffer[0], &depthBuffer[0] ) );
				}

				if (signal_PointXYZRGBA->num_slots() > 0) {
					signal_PointXYZRGBA->operator()(convertRGBDepthToPointXYZRGBA(&colorBuffer[0], &depthBuffer[0]));
				}
#ifdef HAVE_OPENNI2
				if (signal_ImageDepth->num_slots() > 0)
					signal_ImageDepth->operator()(convertColorImage(colorBuffer), convertDepthImage(depthBuffer), 1.0);
				if (signal_ImageDepthImage->num_slots() > 0)
					signal_ImageDepthImage->operator()(convertColorImage(colorBuffer), convertDepthImage(depthBuffer), convertColorImageOrig(colorBuffer));
#endif
#ifdef HAVE_OPENNI
				if (signal_ImageDepthOni->num_slots() > 0)
					signal_ImageDepthOni->operator()(convertColorImageOni(colorBuffer), convertDepthImageOni(depthBuffer), 1.0);
				if (signal_ImageDepthImageOni->num_slots() > 0)
					signal_ImageDepthImageOni->operator()(convertColorImageOni(colorBuffer), convertDepthImageOni(depthBuffer), convertColorImageOrigOni(colorBuffer));
#endif
			}
		}
	}

#ifdef HAVE_OPENNI2
	void Kinect2Grabber::UpdateMapping()
	{
		if (mapping_updated)
			return;

		int depth_size = depthWidth*depthHeight;

		if (camera_space_points.size() != depth_size)
			camera_space_points.resize(depth_size);

		if (color_space_points.size() != depth_size)
			color_space_points.resize(depth_size);

		mapper->MapDepthFrameToCameraSpace(depth_size, &depthBuffer[0], camera_space_points.size(), &camera_space_points[0]);
		mapper->MapDepthFrameToColorSpace(depth_size, &depthBuffer[0], color_space_points.size(), &color_space_points[0]);

		mapping_updated = true;
	}

	io::Image::Ptr Kinect2Grabber::convertColorImageOrig(const std::vector<RGBQUAD>& buffer)
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

		return ToImageRGB24<io::Image>(&orig_buffer[0], colorWidth, colorHeight);
	}

	io::Image::Ptr Kinect2Grabber::convertColorImage(const std::vector<RGBQUAD>& buffer)
	{
		UpdateMapping();

		int color_size = colorWidth*colorHeight;
		int depth_size = depthWidth*depthHeight;

		color_buffer.resize(depth_size * 3, 0);

		unsigned char* pt = &color_buffer[0];
		CameraSpacePoint* csp = &camera_space_points[0];
		ColorSpacePoint* colsp = &color_space_points[0];

		for (int i = 0; i < depth_size; i++, csp++, colsp++)
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
				pt += 3;
		}

		return ToImageRGB24<io::Image>(&color_buffer[0], depthWidth, depthHeight);
	}

	io::DepthImage::Ptr Kinect2Grabber::convertDepthImage(const std::vector<UINT16>& buffer)
	{
		CameraIntrinsics intrinsics;
		mapper->GetDepthCameraIntrinsics(&intrinsics);

		//parameters are not available during first couple of seconds
		if (!intrinsics.FocalLengthX)
			intrinsics.FocalLengthX = 364.82281494140625;

		return ToDepthImage<io::DepthImage>(&buffer[0], depthWidth, depthHeight, intrinsics.FocalLengthX);
	}
#endif

#ifdef HAVE_OPENNI
	openni_wrapper::Image::Ptr Kinect2Grabber::convertColorImageOrigOni(const std::vector<RGBQUAD>& buffer)
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

		return ToImageRGB24<openni_wrapper::Image>(&orig_buffer[0], colorWidth, colorHeight);
	}

	openni_wrapper::Image::Ptr Kinect2Grabber::convertColorImageOni(const std::vector<RGBQUAD>& buffer)
	{
		//convert to rgb buffer
		color_buffer.resize(colorHeight*colorWidth * 3);
		const RGBQUAD* cbuffer = &buffer[0];
		unsigned char* convbuffer = &color_buffer[0];
		for (unsigned int i = 0; i < buffer.size(); i++, cbuffer++)
		{
			*convbuffer++ = cbuffer->rgbRed;
			*convbuffer++ = cbuffer->rgbGreen;
			*convbuffer++ = cbuffer->rgbBlue;
		}

		return ToImageRGB24<openni_wrapper::Image>(&color_buffer[0], colorWidth, colorHeight);
	}

	openni_wrapper::DepthImage::Ptr Kinect2Grabber::convertDepthImageOni(const std::vector<UINT16>& buffer)
	{
		CameraIntrinsics intrinsics;
		mapper->GetDepthCameraIntrinsics(&intrinsics);

		//parameters are not available during first couple of seconds
		if (!intrinsics.FocalLengthX)
			intrinsics.FocalLengthX = 364.82281494140625;

		return ToDepthImage<openni_wrapper::DepthImage>(&buffer[0], depthWidth, depthHeight, intrinsics.FocalLengthX);
	}
#endif

	PointCloud<PointXYZ>::Ptr Kinect2Grabber::convertDepthToPointXYZ(UINT16* depthBuffer)
	{
		PointCloud<PointXYZ>::Ptr cloud( new PointCloud<PointXYZ>() );

		cloud->width = static_cast<uint32_t>( depthWidth );
		cloud->height = static_cast<uint32_t>( depthHeight );
		cloud->is_dense = false;

		cloud->points.resize( cloud->height * cloud->width );

		PointXYZ* pt = &cloud->points[0];
		for( int y = 0; y < depthHeight; y++ ){
			for( int x = 0; x < depthWidth; x++, pt++ ){
				PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
				point.x = cameraSpacePoint.X;
				point.y = cameraSpacePoint.Y;
				point.z = cameraSpacePoint.Z;

				*pt = point;
			}
		}

		return cloud;
	}

	PointCloud<PointXYZRGB>::Ptr Kinect2Grabber::convertRGBDepthToPointXYZRGB( RGBQUAD* colorBuffer, UINT16* depthBuffer )
	{
		PointCloud<PointXYZRGB>::Ptr cloud( new PointCloud<PointXYZRGB>() );

		cloud->width = static_cast<uint32_t>( depthWidth );
		cloud->height = static_cast<uint32_t>( depthHeight );
		cloud->is_dense = false;

		cloud->points.resize( cloud->height * cloud->width );

		PointXYZRGB* pt = &cloud->points[0];
		for( int y = 0; y < depthHeight; y++ ){
			for( int x = 0; x < depthWidth; x++, pt++ ){
				PointXYZRGB point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				mapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );
				int colorX = static_cast<int>( std::floor( colorSpacePoint.X + 0.5f ) );
				int colorY = static_cast<int>( std::floor( colorSpacePoint.Y + 0.5f ) );
				if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
					RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
					point.b = color.rgbBlue;
					point.g = color.rgbGreen;
					point.r = color.rgbRed;
				}

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
				if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				}

				*pt = point;
			}
		}

		return cloud;
	}

	PointCloud<PointXYZRGBA>::Ptr Kinect2Grabber::convertRGBDepthToPointXYZRGBA(RGBQUAD* colorBuffer, UINT16* depthBuffer)
	{
		UpdateMapping();

		int color_size = colorWidth*colorHeight;
		int cloud_size = depthWidth*depthHeight;

		PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;
		cloud->points.resize(cloud_size);

		PointXYZRGBA* pt = &cloud->points[0];
		CameraSpacePoint* csp = &camera_space_points[0];
		ColorSpacePoint* colsp = &color_space_points[0];

		for (int i = 0; i < cloud_size; i++, pt++, csp++, colsp++)
		{
			int color_ind = (int)(colsp->X + 0.5) + (int)(colsp->Y + 0.5)*colorWidth;
			
			if ((color_ind >= 0) && (color_ind < color_size))
			{
				RGBQUAD* cp = &colorBuffer[color_ind];
				pt->x = csp->X;
				pt->y = csp->Y;
				pt->z = csp->Z;
				pt->b = cp->rgbBlue;
				pt->g = cp->rgbGreen;
				pt->r = cp->rgbRed;
			}		
		}

		return cloud;
	}

	void Kinect2Grabber::PrintCalib()
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
}

#endif KINECT2_GRABBER

