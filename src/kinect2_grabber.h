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
#include <pcl/io/openni2/openni2_metadata_wrapper.h>

namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	struct pcl::PointXYZRGBA;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease( Interface *& IRelease )
	{
		if( IRelease != NULL ){
			IRelease->Release();
			IRelease = NULL;
		}
	}

	class Kinect2Grabber : public pcl::Grabber
	{
		public:
			Kinect2Grabber();
			virtual ~Kinect2Grabber() throw ();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;

			typedef void (signal_Kinect2_PointXYZ)( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
			typedef void (signal_Kinect2_PointXYZRGB)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
			typedef void (signal_Kinect2_PointXYZRGBA)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&);
			typedef void (signal_Kinect2_ImageDepth)(const boost::shared_ptr<io::Image>&, const boost::shared_ptr<io::DepthImage>&, float reciprocalFocalLength);

			void PrintCalib();

	protected:
			boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
			boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;
			boost::signals2::signal<signal_Kinect2_PointXYZRGBA>* signal_PointXYZRGBA;
			boost::signals2::signal<signal_Kinect2_ImageDepth>* signal_ImageDepth;

			pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ( UINT16* depthBuffer );
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB(RGBQUAD* colorBuffer, UINT16* depthBuffer);
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBDepthToPointXYZRGBA(RGBQUAD* colorBuffer, UINT16* depthBuffer);
			io::Image::Ptr convertColorImage(const std::vector<RGBQUAD>& colorBuffer);
			io::DepthImage::Ptr convertDepthImage(const std::vector<UINT16>& depthBuffer);

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

			OniFrame oni_depth_frame, oni_color_frame;
			boost::shared_ptr<io::DepthImage> depth_image;
			boost::shared_ptr<io::Image> color_image;
			io::FrameWrapper::Ptr depth_frameWrapper, color_frameWrapper;
			std::vector<unsigned char> converted_buffer;
			vector<DepthSpacePoint> depth_space_points;
			vector<CameraSpacePoint> camera_space_points;
			vector<ColorSpacePoint> color_space_points;
	};

	pcl::Kinect2Grabber::Kinect2Grabber()
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
		, signal_ImageDepth(nullptr)
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
		signal_ImageDepth = createSignal<signal_Kinect2_ImageDepth>();
	}

	pcl::Kinect2Grabber::~Kinect2Grabber() throw( )
	{
		stop();

		disconnect_all_slots<signal_Kinect2_PointXYZ>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGB>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGBA>();
		disconnect_all_slots<signal_Kinect2_ImageDepth>();

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

	void pcl::Kinect2Grabber::start()
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

	void pcl::Kinect2Grabber::stop()
	{
		boost::unique_lock<boost::mutex> lock( mutex );
		
		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::Kinect2Grabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock( mutex );

		return running;

		lock.unlock();
	}

	std::string pcl::Kinect2Grabber::getName() const{
		return std::string( "Kinect2Grabber" );
	}

	float pcl::Kinect2Grabber::getFramesPerSecond() const {
		return 30.0f;
	}

	void pcl::Kinect2Grabber::threadFunction()
	{
		while( !quit ){
			boost::unique_lock<boost::mutex> lock( mutex );

			// Acquire Latest Color Frame
			IColorFrame* colorFrame = nullptr;
			result = colorReader->AcquireLatestFrame( &colorFrame );
			if( SUCCEEDED( result ) ){
				// Retrieved Color Data
				result = colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
				//default format: YUY2
				if (FAILED(result)){
					throw std::exception( "Exception : IColorFrame::CopyConvertedFrameDataToArray()" );
				}
			}

			SafeRelease(colorFrame);

			// Acquire Latest Depth Frame
			IDepthFrame* depthFrame = nullptr;
			result = depthReader->AcquireLatestFrame( &depthFrame );
			if( SUCCEEDED( result ) ){
				// Retrieved Depth Data
				result = depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] );
				if( FAILED( result ) ){
					throw std::exception( "Exception : IDepthFrame::CopyFrameDataToArray()" );
				}
			}
			SafeRelease( depthFrame );

			lock.unlock();

			if( signal_PointXYZ->num_slots() > 0 ) {
				signal_PointXYZ->operator()( convertDepthToPointXYZ( &depthBuffer[0] ) );
			}

			if( signal_PointXYZRGB->num_slots() > 0 ) {
				signal_PointXYZRGB->operator()( convertRGBDepthToPointXYZRGB( &colorBuffer[0], &depthBuffer[0] ) );
			}

			if (signal_PointXYZRGBA->num_slots() > 0) {
				signal_PointXYZRGBA->operator()(convertRGBDepthToPointXYZRGBA(&colorBuffer[0], &depthBuffer[0]));
			}

			if (signal_ImageDepth->num_slots() > 0)
			{
				signal_ImageDepth->operator()(convertColorImage(colorBuffer), convertDepthImage(depthBuffer), 1.0);
			}
		}
	}

	io::Image::Ptr pcl::Kinect2Grabber::convertColorImage(const std::vector<RGBQUAD>& buffer)
	{
		//TODO: convert directly from YUY2
		//convert to rgb buffer
		converted_buffer.resize(colorHeight*colorWidth*3);
		for (unsigned int i = 0; i < buffer.size(); i++)
		{
			converted_buffer[i * 3 + 0] = buffer[i].rgbRed;
			converted_buffer[i * 3 + 1] = buffer[i].rgbGreen;
			converted_buffer[i * 3 + 2] = buffer[i].rgbBlue;
		}

		oni_color_frame.data = (void*)&converted_buffer[0];
		oni_color_frame.dataSize = converted_buffer.size();
		oni_color_frame.height = colorHeight;
		oni_color_frame.width = colorWidth;
		oni_color_frame.stride = colorWidth*3;

		openni::VideoFrameRef frame;
		frame._setFrame(&oni_color_frame);
		color_frameWrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		color_image =
			boost::make_shared<io::ImageRGB24>(color_frameWrapper);

		return color_image;
	}

	io::DepthImage::Ptr pcl::Kinect2Grabber::convertDepthImage(const std::vector<UINT16>& buffer)
	{
		oni_depth_frame.data = (void*)&buffer[0];
		oni_depth_frame.dataSize = buffer.size()*sizeof(UINT16);
		oni_depth_frame.height = depthHeight;
		oni_depth_frame.width = depthWidth;
		oni_depth_frame.stride = depthWidth*sizeof(UINT16);
		
		openni::VideoFrameRef frame;
		frame._setFrame(&oni_depth_frame);
		depth_frameWrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		CameraIntrinsics intrinsics;
		mapper->GetDepthCameraIntrinsics(&intrinsics);

		//parameters are not available during first couple of seconds
		if (!intrinsics.FocalLengthX)
			intrinsics.FocalLengthX = 364.82281494140625;

		float focalLength = intrinsics.FocalLengthX;
		float baseline = 52;
		pcl::uint64_t no_sample_value = 0;
		pcl::uint64_t shadow_value = 0;

		depth_image =
			boost::make_shared<io::DepthImage>(depth_frameWrapper, baseline, focalLength, shadow_value, no_sample_value);

		return depth_image;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::Kinect2Grabber::convertDepthToPointXYZ(UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

		cloud->width = static_cast<uint32_t>( depthWidth );
		cloud->height = static_cast<uint32_t>( depthHeight );
		cloud->is_dense = false;

		cloud->points.resize( cloud->height * cloud->width );

		pcl::PointXYZ* pt = &cloud->points[0];
		for( int y = 0; y < depthHeight; y++ ){
			for( int x = 0; x < depthWidth; x++, pt++ ){
				pcl::PointXYZ point;

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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::Kinect2Grabber::convertRGBDepthToPointXYZRGB( RGBQUAD* colorBuffer, UINT16* depthBuffer )
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );

		cloud->width = static_cast<uint32_t>( depthWidth );
		cloud->height = static_cast<uint32_t>( depthHeight );
		cloud->is_dense = false;

		cloud->points.resize( cloud->height * cloud->width );

		pcl::PointXYZRGB* pt = &cloud->points[0];
		for( int y = 0; y < depthHeight; y++ ){
			for( int x = 0; x < depthWidth; x++, pt++ ){
				pcl::PointXYZRGB point;

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

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl::Kinect2Grabber::convertRGBDepthToPointXYZRGBA(RGBQUAD* colorBuffer, UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		if (depth_space_points.size() != cloud->points.size())
		{
			depth_space_points.resize(cloud->points.size());
			for (int y = 0, int indx = 0; y < depthHeight; y++){
				for (int x = 0; x < depthWidth; x++, indx++){
					depth_space_points[indx].X = x;
					depth_space_points[indx].Y = y;
				}
			}
		}

		if (camera_space_points.size() != depth_space_points.size())
			camera_space_points.resize(depth_space_points.size());

		if (color_space_points.size() != depth_space_points.size())
			color_space_points.resize(depth_space_points.size());

		mapper->MapDepthPointsToCameraSpace(depth_space_points.size(), &depth_space_points[0], depth_space_points.size(), depthBuffer, camera_space_points.size(), &camera_space_points[0]);
		mapper->MapDepthPointsToColorSpace(depth_space_points.size(), &depth_space_points[0], depth_space_points.size(), depthBuffer, color_space_points.size(), &color_space_points[0]);

		pcl::PointXYZRGBA* pt = &cloud->points[0];
		for (int i = 0; i < cloud->points.size(); i++, pt++)
		{
			int colorX = (int)color_space_points[i].X;
			int colorY = (int)color_space_points[i].Y;

			if ((colorX >= 0) && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight))
			{
				pt->x = camera_space_points[i].X;
				pt->y = camera_space_points[i].Y;
				pt->z = camera_space_points[i].Z;
				pt->b = colorBuffer[i].rgbBlue;
				pt->g = colorBuffer[i].rgbGreen;
				pt->r = colorBuffer[i].rgbRed;
			}		
		}

		return cloud;
	}

	void pcl::Kinect2Grabber::PrintCalib()
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

