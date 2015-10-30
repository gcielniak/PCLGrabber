#pragma once

#include <pcl/io/lzf_image_io.h>
#include <pcl/console/print.h>
#include <boost/make_shared.hpp>
#include <pcl/common/time.h> //fps calculations

#ifdef HAVE_OPENNI2
#include <pcl/io/image_rgb24.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#endif

#ifdef HAVE_OPENNI
#include <pcl/io/openni_camera/image_metadata_wrapper.h>
#include <pcl/io/openni_camera/openni_image_rgb24.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#endif

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace pcl
{
#ifdef HAVE_OPENCV
	class CvMatExt {
		public:
			CvMatExt(const cv::Mat& image, long long timestamp, double focal_length=0.0) {
				this->image = image;
				this->timestamp = timestamp;
				this->focal_length = focal_length;
			}

			cv::Mat image;
			long long timestamp;
			double focal_length;
	};
#endif
	
	template < typename ImageT >
	boost::shared_ptr<ImageT> ToImageRGB24(const unsigned char* buffer, int width, int height, long long timestamp)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	boost::shared_ptr<io::Image> ToImageRGB24<io::Image>(const unsigned char* buffer, int width, int height, long long timestamp)
	{
		OniFrame *oni_frame = new OniFrame();
		oni_frame->data = (void*)buffer;
		oni_frame->dataSize = width * height * 3;
		oni_frame->height = height;
		oni_frame->width = width;
		oni_frame->stride = width * 3;
		oni_frame->timestamp = timestamp;

		openni::VideoFrameRef frame;
		frame._setFrame(oni_frame);
		io::FrameWrapper::Ptr frame_wrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		return boost::make_shared<io::ImageRGB24>(frame_wrapper);
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	boost::shared_ptr<openni_wrapper::Image> ToImageRGB24<openni_wrapper::Image>(const unsigned char* buffer, int width, int height, long long timestamp)
	{
		boost::shared_ptr< xn::ImageMetaData > frame_wrapper = boost::make_shared<xn::ImageMetaData>();

		frame_wrapper->ReAdjust(width, height, XN_PIXEL_FORMAT_RGB24, &buffer[0]);
		frame_wrapper->Timestamp() = timestamp;

		return boost::make_shared<openni_wrapper::ImageRGB24>(frame_wrapper);
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	boost::shared_ptr<CvMatExt> ToImageRGB24<CvMatExt>(const unsigned char* buffer, int width, int height, long long timestamp)
	{
		return boost::make_shared<CvMatExt>(cv::Mat(height, width, CV_8UC3, (void*)buffer, width * 3), timestamp);
	}
#endif

	template < typename ImageT >
	boost::shared_ptr<ImageT> ToDepthImage(const unsigned short* buffer, int width, int height, float focal_length, long long timestamp)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	io::DepthImage::Ptr ToDepthImage<io::DepthImage>(const unsigned short* buffer, int width, int height, float focal_length, long long timestamp)
	{
		OniFrame *oni_frame = new OniFrame();
		oni_frame->data = (void*)buffer;
		oni_frame->dataSize = width * height * sizeof(unsigned short);
		oni_frame->height = height;
		oni_frame->width = width;
		oni_frame->stride = width * sizeof(unsigned short);
		oni_frame->timestamp = timestamp;

		openni::VideoFrameRef frame;
		frame._setFrame(oni_frame);
		io::FrameWrapper::Ptr frame_wrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		return boost::make_shared<io::DepthImage>(frame_wrapper, 0.0, focal_length, 0.0, 0.0);
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	openni_wrapper::DepthImage::Ptr ToDepthImage<openni_wrapper::DepthImage>(const unsigned short* buffer, int width, int height, float focal_length, long long timestamp)
	{
		boost::shared_ptr< xn::DepthMetaData > frame_wrapper = boost::make_shared<xn::DepthMetaData>();

		frame_wrapper->ReAdjust(width, height, (const XnDepthPixel*)&buffer[0]);
		frame_wrapper->Timestamp() = timestamp;

		return boost::make_shared<openni_wrapper::DepthImage>(frame_wrapper, 0.0, focal_length, 0.0, 0.0);
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	boost::shared_ptr<CvMatExt> ToDepthImage<CvMatExt>(const unsigned short* buffer, int width, int height, float focal_length, long long timestamp)
	{
		return boost::make_shared<CvMatExt>(cv::Mat(height, width, CV_16UC1, (void*)buffer, width * 2), timestamp, focal_length);
	}
#endif

	template < typename ImageT >
	unsigned char* GetRGBBuffer(const boost::shared_ptr<ImageT>&)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	unsigned char* GetRGBBuffer(const boost::shared_ptr<io::Image>& image)
	{
		if (image->getEncoding() != io::Image::RGB)
		{
			unsigned char* buffer = new unsigned char[image->getWidth()*image->getHeight() * 3];
			image->fillRGB(image->getWidth(), image->getHeight(), buffer);
			return buffer;
		}
		else
			return (unsigned char*)image->getData();
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	unsigned char* GetRGBBuffer(const boost::shared_ptr<openni_wrapper::Image>& image)
	{
		if (image->getEncoding() != openni_wrapper::Image::RGB)
		{
			unsigned char* buffer = new unsigned char[image->getWidth()*image->getHeight() * 3];
			image->fillRGB(image->getWidth(), image->getHeight(), buffer);
			return buffer;
		}
		else
			return (unsigned char*)image->getMetaData().Data();
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	unsigned char* GetRGBBuffer(const boost::shared_ptr<CvMatExt>& image)
	{
		return (unsigned char*)image->image.data;
	}
#endif

	template < typename ImageT >
	unsigned short* GetDepthBuffer(const boost::shared_ptr<ImageT>&)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	unsigned short* GetDepthBuffer(const boost::shared_ptr<io::DepthImage>& image)
	{
		return (unsigned short*)image->getData();
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	unsigned short* GetDepthBuffer(const boost::shared_ptr<openni_wrapper::DepthImage>& image)
	{
		return (unsigned short*)image->getDepthMetaData().Data();
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	unsigned short* GetDepthBuffer(const boost::shared_ptr<CvMatExt>& image)
	{
		return (unsigned short*)image->image.data;
	}
#endif

	template < typename ImageT >
	float GetFocalLength(const boost::shared_ptr<ImageT>&)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	float GetFocalLength(const boost::shared_ptr<io::DepthImage>& image)
	{
		return image->getFocalLength();
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	float GetFocalLength(const boost::shared_ptr<openni_wrapper::DepthImage>& image)
	{
		return image->getFocalLength();
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	float GetFocalLength(const boost::shared_ptr<CvMatExt>& image)
	{
		return image->focal_length;
	}
#endif

	template <typename ImageT>
	long long GetTimeStamp(const boost::shared_ptr<ImageT>&)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	long long GetTimeStamp(const boost::shared_ptr<io::Image>& image)
	{
		return image->getTimestamp();
	}

	template <>
	long long GetTimeStamp(const boost::shared_ptr<io::DepthImage>& image)
	{
		return image->getTimestamp();
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	long long GetTimeStamp(const boost::shared_ptr<openni_wrapper::Image>& image)
	{
		return image->getTimeStamp();
	}

	template <>
	long long GetTimeStamp(const boost::shared_ptr<openni_wrapper::DepthImage>& image)
	{
		return image->getTimeStamp();
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	long long GetTimeStamp(const boost::shared_ptr<CvMatExt>& image)
	{
		return image->timestamp;
	}
#endif

	template <typename ImageT>
	int GetWidth(const boost::shared_ptr<ImageT>&)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	int GetWidth(const boost::shared_ptr<io::Image>& image)
	{
		return image->getWidth();
	}

	template <>
	int GetWidth(const boost::shared_ptr<io::DepthImage>& image)
	{
		return image->getWidth();
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	int GetWidth(const boost::shared_ptr<openni_wrapper::Image>& image)
	{
		return image->getWidth();
	}

	template <>
	int GetWidth(const boost::shared_ptr<openni_wrapper::DepthImage>& image)
	{
		return image->getWidth();
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	int GetWidth(const boost::shared_ptr<CvMatExt>& image)
	{
		return image->image.cols;
	}
#endif

	template <typename ImageT>
	int GetHeight(const boost::shared_ptr<ImageT>&)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	int GetHeight(const boost::shared_ptr<io::Image>& image)
	{
		return image->getHeight();
	}

	template <>
	int GetHeight(const boost::shared_ptr<io::DepthImage>& image)
	{
		return image->getHeight();
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	int GetHeight(const boost::shared_ptr<openni_wrapper::Image>& image)
	{
		return image->getHeight();
	}

	template <>
	int GetHeight(const boost::shared_ptr<openni_wrapper::DepthImage>& image)
	{
		return image->getHeight();
	}
#endif

#ifdef HAVE_OPENCV
	template <>
	int GetHeight(const boost::shared_ptr<CvMatExt>& image)
	{
		return image->image.rows;
	}
#endif

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
								    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
								    } \
}while(false)

}

