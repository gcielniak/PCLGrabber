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

namespace pcl
{
	template < typename ImageT >
	boost::shared_ptr<ImageT> ToImageRGB24(const unsigned char* buffer, int width, int height)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	boost::shared_ptr<io::Image> ToImageRGB24<io::Image>(const unsigned char* buffer, int width, int height)
	{
		OniFrame *oni_frame = new OniFrame();
		oni_frame->data = (void*)buffer;
		oni_frame->dataSize = width * height * 3;
		oni_frame->height = height;
		oni_frame->width = width;
		oni_frame->stride = width * 3;

		openni::VideoFrameRef frame;
		frame._setFrame(oni_frame);
		io::FrameWrapper::Ptr frame_wrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		return boost::make_shared<io::ImageRGB24>(frame_wrapper);
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	boost::shared_ptr<openni_wrapper::Image> ToImageRGB24<openni_wrapper::Image>(const unsigned char* buffer, int width, int height)
	{
		boost::shared_ptr< xn::ImageMetaData > frame_wrapper = boost::make_shared<xn::ImageMetaData>();

		frame_wrapper->ReAdjust(width, height, XnPixelFormat::XN_PIXEL_FORMAT_RGB24, &buffer[0]);

		return boost::make_shared<openni_wrapper::ImageRGB24>(frame_wrapper);
	}
#endif

	template < typename ImageT >
	boost::shared_ptr<ImageT> ToDepthImage(const unsigned short* buffer, int width, int height, float focal_length)
	{
	}

#ifdef HAVE_OPENNI2
	template <>
	io::DepthImage::Ptr ToDepthImage<io::DepthImage>(const unsigned short* buffer, int width, int height, float focal_length)
	{
		OniFrame *oni_frame = new OniFrame();
		oni_frame->data = (void*)buffer;
		oni_frame->dataSize = width * height * sizeof(unsigned short);
		oni_frame->height = height;
		oni_frame->width = width;
		oni_frame->stride = width * sizeof(unsigned short);

		openni::VideoFrameRef frame;
		frame._setFrame(oni_frame);
		io::FrameWrapper::Ptr frame_wrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		return boost::make_shared<io::DepthImage>(frame_wrapper, 0.0, focal_length, 0.0, 0.0);
	}
#endif

#ifdef HAVE_OPENNI
	template <>
	openni_wrapper::DepthImage::Ptr ToDepthImage<openni_wrapper::DepthImage>(const unsigned short* buffer, int width, int height, float focal_length)
	{
		boost::shared_ptr< xn::DepthMetaData > frame_wrapper = boost::make_shared<xn::DepthMetaData>();

		frame_wrapper->ReAdjust(width, height, (const XnDepthPixel*)&buffer[0]);

		return boost::make_shared<openni_wrapper::DepthImage>(frame_wrapper, 0.0, focal_length, 0.0, 0.0);
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

