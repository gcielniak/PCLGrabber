#pragma once

#include <pcl/io/lzf_image_io.h>
#include <pcl/console/print.h>

#ifdef HAVE_OPENNI2
#include <pcl/io/image.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>

namespace pcl
{
	io::Image::Ptr ToImageRGB24(const unsigned char* buffer, int width, int height)
	{
		io::Image::Ptr image;

		OniFrame *oni_frame = new OniFrame();
		oni_frame->data = (void*)buffer;
		oni_frame->dataSize = width * height * 3;
		oni_frame->height = height;
		oni_frame->width = width;
		oni_frame->stride = width * 3;

		openni::VideoFrameRef frame;
		frame._setFrame(oni_frame);
		io::FrameWrapper::Ptr frame_wrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		image = boost::make_shared<io::ImageRGB24>(frame_wrapper);

		return image;
	}

	io::DepthImage::Ptr ToDepthImage(const unsigned short* buffer, int width, int height, float focal_length)
	{
		io::DepthImage::Ptr image;

		OniFrame *oni_frame = new OniFrame();
		oni_frame->data = (void*)buffer;
		oni_frame->dataSize = width * height * sizeof(unsigned short);
		oni_frame->height = height;
		oni_frame->width = width;
		oni_frame->stride = width * sizeof(unsigned short);

		openni::VideoFrameRef frame;
		frame._setFrame(oni_frame);
		io::FrameWrapper::Ptr frame_wrapper = boost::make_shared<io::openni2::Openni2FrameWrapper>(frame);

		image = boost::make_shared<io::DepthImage>(frame_wrapper, 0.0, focal_length, 0.0, 0.0);

		return image;
	}
}
#endif

#ifdef HAVE_OPENNI
#include <pcl/io/openni_camera/image_metadata_wrapper.h>
#include <pcl/io/openni_camera/openni_image_rgb24.h>
#include <pcl/io/openni_camera/image_depth.h>

namespace pcl
{
	openni_wrapper::Image::Ptr ToImageRGB24Oni(const unsigned char* buffer, int width, int height)
	{
		boost::shared_ptr< xn::ImageMetaData > frame_wrapper = boost::make_shared<xn::ImageMetaData>();

		frame_wrapper->ReAdjust(width, height, XnPixelFormat::XN_PIXEL_FORMAT_RGB24, &buffer[0]);

		return boost::make_shared<openni_wrapper::ImageRGB24>(frame_wrapper);
	}

	openni_wrapper::DepthImage::Ptr ToDepthImageOni(const unsigned short* buffer, int width, int height, float focal_length)
	{
		boost::shared_ptr< xn::DepthMetaData > frame_wrapper = boost::make_shared<xn::DepthMetaData>();

		frame_wrapper->ReAdjust(width, height, (const XnDepthPixel*)&buffer[0]);

		return boost::make_shared<openni_wrapper::DepthImage>(frame_wrapper, 0.0, focal_length, 0.0, 0.0);
	}
}
#endif

