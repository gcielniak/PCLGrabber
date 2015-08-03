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

	class LZFRGB24ImageReaderExt : public io::LZFRGB24ImageReader
	{
	public:
		/** Empty constructor */
		LZFRGB24ImageReaderExt() : io::LZFRGB24ImageReader() {}

		/** Empty destructor */
		virtual ~LZFRGB24ImageReaderExt() {}

		//////////////////////////////////////////////////////////////////////////////
		bool read(const std::string &filename, io::Image::Ptr& image)
		{
			uint32_t uncompressed_size;
			std::vector<char> compressed_data;
			if (!loadImageBlob(filename, compressed_data, uncompressed_size))
			{
				PCL_ERROR("[pcl::io::LZFRGB24ImageReaderExt::read] Unable to read image data from %s.\n", filename.c_str());
				return (false);
			}

			if (uncompressed_size != getWidth() * getHeight() * 3)
			{
				PCL_DEBUG("[pcl::io::LZFRGB24ImageReaderExt::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFRGB24ImageReaderExt::read] Are you sure %s is a 24-bit RGB PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth() * getHeight() * 3, filename.c_str(), getImageType().c_str());
				return (false);
			}

			std::vector<char> uncompressed_data(uncompressed_size);
			decompress(compressed_data, uncompressed_data);

			if (uncompressed_data.empty())
			{
				PCL_ERROR("[pcl::io::LZFRGB24ImageReaderExt::read] Error uncompressing data stored in %s!\n", filename.c_str());
				return (false);
			}

			register int rgb_idx = 0;
			unsigned char *color_r = reinterpret_cast<unsigned char*> (&uncompressed_data[0]);
			unsigned char *color_g = reinterpret_cast<unsigned char*> (&uncompressed_data[getWidth() * getHeight()]);
			unsigned char *color_b = reinterpret_cast<unsigned char*> (&uncompressed_data[2 * getWidth() * getHeight()]);

			int image_size = getWidth() * getHeight();

			unsigned char* image_buffer = new unsigned char[image_size * 3];

			for (unsigned int i = 0; i < image_size * 3; ++rgb_idx)
			{
				image_buffer[i++] = color_r[rgb_idx];
				image_buffer[i++] = color_g[rgb_idx];
				image_buffer[i++] = color_b[rgb_idx];
			}

			image = ToImageRGB24(image_buffer, getWidth(), getHeight());

			return true;
		}
	};

	class LZFDepth16ImageReaderExt : public io::LZFDepth16ImageReader
	{
	public:
		/** Empty constructor */
		LZFDepth16ImageReaderExt() : io::LZFDepth16ImageReader() {}

		/** Empty destructor */
		virtual ~LZFDepth16ImageReaderExt() {}

		//////////////////////////////////////////////////////////////////////////////
		bool read(const std::string &filename, io::DepthImage::Ptr& image)
		{
			uint32_t uncompressed_size;
			std::vector<char> compressed_data;
			if (!loadImageBlob(filename, compressed_data, uncompressed_size))
			{
				PCL_ERROR("[pcl::io::LZFDepth16ImageReaderExt::read] Unable to read image data from %s.\n", filename.c_str());
				return (false);
			}

			if (uncompressed_size != getWidth() * getHeight() * 2)
			{
				PCL_DEBUG("[pcl::io::LZFDepth16ImageReaderExt::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFDepth16ImageReaderExt::read] Are you sure %s is a 16-bit depth PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth() * getHeight() * 2, filename.c_str(), getImageType().c_str());
				return (false);
			}

			std::vector<char> uncompressed_data(uncompressed_size);
			decompress(compressed_data, uncompressed_data);

			if (uncompressed_data.empty())
			{
				PCL_ERROR("[pcl::io::LZFDepth16ImageReaderExt::read] Error uncompressing data stored in %s!\n", filename.c_str());
				return (false);
			}

			unsigned short *depth_data = reinterpret_cast<unsigned short*> (&uncompressed_data[0]);

			int image_size = getWidth() * getHeight();

			unsigned short* image_buffer = new unsigned short[image_size];

			for (unsigned int i = 0; i < image_size; i++)
			{
				image_buffer[i] = depth_data[i];
			}

			image = ToDepthImage(image_buffer, getWidth(), getHeight(), 0.0);

			return true;
		}
	};
}
#endif
