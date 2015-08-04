#pragma once
#include "ImageUtils.h"

namespace pcl
{
	template < typename ImageT >
	class LZFRGB24ImageReaderExt : public io::LZFRGB24ImageReader
	{
	public:
		/** Empty constructor */
		LZFRGB24ImageReaderExt() : io::LZFRGB24ImageReader() {}

		/** Empty destructor */
		virtual ~LZFRGB24ImageReaderExt() {}

		boost::shared_ptr<ImageT> read(const std::string &filename, vector<unsigned char>& buffer)
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

			buffer.resize(image_size * 3);

			unsigned char *bf = &buffer[0];

			for (int i = 0; i < image_size; i++)
			{
				*bf++ = *color_r++;
				*bf++ = *color_g++;
				*bf++ = *color_b++;
			}

			return ToImageRGB24<ImageT>(&buffer[0], getWidth(), getHeight());
		}
	};

	template < typename ImageT >
	class LZFDepth16ImageReaderExt : public io::LZFDepth16ImageReader
	{
	public:
		/** Empty constructor */
		LZFDepth16ImageReaderExt() : io::LZFDepth16ImageReader() {}

		/** Empty destructor */
		virtual ~LZFDepth16ImageReaderExt() {}

		boost::shared_ptr<ImageT> read(const std::string &filename, vector<unsigned short>& buffer, float focal_point)
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

			buffer.assign(depth_data, depth_data + getWidth() * getHeight());

			return ToDepthImage<ImageT>(&buffer[0], getWidth(), getHeight(), focal_point);
		}
	};
}