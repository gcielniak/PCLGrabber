#pragma once
#include "ImageUtils.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

namespace pcl {
	using namespace std;

	template < typename ImageT >
	class LZFRGB24ImageReaderExt : public io::LZFRGB24ImageReader
	{
	protected:
		boost::posix_time::ptime epoch;

	public:
		enum ColorFormat {
			CF_RGB,
			CF_BGR
		};

		/** Empty constructor */
		LZFRGB24ImageReaderExt() : io::LZFRGB24ImageReader(), epoch(boost::gregorian::date(1970, 1, 1)) {}

		/** Empty destructor */
		virtual ~LZFRGB24ImageReaderExt() {}

		boost::shared_ptr<ImageT> read(const std::string &filename, vector<unsigned char>& buffer, const ColorFormat color_format = CF_BGR)
		{
			uint32_t uncompressed_size;
			std::vector<char> compressed_data;
			if (!loadImageBlob(filename, compressed_data, uncompressed_size))
			{
				PCL_ERROR("[pcl::io::LZFRGB24ImageReaderExt::read] Unable to read image data from %s.\n", filename.c_str());
				return boost::shared_ptr<ImageT>();
			}

			if (uncompressed_size != getWidth() * getHeight() * 3)
			{
				PCL_DEBUG("[pcl::io::LZFRGB24ImageReaderExt::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFRGB24ImageReaderExt::read] Are you sure %s is a 24-bit RGB PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth() * getHeight() * 3, filename.c_str(), getImageType().c_str());
				return boost::shared_ptr<ImageT>();
			}

			std::vector<char> uncompressed_data(uncompressed_size);
			decompress(compressed_data, uncompressed_data);

			if (uncompressed_data.empty())
			{
				PCL_ERROR("[pcl::io::LZFRGB24ImageReaderExt::read] Error uncompressing data stored in %s!\n", filename.c_str());
				return boost::shared_ptr<ImageT>();
			}

			register int rgb_idx = 0;
			unsigned char *color_r = reinterpret_cast<unsigned char*> (&uncompressed_data[0]);
			unsigned char *color_g = reinterpret_cast<unsigned char*> (&uncompressed_data[getWidth() * getHeight()]);
			unsigned char *color_b = reinterpret_cast<unsigned char*> (&uncompressed_data[2 * getWidth() * getHeight()]);

			int image_size = getWidth() * getHeight();

			buffer.resize(image_size * 3);

			unsigned char *bf = &buffer[0];

			if (color_format == CF_RGB) {
				for (int i = 0; i < image_size; i++)
				{
					*bf++ = *color_r++;
					*bf++ = *color_g++;
					*bf++ = *color_b++;
				}
			}
			else if (color_format == CF_BGR) {
				for (int i = 0; i < image_size; i++)
				{
					*bf++ = *color_b++;
					*bf++ = *color_g++;
					*bf++ = *color_r++;
				}
			}

			boost::filesystem::path path(filename);
			boost::posix_time::ptime t = boost::posix_time::from_iso_string(path.filename().string().substr(6, 22));//image timestamp
			boost::posix_time::time_duration td(t - epoch);//time since epoch

			return ToImageRGB24<ImageT>(&buffer[0], getWidth(), getHeight(), td.total_microseconds());
		}
	};

	template < typename ImageT >
	class LZFDepth16ImageReaderExt : public io::LZFDepth16ImageReader
	{
	protected:
		boost::posix_time::ptime epoch;

	public:
		/** Empty constructor */
		LZFDepth16ImageReaderExt() : io::LZFDepth16ImageReader(), epoch(boost::gregorian::date(1970, 1, 1)) {}

		/** Empty destructor */
		virtual ~LZFDepth16ImageReaderExt() {}

		boost::shared_ptr<ImageT> read(const std::string &filename, vector<unsigned short>& buffer, float focal_point)
		{
			uint32_t uncompressed_size;
			std::vector<char> compressed_data;
			if (!loadImageBlob(filename, compressed_data, uncompressed_size))
			{
				PCL_ERROR("[pcl::io::LZFDepth16ImageReaderExt::read] Unable to read image data from %s.\n", filename.c_str());
				return boost::shared_ptr<ImageT>();
			}

			if (uncompressed_size != getWidth() * getHeight() * 2)
			{
				PCL_DEBUG("[pcl::io::LZFDepth16ImageReaderExt::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFDepth16ImageReaderExt::read] Are you sure %s is a 16-bit depth PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth() * getHeight() * 2, filename.c_str(), getImageType().c_str());
				return boost::shared_ptr<ImageT>();
			}

			std::vector<char> uncompressed_data(uncompressed_size);
			decompress(compressed_data, uncompressed_data);

			if (uncompressed_data.empty())
			{
				PCL_ERROR("[pcl::io::LZFDepth16ImageReaderExt::read] Error uncompressing data stored in %s!\n", filename.c_str());
				return boost::shared_ptr<ImageT>();
			}

			boost::filesystem::path path(filename);
			boost::posix_time::ptime t = boost::posix_time::from_iso_string(path.filename().string().substr(6, 22));//image timestamp
			boost::posix_time::time_duration td(t - epoch);//time since epoch

			unsigned short *depth_data = reinterpret_cast<unsigned short*> (&uncompressed_data[0]);

			buffer.assign(depth_data, depth_data + getWidth() * getHeight());

			return ToDepthImage<ImageT>(&buffer[0], getWidth(), getHeight(), focal_point, td.total_microseconds());
		}
	};

	class LZFBayer8ImageReaderExt : public io::LZFBayer8ImageReader
	{
	protected:
		boost::posix_time::ptime epoch;

	public:
		/** Empty constructor */
		LZFBayer8ImageReaderExt() : io::LZFBayer8ImageReader(), epoch(boost::gregorian::date(1970, 1, 1)) {}

		/** Empty destructor */
		virtual ~LZFBayer8ImageReaderExt() {}

		//////////////////////////////////////////////////////////////////////////////
		boost::shared_ptr<CvMatExt> read(const std::string &filename, std::vector<unsigned char>& buffer) {
			uint32_t uncompressed_size;
			std::vector<char> compressed_data;
			if (!loadImageBlob(filename, compressed_data, uncompressed_size))
			{
				PCL_ERROR("[pcl::io::LZFBayer8ImageReaderExt::read] Unable to read image data from %s.\n", filename.c_str());
				return boost::shared_ptr<CvMatExt>();
			}

			if (uncompressed_size != getWidth() * getHeight())
			{
				PCL_DEBUG("[pcl::io::LZFBayer8ImageReaderExt::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFBayer8ImageReader::read] Are you sure %s is a 8-bit Bayer PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth() * getHeight(), filename.c_str(), getImageType().c_str());
				return boost::shared_ptr<CvMatExt>();
			}

			std::vector<char> uncompressed_data(uncompressed_size);
			decompress(compressed_data, uncompressed_data);

			if (uncompressed_data.empty())
			{
				PCL_ERROR("[pcl::io::LZFBayer8ImageReaderExt::read] Error uncompressing data stored in %s!\n", filename.c_str());
				return boost::shared_ptr<CvMatExt>();
			}

			boost::filesystem::path path(filename);
			boost::posix_time::ptime t = boost::posix_time::from_iso_string(path.filename().string().substr(6, 22));//image timestamp
			boost::posix_time::time_duration td(t - epoch);//time since epoch

			unsigned char *ir_data = reinterpret_cast<unsigned char*> (&uncompressed_data[0]);

			buffer.assign(ir_data, ir_data + getWidth() * getHeight());

			return boost::make_shared<CvMatExt>(cv::Mat(getHeight(), getWidth(), CV_8UC1, (void*)&buffer[0], getWidth()), td.total_microseconds());
		}
	};
}