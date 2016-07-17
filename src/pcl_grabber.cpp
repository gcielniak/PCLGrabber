#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#pragma warning( disable : 4996)
#endif

#ifdef HAVE_OPENNI
#include <pcl/io/openni_camera/image_metadata_wrapper.h>
#include <pcl/io/openni_camera/openni_image_rgb24.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#endif

#include "ImageUtils.h"
#include "BasicViewer.h"
#include "FileOutput.h"
#include "BasicGrabber.h"

using namespace pcl;

void print_help()
{
	cerr << "PCLGrabber usage:" << endl;

	cerr << "  -l : list all available input platforms and devices" << endl;
	cerr << "  -p : specify the input platform" << endl;
	cerr << "  -d : specify the input device" << endl;
	cerr << "  -f : use files from the specified directory" << endl;
	cerr << "  -s : swap red and blue channels (BGR format by default)" << endl;
	cerr << "-fps : set fps rate for file input" << endl;
	cerr << "  -r : loop the seuqence for file input" << endl;
	cerr << " -vc : visualise the cloud point" << endl;
	cerr << " -vi : visualise images" << endl;
	cerr << "  -w : write files using a specific format:" << endl;
	cerr << "       0 - pclzf" << endl;
	cerr << "       1 - pcd" << endl;
	cerr << "       2 - png" << endl;
	cerr << "       3 - png registered" << endl;
	cerr << "  -h : print this message" << endl;
}

int main(int argc, char **argv)
{
	if (argc == 1)
	{
		print_help();
		exit(0);
	}

	typedef PointXYZRGBA PointT;
	BasicGrabber<PointT> grabber;
#ifdef HAVE_OPENCV
	FileOutput<PointT, CvMatExt, CvMatExt> writer;
	BasicViewer<PointT, CvMatExt, CvMatExt> viewer;
#elif HAVE_OPENNI2
	FileOutput<PointT, io::Image, io::DepthImage> writer;
	BasicViewer<PointT, io::Image, io::DepthImage> viewer;
#elif HAVE_OPENNI
	FileOutput<PointT, openni_wrapper::Image, openni_wrapper::DepthImage> writer;
	BasicViewer<PointT, openni_wrapper::Image, openni_wrapper::DepthImage> viewer;
#endif

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-l") == 0)
		{
			DeviceInput device_input;
			device_input.ListAllDevices();
			return 0;
		}
		else if ((strcmp(argv[i], "-p") == 0) && (i < (argc - 1))) { grabber.Platform(atoi(argv[++i])); }
		else if ((strcmp(argv[i], "-d") == 0) && (i < (argc - 1))) { grabber.Device(atoi(argv[++i])); }
		else if ((strcmp(argv[i], "-w") == 0) && (i < (argc - 1))) { writer.Format(atoi(argv[++i])); }
		else if ((strcmp(argv[i], "-f") == 0) && (i < (argc - 1))) {
			grabber.File(argv[++i]); 
			writer.SimulatedTime(true);
			writer.OutputDir(".\\data\\" + boost::filesystem::path(argv[i]).filename().string() + "_copy\\");
		}
		else if ((strcmp(argv[i], "-fps") == 0) && (i < (argc - 1))) { grabber.FPS(atof(argv[++i])); }
		else if (strcmp(argv[i], "-r") == 0) { grabber.Repeat(true); }
		else if (strcmp(argv[i], "-vc") == 0) { viewer.VisualiseCloudPoint(true); }
		else if (strcmp(argv[i], "-vi") == 0) { viewer.VisualiseImages(true); }
		else if (strcmp(argv[i], "-s") == 0) { grabber.SwapRBChannels(true); }
		else if (strcmp(argv[i], "-h") == 0) { print_help(); }
	}

	try
	{
		grabber.Init();
	}
	catch (pcl::PCLException&)
	{
		cerr << "Could not initialise the specified input." << endl;
		return 0;	
	}

	viewer.RegisterCallbacks(grabber.GetGrabber());
	writer.RegisterCallbacks(grabber.GetGrabber());

	grabber.Start();

	while (viewer.SpinOnce())
		;

	grabber.Stop();

	return 0;
}

