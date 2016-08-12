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
#include "FileGrabberExt.h"
#include "DeviceInput.h"

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
	#ifdef HAVE_OPENCV
	typedef CvMatExt ImageT;
	typedef CvMatExt DepthT;
	#elif HAVE_OPENNI2
	typedef io::Image ImageT;
	typedef io::DepthImage DepthT;
	#elif HAVE_OPENNI
	typedef openni_wrapper::Image ImageT;
	typedef openni_wrapper::DepthImage DepthT;
	#endif

	Grabber* grabber = 0;
	std::string file_name;
	int device = 0, platform = 0;
	float fps = 10.0;
	bool repeat = false, swap_rb_channels = false;

	FileOutput<PointT, ImageT, DepthT> writer;
	BasicViewer<PointT, ImageT, DepthT> viewer;

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-l") == 0)
		{
			DeviceInput device_input;
			device_input.ListAllDevices();
			return 0;
		}
		else if ((strcmp(argv[i], "-p") == 0) && (i < (argc - 1))) { platform = atoi(argv[++i]); }
		else if ((strcmp(argv[i], "-d") == 0) && (i < (argc - 1))) { device = atoi(argv[++i]); }
		else if ((strcmp(argv[i], "-w") == 0) && (i < (argc - 1))) { writer.Format(atoi(argv[++i])); }
		else if ((strcmp(argv[i], "-f") == 0) && (i < (argc - 1))) {
			file_name = argv[++i];
			writer.SimulatedTime(true);
			writer.OutputDir(".\\data\\" + boost::filesystem::path(argv[i]).filename().string() + "_copy\\");
		}
		else if ((strcmp(argv[i], "-fps") == 0) && (i < (argc - 1))) { fps = atof(argv[++i]); }
		else if (strcmp(argv[i], "-r") == 0) { repeat = true; }
		else if (strcmp(argv[i], "-vc") == 0) { viewer.VisualiseCloudPoint(true); }
		else if (strcmp(argv[i], "-vi") == 0) { viewer.VisualiseImages(true); }
		else if (strcmp(argv[i], "-s") == 0) { swap_rb_channels = true; }
		else if (strcmp(argv[i], "-h") == 0) { print_help(); }
	}

	if (file_name != "") { //select a file grabber (pcd or pclzf/png)
		try { grabber = new PCDGrabberExt<PointT, ImageT, DepthT>(file_name, fps, repeat); }
		catch (pcl::PCLException&) {}

		if (!grabber) {
			try { grabber = new ImageGrabberExt<PointT, ImageT, DepthT>(file_name, fps, repeat, swap_rb_channels); }
			catch (pcl::PCLException&) {}
		}
	}
	else { //select device
		DeviceInput device_input;
		grabber = device_input.GetGrabber(platform, device);
	}

	if (!grabber) {
		cerr << "Could not initialise the specified input." << endl;
		return 0;
	}

	viewer.RegisterCallbacks(grabber);
	writer.RegisterCallbacks(grabber);

	grabber->start();

	while (viewer.SpinOnce() && grabber->isRunning())
		;

	grabber->stop();

	return 0;
}

