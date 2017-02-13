#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#pragma warning( disable : 4996)
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

using namespace PCLGrabber;

int main(int argc, char **argv) {

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
	std::string file_name, output_path;
	int device = 0, platform = 0;
	float fps = 10.0;
	bool repeat = false, simulated_time = false;
	bool vis_cloud = false, vis_images = false;
	int output_format = -1;

	PCLGrabber::FileOutputBase* writer;
	PCLGrabber::BasicViewerBase* viewer;

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-l") == 0)
		{
			PCLGrabber::DeviceInput::ListAllDevices();
			return 0;
		}
		else if ((strcmp(argv[i], "-p") == 0) && (i < (argc - 1))) { platform = atoi(argv[++i]); }
		else if ((strcmp(argv[i], "-d") == 0) && (i < (argc - 1))) { device = atoi(argv[++i]); }
		else if ((strcmp(argv[i], "-w") == 0) && (i < (argc - 1))) { output_format = atoi(argv[++i]); }
		else if ((strcmp(argv[i], "-f") == 0) && (i < (argc - 1))) {
			file_name = argv[++i];
			simulated_time = true;
			if (*file_name.rbegin() == '/')//check last character so it works with Linux dirs
                file_name = file_name.substr(0, file_name.length()-1);
			output_path = "./data/" + boost::filesystem::path(file_name).filename().string() + "_copy/";
		}
		else if ((strcmp(argv[i], "-fps") == 0) && (i < (argc - 1))) { fps = atof(argv[++i]); }
		else if (strcmp(argv[i], "-r") == 0) { repeat = true; }
		else if (strcmp(argv[i], "-vc") == 0) { vis_cloud = true; }
		else if (strcmp(argv[i], "-vi") == 0) { vis_images = true; }
		else if (strcmp(argv[i], "-h") == 0) { print_help(); }
	}

	if (file_name != "") { //select a file grabber (pcd or pclzf/png)
		try { grabber = new PCLGrabber::PCDGrabberExt<PointT, ImageT, DepthT>(file_name, fps, repeat); }
		catch (pcl::PCLException&) {}

		if (!grabber) {
			try { grabber = new PCLGrabber::ImageGrabberExt<PointT, ImageT, DepthT>(file_name, fps, repeat); }
			catch (pcl::PCLException&) {}
		}

		if (grabber) {
			writer = new FileOutput<PointT, ImageT, DepthT>();
			viewer = new BasicViewer<PointT, ImageT, DepthT>();
		}
	}
	else { //select device
		PCLGrabber::DeviceInput* device_input = new PCLGrabber::DeviceInput();

		grabber = device_input->GetGrabber(platform, device);

		switch (device_input->GetPlatformType()) {
		case PCLGrabber::OPENNI2_PLATFORM:
			writer = new FileOutput<PointT, io::Image, io::DepthImage>();
			viewer = new BasicViewer<PointT, io::Image, io::DepthImage>();
			break;
		case PCLGrabber::OPENNI_PLATFORM:
			writer = new FileOutput<PointT, openni_wrapper::Image, openni_wrapper::DepthImage>();
			viewer = new BasicViewer<PointT, openni_wrapper::Image, openni_wrapper::DepthImage>();
			break;
		default:
			writer = new FileOutput<PointT, ImageT, DepthT>();
			viewer = new BasicViewer<PointT, ImageT, DepthT>();
			break;
		}
	}

	if (!grabber) {
		cerr << "Could not initialise the specified input." << endl;
		return 0;
	}

	viewer->VisualiseCloudPoint(vis_cloud);
	viewer->VisualiseImages(vis_images);
	viewer->RegisterCallbacks(grabber);

	if (output_format != -1) {
		writer->SimulatedTime(simulated_time);
		if (output_path != "")
			writer->OutputDir(output_path);
		writer->Format(output_format);
		writer->RegisterCallbacks(grabber);
	}

	grabber->start();

	while (viewer->SpinOnce())
		;

	grabber->stop();

	return 0;

}

