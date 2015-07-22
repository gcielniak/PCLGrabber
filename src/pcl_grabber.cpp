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
	cerr << "-fps : set fps rate for file input" << endl;
	cerr << "  -r : loop the seuqence for file input" << endl;
	cerr << " -vc : visualise the cloud point" << endl;
	cerr << " -vi : visualise images" << endl;
	cerr << "  -w : write files using a specific format:" << endl;
	cerr << "       0 - pclzf" << endl;
	cerr << "       1 - pcd" << endl;
	cerr << "       2 - png" << endl;
	cerr << "       3 - pclzf + pcd" << endl;
	cerr << "  -h : print this message" << endl;
}

int main(int argc, char **argv)
{
	if (argc == 1)
	{
		print_help();
		exit(0);
	}

	typedef PointXYZRGBA PointType;
	BasicGrabber<PointType> grabber;
	FileOutput<PointType> writer, aux_writer;
	BasicViewer<PointType> viewer;

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
		else if ((strcmp(argv[i], "-f") == 0) && (i < (argc - 1))) { grabber.File(argv[++i]); }
		else if ((strcmp(argv[i], "-fps") == 0) && (i < (argc - 1))) { grabber.FPS(atof(argv[++i])); }
		else if (strcmp(argv[i], "-r") == 0) { grabber.Repeat(true); }
		else if (strcmp(argv[i], "-vc") == 0) { viewer.VisualiseCloudPoint(true); }
		else if (strcmp(argv[i], "-vi") == 0) { viewer.VisualiseImages(true); }
		else if (strcmp(argv[i], "-h") == 0) { print_help(); }
	}

	grabber.Init();

	viewer.RegisterCallbacks(grabber.GetGrabber());

	if (writer.Format() == 3)
	{
		writer.Format(0); //pclzf
		string dir_name = writer.OutputDir();
		dir_name.insert(dir_name.length()-1, "_PCLZF");
		writer.OutputDir(dir_name);
		writer.RegisterCallbacks(grabber.GetGrabber());

		aux_writer.Format(1); //pcd
		dir_name = aux_writer.OutputDir();
		dir_name.insert(dir_name.length() - 1, "_PCD");
		aux_writer.OutputDir(dir_name);
		aux_writer.RegisterCallbacks(grabber.GetGrabber());
	}
	else
	{
		writer.RegisterCallbacks(grabber.GetGrabber());
	}

	grabber.Start();

	while (viewer.SpinOnce())
		;

	grabber.Stop();

	return 0;
}

