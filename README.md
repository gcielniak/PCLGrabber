# PCLGrabber

## Sensor Support
 - ZED Camera (requires [ZED SDK](https://www.stereolabs.com/developers/))
 - Intel RealSense (requires [Intel RealSense SDK](https://software.intel.com/en-us/intel-realsense-sdk))
 - Genie Nanos (requires [Common Vision Blox](http://www.commonvisionblox.com/en/trialversions/))
 - Kinect 2 (requires [Kinect 2 SDK](https://www.microsoft.com/en-gb/download/details.aspx?id=44561))
 
## Prerequisites
- cmake
- PCL (1.7>=)
 
## Building
*Ubuntu 14.04 LTS*: using PCL 1.7
- set up the project: `mkdir build & cd build & cmake .. -G "Unix Makefiles" & make`.

*Windows 10*: using PCL 1.8 and Visual Studio 2015
- set up the project: `mkdir build & cd build & cmake .. -G "Visual Studio 14 2015 Win64"`;
- to quickly build the program from a command line: `cmake --build . --config release`.

## Running
The program allows for recording, converting and visualising 3D/image data. Check all available options by running the program without input arguments.
- to visualise: `-vc` to show point clouds and `-vi` to show images;
- input: `-p/-d` platform/device for a sensor or `-f` for a pre-recorded file folder;
- to record: `-w` flag. E.g. -w 2 to record data in PNG format. Can also be used with file input and then acts as a converter.
