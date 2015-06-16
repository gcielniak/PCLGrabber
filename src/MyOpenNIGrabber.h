#pragma once
#include <pcl/io/openni2_grabber.h>

using namespace std;

namespace pcl
{
	class MyGrabber : public pcl::io::OpenNI2Grabber
	{
	public:
		virtual void start() override
		{
			try
			{
				// check if we need to start/stop any stream
				if (image_required_ && !device_->isColorStreamStarted())
				{
					block_signals();
					device_->startColorStream();
//					startSynchronization();
				}

				if (depth_required_ && !device_->isDepthStreamStarted())
				{
					block_signals();
					if (device_->hasColorSensor() && device_->isImageRegistrationModeSupported())
					{
						device_->setImageRegistrationMode(true);
					}
					device_->startDepthStream();
//					startSynchronization();
				}

				if (ir_required_ && !device_->isIRStreamStarted())
				{
					block_signals();
					device_->startIRStream();
				}
				running_ = true;
			}
			catch (IOException& ex)
			{
				PCL_THROW_EXCEPTION(pcl::IOException, "Could not start streams. Reason: " << ex.what());
			}

			// workaround, since the first frame is corrupted
			//boost::this_thread::sleep (boost::posix_time::seconds (1));
			unblock_signals();
		}
	};
}