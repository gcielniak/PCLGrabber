#pragma once
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "GenICamDevice.h"

using namespace std;

namespace PCLGrabber {

	using namespace pcl;

	class GenICamGrabberBase : public Grabber {
	protected:
		boost::thread thread;
		mutable boost::mutex mutex;
		bool quit, running;
		GenICamera camera_1, camera_2;
		CvMatExt image_1, image_2;
		typedef void (Signal_ImageDepth)(const boost::shared_ptr<CvMatExt>&, const boost::shared_ptr<CvMatExt>&, float focal_length);

		boost::signals2::signal<Signal_ImageDepth>* signal_ImageDepth;

		virtual void threadFunction() {
			camera_1.Start();
			camera_2.Start();
			while (!quit) {
				if (signal_ImageDepth->num_slots()) {
					camera_1.TriggerSoftware();
					camera_2.TriggerSoftware();
					image_1.image = camera_1.Capture();
					image_2.image = camera_2.Capture();
					image_1.timestamp = camera_1.GetTimeStamp();
					image_2.timestamp = camera_2.GetTimeStamp();
					signal_ImageDepth->operator()(boost::make_shared<CvMatExt>(image_1), boost::make_shared<CvMatExt>(image_2), 0.0);
				}
			}
		}

	public:
		GenICamGrabberBase() :
			quit(false) {
			camera_1.Init();
			camera_2.Init();
			camera_2.SetCamera(1);
			camera_1.TriggerMode(true);
			camera_2.TriggerMode(true);
			cerr << "TM1: " << camera_1.TriggerMode() << endl;
			cerr << "TM2: " << camera_2.TriggerMode() << endl;
			signal_ImageDepth = createSignal<Signal_ImageDepth>();
		}

		~GenICamGrabberBase() throw() {
			stop();
			disconnect_all_slots<Signal_ImageDepth>();
		}

		void start() {
			running = true;
			thread = boost::thread(&GenICamGrabberBase::threadFunction, this);
		}

		void stop() {
			boost::unique_lock<boost::mutex> lock(mutex);
			quit = true;
			running = false;
			lock.unlock();
		}

		bool isRunning() const {
			boost::unique_lock<boost::mutex> lock(mutex);
			return running;
			lock.unlock();
		}

		std::string getName() const {
			return "GenICamera";
		}

		float getFramesPerSecond() const {
			float fps = std::numeric_limits<float>::quiet_NaN();
			return fps;
		}
	};
}
