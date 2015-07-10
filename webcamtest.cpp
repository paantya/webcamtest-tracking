
#include <opencv2\highgui\highgui.hpp>
#include "TSDataHandler.h"
#include <QtCore\qthread.h>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\video\video.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <iostream>


#include <iostream>
#include <stdio.h>

#include "WebcamCapture.h"
#include "ProcessingThread.h"
#include "DiskWriterThread.h"
#include "OutputWindow.h"

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
	// init
	TSDataHandler* dh_cap2proc = new TSDataHandler();
	TSDataHandler* dh_proc2out = new TSDataHandler();

	// spawn threads
	WebcamCapture capThread(dh_cap2proc);
	ProcessingThread procThread(dh_cap2proc, dh_proc2out);
	//OutputWindow winThread(dh_proc2out);
	capThread.start();
	procThread.start();

	Mat img;
	// work
	forever
	{
		if (dh_proc2out->ReadFrame(img))
		{
			imshow("Output", img);
			waitKey(1);
		}
	}

	// exit
	capThread.exit();
	procThread.exit();

	return 0;
}