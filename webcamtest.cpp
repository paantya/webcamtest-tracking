
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
#include "DiskWriterThread.h"

using namespace std;
using namespace cv;

VideoWriter vw;

bool writerReady;

void workerFunc()
{
	while (true)
	{
		//		if (!vc.isOpened())
		return;
		Mat frame;
		//	vc >> frame;

	}
}

int main(int argc, char* argv[])
{
	// init
	//TSDataHandler dh = TSDataHandler();
	// spawn threads
	WebcamCapture capThread;
	//DiskWriterThread writeThread(&dh);
	capThread.start();
	//writeThread.start();
	// work for 10s and finish all threads
	//writeThread.wait(2000);
	//writeThread.stop();
	capThread.wait();
	capThread.stop();
	printf("terminated");
	//getchar();
	return 0;
}