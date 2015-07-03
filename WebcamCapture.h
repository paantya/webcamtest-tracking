#pragma once
#include <QtCore\qthread.h>
#include <QtCore\qdebug.h>
#include <QtCore\qmutex.h>
//#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
//#include <opencv2\objdetect\objdetect.hpp>

#include "TSDataHandler.h"
#include <QtCore\qthread.h>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\video\video.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <iostream>

class WebcamCapture : public QThread
{
public:
	WebcamCapture(TSDataHandler *dh);
	WebcamCapture();
	~WebcamCapture();
	void stop();
private:
	cv::VideoCapture *vc;
	void run();
	bool running;
	TSDataHandler *dh;
};

