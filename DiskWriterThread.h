#pragma once
#include "TSDataHandler.h"
#include <QtCore\qthread.h>
#include "WebcamCapture.h"

class DiskWriterThread : public QThread
{
public:
	DiskWriterThread(TSDataHandler *dh);
	~DiskWriterThread();
	void stop();
private:
	void run();
	cv::VideoWriter vw;
	bool running;
	TSDataHandler *dh;
};

