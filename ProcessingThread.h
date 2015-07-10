#pragma once

#include <QtCore\qthread.h>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\video\video.hpp>
#include "TSDataHandler.h"
#include "TimingsDebug.h"

using namespace std;
using namespace cv;
using std::vector;

class ProcessingThread :
	public QThread
{
public:
	ProcessingThread(TSDataHandler *dh_in, TSDataHandler *dh_out = nullptr);
	~ProcessingThread();
private:
	void run();
	TSDataHandler *dh_in, *dh_out;
};