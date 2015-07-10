#pragma once
#include <QtCore\qthread.h>
#include <opencv2\highgui\highgui.hpp>
#include "TSDataHandler.h"
#include "TimingsDebug.h"

class OutputWindow :
	public QThread
{
public:
	OutputWindow(TSDataHandler *dh_in);
	~OutputWindow();
private:
	TSDataHandler *dh_in;
	void run();
};

