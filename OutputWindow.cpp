#include "OutputWindow.h"


OutputWindow::OutputWindow(TSDataHandler *dh_in)
{
	this->dh_in = dh_in;
}

void OutputWindow::run()
{
	cv::namedWindow("Output", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("1", CV_WINDOW_AUTOSIZE);
	cv::Mat img;
	while (isRunning())
	{
		if (dh_in->ReadFrame(img)) {
			
			
		}
	}
}

OutputWindow::~OutputWindow()
{
}
