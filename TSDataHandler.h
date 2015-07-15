#pragma once
#include <opencv2\highgui\highgui.hpp>
#include <QtCore\qmutex.h>
#include <deque>
#include <list>

class TSDataHandler
{
public:
	TSDataHandler(int frameLimit = 10);
	~TSDataHandler();
	void WriteFrame(cv::Mat input);
	bool ReadFrame(cv::Mat &output);
private:
	int mFrameLimit;
	cv::Mat mFrame;
	std::deque<cv::Mat> mFrameQueue;

};

