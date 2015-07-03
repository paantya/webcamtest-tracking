#pragma once
#include <opencv2\highgui\highgui.hpp>
#include <QtCore\qmutex.h>
#include <deque>
#include <list>

class TSDataHandler
{
public:
	TSDataHandler(int frameLimit = 150);
	~TSDataHandler();
	void WriteFrame(cv::Mat input);
	bool ReadFrame(cv::Mat &output);
	int frameLimit;
private:
	cv::Mat frame;
	std::deque<cv::Mat> frame_queue;

};

