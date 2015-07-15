#include "WebcamCapture.h"

void WebcamCapture::run()
{

	Mat orig;
	TimerCreate();
	while (isRunning())
	{
		TimerUpdate();
		vector<unsigned char> m_status;
		vector<float>         m_error;
		vector<Point2f> trackedPts;
		if (mVideoCapture->isOpened())
		{
			if (mVideoCapture->read(orig))
				mDataHandler_out->WriteFrame(orig);
		}
		else
		{
			this->exit(-1);
			
		}
		
		TimerElapsed();
	}
	mVideoCapture->release();
	this->exit(0);
}

WebcamCapture::WebcamCapture(TSDataHandler *dh_out)
{
	// init our capture
	this->mVideoCapture = new VideoCapture;
	
	mVideoCapture->open(0); // open default device
	mVideoCapture->set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	mVideoCapture->set(CV_CAP_PROP_FRAME_WIDTH, 320);
	this->mDataHandler_out = dh_out;
}

WebcamCapture::~WebcamCapture()
{
	mVideoCapture->release();
}
