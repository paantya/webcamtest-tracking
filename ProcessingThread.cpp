#include "ProcessingThread.h"


ProcessingThread::ProcessingThread(TSDataHandler *dh_in, TSDataHandler *dh_out)
{
	this->dh_in = dh_in;
	if (dh_out == nullptr)
		this->dh_out = this->dh_in;
	else
		this->dh_out = dh_out;
}

void ProcessingThread::run()
{
	Mat orig, previmg, nextimg, output, mask;
	vector<Point2f> prev_pts, orig_pts;
	vector<uchar> m_status;
	Mat m_error;

	TimerCreate();

	while (isRunning())
	{
		TimerUpdate();
		while (dh_in->ReadFrame(orig))
		{
			vector<Point2f> next_pts, tracked_pts;
			output = orig.clone();
			bool useOpticalFlow = true;

#pragma region OpticalFlow
			if (useOpticalFlow){
				vector<Point2f> orig_pts_new;
				cvtColor(orig, nextimg, CV_BGR2GRAY);
				if (orig_pts.size() == 0)
				{
					previmg = nextimg.clone();
					goodFeaturesToTrack(nextimg, prev_pts, 10, 0.05, 0.2, mask);
					orig_pts = prev_pts;
				}
				else
				{
					if (prev_pts.size() > 0 && !previmg.empty())
					{
						calcOpticalFlowPyrLK(previmg, nextimg, prev_pts, next_pts, m_status, m_error);
					}
					for (int i = 0; i < m_status.size(); i++)
					{
						int j = 1;
						if (m_status[i])
						{
							tracked_pts.push_back(next_pts[i]);
							circle(output, next_pts[i], 5, Scalar(255));
							orig_pts_new.push_back(orig_pts[i]);
						}
					}

					orig_pts = orig_pts_new;
					prev_pts = tracked_pts;
					nextimg.copyTo(previmg);
				}
			}
			dh_out->WriteFrame(output);
#pragma endregion

		}
		TimerElapsed();
		yieldCurrentThread();
	}
}

ProcessingThread::~ProcessingThread()
{
}
