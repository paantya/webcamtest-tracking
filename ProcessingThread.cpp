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
			cvtColor(orig, nextimg, CV_BGR2GRAY);
			output = orig.clone();

			bool useOpticalFlow = true;

#pragma region OpticalFlow
			if (useOpticalFlow){
				Mat newimg;
				//Point2f mid_tracked, mid_orig;
				vector<Point2f> orig_pts_new;
				
				if (orig_pts.size() == 0)
				{
					previmg = nextimg;
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
						if (m_status[i])
						{
							tracked_pts.push_back(next_pts[i]);
							orig_pts_new.push_back(orig_pts[i]);
							//circle(output, next_pts[i], 5, Scalar(255, 0, 0));
						}
					}

					orig_pts = orig_pts_new;
					prev_pts = tracked_pts;
					previmg = nextimg.clone();
					for each (Point2f pt in tracked_pts)
					{
						circle(output, pt, 5, Scalar(255, 0, 0));
					}
					/*
					if (orig_pts.size() > 5)
					{
					Mat rvec, tvec;
					vector<Point3d> framePoints;
					framePoints.push_back(Point3d(0, 0, 0));
					framePoints.push_back(Point3d(0.0001, 0, 0));
					framePoints.push_back(Point3d(0, 0.0001, 0));
					framePoints.push_back(Point3d(0, 0, 0.0001));
					Mat homo = findHomography(orig_pts, tracked_pts, CV_RANSAC);
					if (!homo.empty() && homo.data)
					{
					//warpPerspective(output, output, homo.inv(),output.size());
					cameraPoseFromHomography(homo, rvec, tvec);
					Mat rodrigues;
					//cv::setIdentity(rvec, 1);
					//cv::normalize(rvec, rvec);
					//Rodrigues(rvec, rodrigues);
					cv::normalize(tvec, tvec);
					projectPoints(framePoints, rvec, tvec0, intrinsics, distortion, imagePoints);
					//perspectiveTransform(framePoints, framePoints, homo);
					line(output, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
					line(output, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
					line(output, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
					printf("%f %f %f\n", tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
					}
					vector<Point3f> orig3d, next3d;
					for (int i = 0; i < orig_pts.size(); i++)
					{
					orig3d.push_back(Point3f(orig_pts[i].x, orig_pts[i].y, 0));
					next3d.push_back(Point3f(trackedPts[i].x, trackedPts[i].y, 0));
					}

					vector<Point3f> framePoints;
					framePoints.push_back(Point3f(0.0, 0.0, 0.0));
					framePoints.push_back(Point3f(1.0, 0.0, 0.0));
					framePoints.push_back(Point3f(0.0, 1.0, 0.0));
					framePoints.push_back(Point3f(0.0, 0.0, 1.0));

					solvePnP(orig3d, trackedPts, intrinsics, distortion, rvec, tvec);
					projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints);
					cv::line(output, imageFramePoints[0], imageFramePoints[1], Scalar(0, 0, 255));
					cv::line(output, imageFramePoints[0], imageFramePoints[2], Scalar(0, 255, 0));
					cv::line(output, imageFramePoints[0], imageFramePoints[3], Scalar(255, 0, 0));
					}*/

				}
			}
			dh_out->WriteFrame(output);
#pragma endregion

		}
		yieldCurrentThread();
		TimerElapsed();
	}
}

ProcessingThread::~ProcessingThread()
{
}
