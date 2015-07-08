#include "WebcamCapture.h"

Mat CenterMask(Size size, int radius = 100)
{
	Mat mask(size, CV_8UC1);
	mask.setTo(Scalar::all(0));
	Point centre(size.width / 2, size.height / 2);
	Scalar white(255, 255, 255);
	circle(mask, centre, radius, white);
	floodFill(mask, centre, white);
	return mask;
}
void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
	pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
	float norm1 = (float)norm(H.col(0));
	float norm2 = (float)norm(H.col(1));
	float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

	Mat p1 = H.col(0);       // Pointer to first column of H
	Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)

	cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose

	p1 = H.col(1);           // Pointer to second column of H
	p2 = pose.col(1);        // Pointer to second column of pose (empty)

	cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose

	p1 = pose.col(0);
	p2 = pose.col(1);

	Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
	Mat c2 = pose.col(2);    // Pointer to third column of pose
	p3.copyTo(c2);       // Third column is the crossproduct of columns one and two

	pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
}
void WebcamCapture::run()
{
	Mat orig, grey, grey_orig, output, previmg, nextimg, color_filtered;
	//sample = imread("krest2.bmp");
	bool detected = false;
	vector<Point2f> prev_pts;
	vector<Point2f> next_pts;
	vector<Point2f> orig_pts;
	vc->read(orig);
	cvtColor(orig, grey_orig, CV_BGR2GRAY);
	//cvtColor(orig, orig, CV_BGR2GRAY);
	equalizeHist(grey_orig, grey);
	previmg = grey_orig.clone();
	Mat mask = CenterMask(grey.size());
	//goodFeaturesToTrack(grey_orig, prev_pts, 100, 0.05, 0.2, mask);
	orig_pts = prev_pts;
	Mat transform;

	//cvtColor(sample, sample, CV_BGR2GRAY);
	//equalizeHist(sample, sample);

	vector<vector<Point>> contours, contours_scene;
	vector<Vec4i> hierarchy, hierarchy_scene;

	int iLowH = 47;
	int iHighH = 86;

	int iLowS = 43;
	int iHighS = 255;

	int iLowV = 43;
	int iHighV = 255;
	/* 3d */
	//vector<Point2f> framePoints, framePoints2, crossPoints;
	vector<Point2f> imagePoints, imageFramePoints, imageOrigin;


	FileStorage fs;
	fs.open("out_camera_data.yml", FileStorage::READ);
	//Mat rvec = Mat(Size(3, 1), CV_64F);
	//Mat tvec = Mat(Size(3, 1), CV_64F);
	Mat intrinsics, distortion;
	fs["Camera_Matrix"] >> intrinsics;
	fs["Distortion_Coefficients"] >> distortion;
	/* 3d end */
	while (running)
	{
		vector<unsigned char> m_status;
		vector<float>         m_error;
		vector<Point2f> trackedPts;
		if (vc->isOpened()){
			if (vc->read(orig)){
				
				cvtColor(orig, grey_orig, CV_BGR2GRAY);
				cvtColor(orig, color_filtered, CV_BGR2HLS);
				output = orig.clone();

				inRange(color_filtered, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), color_filtered);

				// filtering
				// filter out small white noise
				erode(color_filtered, color_filtered, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				dilate(color_filtered, color_filtered, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

				// filter out small black noise
				//dilate(color_filtered, color_filtered, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//erode(color_filtered, color_filtered, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

				//Canny(color_filtered, color_filtered, 50, 200);

#pragma region OpticalFlow
				if (useOpticalFlow){
					//nextimg = color_filtered;
					nextimg = grey_orig.clone();

					if (prev_pts.size() > 0 && !previmg.empty())
					{
						calcOpticalFlowPyrLK(previmg, nextimg, prev_pts, next_pts, m_status, m_error);

					}
					Mat newimg;
					Point2f mid_tracked, mid_orig;
					vector<Point2f> orig_pts_new;

					if (orig_pts.size() == 0)
					{
						previmg = grey_orig.clone();
						goodFeaturesToTrack(nextimg, prev_pts, 10, 0.05, 0.2, mask);
						orig_pts = prev_pts;
					}
					else
					{
						for each (Point2f pt in prev_pts)
							circle(output, pt, 5, Scalar(255, 0, 0));

						for (int i = 0; i < m_status.size(); i++)
						{
							if (m_status[i])
							{
								trackedPts.push_back(next_pts[i]);
								orig_pts_new.push_back(orig_pts[i]);
								mid_tracked += next_pts[i];
								mid_orig += orig_pts[i];
							}
						}

						mid_tracked.x /= trackedPts.size();
						mid_tracked.y /= trackedPts.size();

						mid_orig.x /= orig_pts_new.size();
						mid_orig.y /= orig_pts_new.size();

						circle(output, mid_tracked, 5, Scalar(0, 0, 255));
						circle(output, mid_orig, 5, Scalar(255, 0, 0));
						orig_pts = orig_pts_new;

						if (orig_pts.size() > 5)
						{
							Mat rvec, tvec;
							vector<Point2f> framePoints;
							framePoints.push_back(mid_orig);
							framePoints.push_back(mid_orig + Point2f(35.0, 0.0));
							framePoints.push_back(mid_orig + Point2f(0.0, 35.0));
							Mat homo = findHomography(orig_pts, trackedPts, CV_RANSAC);
							//projectPoints(framePoints, framePoints, tvec, intrinsics, tvec, imagePoints);

							perspectiveTransform(framePoints, framePoints, homo);
							line(output, framePoints[0], framePoints[1], Scalar(0, 0, 255), 3);
							line(output, framePoints[0], framePoints[2], Scalar(255, 0, 0), 3);
							warpPerspective(output, output, homo.inv(),output.size());
							/*
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
							cv::line(output, imageFramePoints[0], imageFramePoints[3], Scalar(255, 0, 0));*/
						}

						line(output, mid_orig, mid_tracked, Scalar(0, 255, 255), 4);
						circle(output, Point(output.size().width / 2, output.size().height / 2), 100, Scalar(0, 0, 0));
					}
				}
#pragma endregion
				/*
				vector<Vec4i> lines;
				vector<Point2f> line_ends;
				//HoughLinesP(color_filtered, lines, 5, CV_PI / 180, 50, 10, 20);

				for each(Vec4i line in lines)
				{
				cv::line(output, Point(line.val[0], line.val[1]), Point(line.val[2], line.val[3]), Scalar(0, 0, 255));
				line_ends.push_back(Point2f(line.val[0], line.val[1]));
				line_ends.push_back(Point2f(line.val[2], line.val[3]));
				}
				//vector<Point2f> bestlabels;
				//Mat bestlabels;
				//if (lines.size() > 2)
				//kmeans(line_ends, 2, bestlabels, TermCriteria(CV_TERMCRIT_EPS && CV_TERMCRIT_ITER, 5, 1), 5, 0);

				//vector<double> angles;


				//projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints);

				// cross rotation angle determination (THIS SEEMS TO WORK)
				for (int i = 0; i < lines.size(); i++)
				{
				int x1 = lines[i].val[2] - lines[i].val[0];
				int y1 = lines[i].val[3] - lines[i].val[1];
				double length1 = sqrt(x1*x1 + y1*y1);
				for (int j = 0; j < lines.size(); j++)
				{
				int x2 = lines[j].val[2] - lines[j].val[0];
				int y2 = lines[j].val[3] - lines[j].val[1];
				double length2 = sqrt(x2*x2 + y2*y2);
				double scalarprod = x1*x2 + y1*y2;
				double theta = acos(scalarprod / (length1*length2));
				if (theta > CV_PI / 4 && theta < CV_PI)
				{
				printf("found cross with angle %f\n", theta);
				break;
				}
				}
				//double theta = acos(y1 / (length1));
				//angles.push_back(theta);
				}
				*/
				Mat dst = Mat::zeros(nextimg.size(), CV_32FC1);
				Mat dst_norm, dst_norm_scaled;
				int blockSize = 2;
				int apertureSize = 3;
				double k = 0.04;
				int thresh = 220;
				//cornerHarris(color_filtered, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
				/// Normalizing
				normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
				convertScaleAbs(dst_norm, dst_norm_scaled);


				/// Drawing a circle around corners
				for (int j = 0; j < dst_norm.rows; j++)
				{
					for (int i = 0; i < dst_norm.cols; i++)
					{
						if ((int)dst_norm.at<float>(j, i) > thresh)
						{
							circle(output, Point(i, j), 5, Scalar(0, 0, 25), 2, 8, 0);
						}
					}
				}

				// CONTROL WINDOW - to be replaced or moved to a separate thread

				//Create trackbars in "Control" window
				createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
				createTrackbar("HighH", "Control", &iHighH, 255);

				createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
				createTrackbar("HighS", "Control", &iHighS, 255);

				createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
				createTrackbar("HighV", "Control", &iHighV, 255);

				namedWindow("Control", CV_WINDOW_AUTOSIZE);
				imshow("Output", output);
				//imshow("HSV", color_filtered);
				int key = waitKey(1);

				nextimg.copyTo(previmg);
				if (trackedPts.size() > 0)
					prev_pts = trackedPts;
				detected = true;
				if (key == 27)
					break;
				if (key == 32) {

					previmg = grey_orig.clone();
					goodFeaturesToTrack(nextimg, prev_pts, 10, 0.05, 0.2, mask);
					orig_pts = prev_pts;

				}
				if (key != -1)
					printf("%i", key);
			}
		}
	}
	vc->release();

	this->exit(0);
}

void WebcamCapture::stop()
{
	running = false;
}

WebcamCapture::WebcamCapture()
{
	// init our capture
	this->vc = new VideoCapture;
	vc->open(0); // open default device
	this->dh = nullptr;
	running = true;;
}


WebcamCapture::~WebcamCapture()
{
	vc->release();
}
