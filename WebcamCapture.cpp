#include "WebcamCapture.h"

cv::Mat CenterMask(cv::Size size, int radius = 100)
{
	cv::Mat mask(size, CV_8UC1);
	mask.setTo(cv::Scalar::all(0));
	cv::Point centre(size.width / 2, size.height / 2);
	cv::Scalar white(255, 255, 255);
	cv::circle(mask, centre, radius, white);
	cv::floodFill(mask, centre, white);
	return mask;
}

void WebcamCapture::run()
{
	cv::Mat orig, grey, grey_orig, output, previmg, nextimg, sample, color_filtered;
	sample = cv::imread("krest2.bmp");
	bool detected = false;
	std::vector<cv::Point2f> prev_pts;
	std::vector<cv::Point2f> next_pts;
	std::vector<cv::Point2f> orig_pts;
	vc->read(orig);
	cv::cvtColor(orig, grey_orig, CV_BGR2GRAY);
	cv::cvtColor(orig, orig, CV_BGR2GRAY);
	cv::equalizeHist(grey_orig, grey);
	previmg = grey_orig.clone();
	cv::Mat mask = CenterMask(grey.size());
	cv::goodFeaturesToTrack(grey_orig, prev_pts, 100, 0.05, 0.2, mask);
	orig_pts = prev_pts;

	cv::Mat transform;

	cv::cvtColor(sample, sample, CV_BGR2GRAY);
	cv::equalizeHist(sample, sample);

	std::vector<std::vector<cv::Point>> contours, contours_scene;
	std::vector<cv::Vec4i> hierarchy, hierarchy_scene;

	int iLowH = 47;
	int iHighH = 86;

	int iLowS = 43;
	int iHighS = 255;

	int iLowV = 43;
	int iHighV = 255;


	while (running)
	{
		std::vector<unsigned char> m_status;
		std::vector<float>         m_error;
		std::vector<cv::Point2f> trackedPts;
		if (vc->isOpened()){
			if (vc->read(orig)){

				cv::cvtColor(orig, grey_orig, CV_BGR2GRAY);
				cv::cvtColor(orig, color_filtered, CV_BGR2HSV);
				output = orig.clone();

				cv::inRange(color_filtered, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), color_filtered);

				// filtering
				// filter out small white noise
				cv::erode(color_filtered, color_filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
				cv::dilate(color_filtered, color_filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

				// filter out small black noise
				cv::dilate(color_filtered, color_filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
				cv::erode(color_filtered, color_filtered, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

				//cv::Canny(color_filtered, color_filtered, 50, 200);

#pragma region OpticalFlow
				if (useOpticalFlow){
					nextimg = color_filtered;

					if (prev_pts.size() > 0 && !previmg.empty())
					{
						cv::calcOpticalFlowPyrLK(previmg, nextimg, prev_pts, next_pts, m_status, m_error);
					}
					cv::Mat newimg;
					cv::Point2f mid_tracked, mid_orig;
					std::vector<cv::Point2f> orig_pts_new;

					if (orig_pts.size() == 0)
					{
						previmg = color_filtered.clone();
						cv::goodFeaturesToTrack(color_filtered, prev_pts, 100, 0.05, 0.2);
						orig_pts = prev_pts;
					}
					else
					{

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

						cv::circle(output, mid_tracked, 5, cv::Scalar(0, 0, 255));
						cv::circle(output, mid_orig, 5, cv::Scalar(255, 0, 0));
						orig_pts = orig_pts_new;
						cv::line(output, mid_orig, mid_tracked, cv::Scalar(0, 255, 255), 4);
					}
				}
#pragma endregion

				std::vector<cv::Vec4i> lines;
				std::vector<cv::Point2f> line_ends;
				cv::HoughLinesP(color_filtered, lines, 5, CV_PI / 180, 50, 10, 20);
				for each(cv::Vec4i line in lines)
				{
					cv::line(output, cv::Point(line.val[0], line.val[1]), cv::Point(line.val[2], line.val[3]), cv::Scalar(0, 0, 255));
					line_ends.push_back(cv::Point2f(line.val[0], line.val[1]));
					line_ends.push_back(cv::Point2f(line.val[2], line.val[3]));
				}

				cv::Mat bestLabels;

				// cross skew angle determination (THIS SEEMS TO WORK)
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
				}

				// CONTROL WINDOW - to be replaced or moved to a separate thread

				//Create trackbars in "Control" window
				cv::createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
				cv::createTrackbar("HighH", "Control", &iHighH, 255);

				cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
				cv::createTrackbar("HighS", "Control", &iHighS, 255);

				cv::createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
				cv::createTrackbar("HighV", "Control", &iHighV, 255);

				cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
				imshow("Output", output);
				imshow("HSV", color_filtered);
				int key = cv::waitKey(1);

				nextimg.copyTo(previmg);
				if (trackedPts.size() > 0)
					prev_pts = trackedPts;
				detected = true;
				if (key == 27)
					break;
				if (key == 32) {
					previmg = grey_orig.clone();
					cv::goodFeaturesToTrack(grey_orig, prev_pts, 100, 0.05, 0.2, mask);
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
	this->vc = new cv::VideoCapture(0);
	vc->open(0); // open default device
	this->dh = nullptr;
	running = true;;
}


WebcamCapture::~WebcamCapture()
{
	vc->release();
}
