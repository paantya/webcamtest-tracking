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
	cv::Mat orig, grey, grey_orig, output, previmg, nextimg, sample;
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

	cv::SurfFeatureDetector detector(4000);
	cv::SurfDescriptorExtractor extractor;
	std::vector<cv::KeyPoint> keypoints_sample, keypoints_scene;
	cv::Mat descriptors_sample, descriptors_scene;
	cv::BFMatcher matcher;
	std::vector<cv::DMatch> matches;

	cv::cvtColor(sample, sample, CV_BGR2GRAY);
	cv::equalizeHist(sample, sample);

	detector.detect(sample, keypoints_sample);
	extractor.compute(sample, keypoints_sample, descriptors_sample);
	std::vector<std::vector<cv::Point>> contours, contours_scene;
	std::vector<cv::Vec4i> hierarchy, hierarchy_scene;
	cv::findContours(sample, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_TC89_KCOS);

	int thresh = 10;
	while (running)
	{
		std::vector<unsigned char> m_status;
		std::vector<float>         m_error;
		std::vector<cv::Point2f> trackedPts;
		if (vc->isOpened()){
			if (vc->read(orig)){

				cv::cvtColor(orig, grey_orig, CV_BGR2GRAY);
				grey = grey_orig.clone();
				output = orig.clone();
				cv::equalizeHist(grey_orig, nextimg);
				
				if (prev_pts.size() > 0 && !previmg.empty())
				{
					cv::calcOpticalFlowPyrLK(previmg, nextimg, prev_pts, next_pts, m_status, m_error);
				}
				cv::Mat newimg;
				
				cv::Point2f mid_tracked, mid_orig;
				std::vector<cv::Point2f> orig_pts_new;

				if (orig_pts.size() > 0){

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
				}

				orig_pts = orig_pts_new;

				cv::line(output, mid_orig, mid_tracked, cv::Scalar(0, 255, 255), 4);

				cv::circle(output, cv::Point(grey.size().width / 2, grey.size().height / 2), 100, cv::Scalar(25, 255, 25));

				imshow("Output", output);
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
