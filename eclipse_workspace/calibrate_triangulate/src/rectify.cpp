#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

int rectify() {
	char filename[20];
	int cam;

	printf("Enter the filename: ");
	scanf("%19s", filename);

	printf("Pick a Camera: ");
	scanf("%d", &cam);

	vector<vector<cv::Point3f> > objectPoints; // real world location
	vector<vector<cv::Point2f> > imagePoints1, imagePoints2; // corners
	vector<cv::Point2f> corners1, corners2;
	vector<vector<cv::Point2f> > left_img_points, right_img_points;
	cv::Mat img, gray, gray1, gray2;

	cv::VideoCapture cap = cv::VideoCapture(cam);
	double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	cout << "Resolution of the video 1 : " << dWidth << " x " << dHeight
			<< endl;

	int k = 0;
	cap >> img;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	gray1 = gray(cv::Rect(0, 0, dWidth/2, dHeight));
	gray2 = gray(cv::Rect(dWidth/2, 0, dWidth/2, dHeight));

	printf("Starting Calibration\n");
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	cv::Mat R1, R2, P1, P2, Q;
	cv::Mat CM1 = cv::Mat(3, 3, CV_64FC1);
	cv::Mat CM2 = cv::Mat(3, 3, CV_64FC1);
	cv::Mat D1, D2;
	cv::Mat R, T, E, F;

	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q;
	fs["M1"] >> CM1;
	fs["M2"] >> CM2;
	fs["D1"] >> D1;
	fs["D2"] >> D2;

	printf("Undistort Rectification Map:\n");

	//Precompute maps for cv::remap()
	cv::Mat lmap[2], rmap[2];
	cout << "Projection Vector 1: " << P1 << endl;
	initUndistortRectifyMap(CM1, D1, R1, P1, gray1.size(), CV_16SC2, lmap[0],
			lmap[1]);
	cout << "New Projection Vector 1: " << P1 << endl;
	initUndistortRectifyMap(CM2, D2, R2, P2, gray1.size(), CV_16SC2, rmap[0],
			rmap[1]);

	cv::Mat imgU1, imgU2;
	while (1) {
		cap >> img;
		cv::Mat img1 = img(cv::Rect(0, 0, dWidth/2, dHeight)), img2 = img(
				cv::Rect(dWidth/2, 0, dWidth/2, dHeight));

		cv::remap(img2, imgU1, lmap[0], lmap[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT,
				cv::Scalar());
		cv::remap(img1, imgU2, rmap[0], rmap[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT,
				cv::Scalar());

		cv::Mat disp, greyLeft, greyRight;
		cv::cvtColor(imgU1, greyLeft, cv::COLOR_BGR2GRAY);
		cv::cvtColor(imgU2, greyRight, cv::COLOR_BGR2GRAY);

		Ptr<cv::StereoBM> match = cv::StereoBM::create(16 * 10, 5);
		match->compute(greyLeft, greyRight, disp);
		cv::Mat XYZ(disp.size(), CV_32FC3);
		cv::reprojectImageTo3D(disp, XYZ, Q, true, CV_32F);

		cv::imshow("disp", disp * 5000);
		cv::imshow("ULeft Image", imgU1);
		cv::imshow("URight Image", imgU2);
		cv::imshow("Full Image", img);
		//cv::imshow("depth", XYZ);

		k = cv::waitKey(1);
		if (k == 27) {
			break;
		}
	}
	cv::destroyAllWindows();
	cap.release();
	printf("Done \n");
	return (0);
}
