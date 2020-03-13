#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
cv::Vec3d  LinearLSTriangulation(cv::Point3d u,       //homogenous image point (u,v,1)
		cv::Mat P,       //camera 1 projection matrix
		cv::Point3d u1,      //homogenous image point in 2nd camera
		cv::Mat P1       //camera 2 projection matrix
		) {
	double data1[12] = { u.x * P.at<double>(2, 0) - P.at<double>(0, 0), u.x
			* P.at<double>(2, 1) - P.at<double>(0, 1), u.x * P.at<double>(2, 2)
			- P.at<double>(0, 2), u.y * P.at<double>(2, 0) - P.at<double>(1, 0),
			u.y * P.at<double>(2, 1) - P.at<double>(1, 1), u.y
					* P.at<double>(2, 2) - P.at<double>(1, 2), u1.x
					* P1.at<double>(2, 0) - P1.at<double>(0, 0), u1.x
					* P1.at<double>(2, 1) - P1.at<double>(0, 1), u1.x
					* P1.at<double>(2, 2) - P1.at<double>(0, 2), u1.y
					* P1.at<double>(2, 0) - P1.at<double>(1, 0), u1.y
					* P1.at<double>(2, 1) - P1.at<double>(1, 1), u1.y
					* P1.at<double>(2, 2) - P1.at<double>(1, 2) };

	cv::Mat A = cv::Mat(4, 3, CV_64FC1, &data1);
	double data2[4] = { -(u.x * P.at<double>(2, 3) - P.at<double>(0, 3)), -(u.y
			* P.at<double>(2, 3) - P.at<double>(1, 3)), -(u1.x
			* P1.at<double>(2, 3) - P1.at<double>(0, 3)), -(u1.y
			* P1.at<double>(2, 3) - P1.at<double>(1, 3)) };
	cv::Mat B = cv::Mat(4, 1, CV_64FC1, &data2);
	cv::Vec3d X;
	cv::solve(A, B, X, cv::DECOMP_SVD);
	return X;
}

int triangulate() {
	int board_w;
	int board_h;
	int cam;

	printf("Triangulation Test\n");
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;
	vector<Point2f> corners;

	printf("Enter number of corners along width: ");
	scanf("%d", &board_w);
	printf("Enter number of corners along height: ");
	scanf("%d", &board_h);
	printf("Pick a Camera: ");
	scanf("%d", &cam);

	Size board_sz = Size(board_w, board_h);
	vector<vector<Point3f> > objectPoints; // real world location
	vector<vector<Point2f> > imagePoints1, imagePoints2; // corners
	vector<Point2f> corners1, corners2;
	vector<vector<Point2f> > left_img_points, right_img_points;

	Mat img, gray, gray1, gray2;

	VideoCapture cap = VideoCapture(cam);
	double dWidth = cap.get(CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	cout << "Resolution of the video 1 : " << dWidth << " x " << dHeight
			<< endl;

	bool found1 = false, found2 = false;
	while (true) {
		cap >> img;
		cvtColor(img, gray, COLOR_BGR2GRAY);
		gray1 = gray(cv::Rect(0, 0, dWidth/2, dHeight));
		gray2 = gray(cv::Rect(dWidth/2, 0, dWidth/2, dHeight));

		found1 = findChessboardCornersSB(gray1, board_sz, corners1,
				CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_ACCURACY
						| CALIB_CB_EXHAUSTIVE);

		found2 = findChessboardCornersSB(gray2, board_sz, corners2,
				CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_ACCURACY
						| CALIB_CB_EXHAUSTIVE);

		if (found1) {
			cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1),
					TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30,
							1e-5));
			drawChessboardCorners(gray1, board_sz, corners1, found1);
		}

		if (found2) {
			cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1),
					TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 30,
							1e-5));
			drawChessboardCorners(gray2, board_sz, corners2, found2);
		}

		imshow("image1", gray1);
		imshow("image2", gray2);

		if ((found1 && found2))
			cout << "Found" << endl;

		if (waitKey(1) == 27 || (found1 && found2)) {
			break;
		}
	}

	cv::FileStorage fs("mystereocalib_2.yml", FileStorage::READ);
	cv::Mat R1, R2, P1, P2, Q;
	Mat CM1 = Mat(3, 3, CV_64FC1);
	Mat CM2 = Mat(3, 3, CV_64FC1);
	Mat D1, D2;
	Mat R, T, E, F;

	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q;
	fs["M1"] >> CM1;
	fs["M2"] >> CM2;
	fs["D1"] >> D1;
	fs["D2"] >> D2;

	Mat points4D;

	cv::triangulatePoints(P1, P2, corners2, corners1, points4D);
	for (int j = 0; j < points4D.cols; j++) {
		cv::Vec4d triangCoords = points4D.col(j);
		cv::Vec3d Coords3D;
		for (unsigned int i = 0; i < 3; i++) {
			Coords3D[i] = triangCoords[i] / triangCoords[3];
		}
		cout << "Point " << j + 1 << ": " << Coords3D << endl;
	}

	cout << "Using Linear LS Trig: "
			<< LinearLSTriangulation(Point3d(147, 219, 1), P1,
					Point3d(489, 167, 1), P2) << endl;

	cv::waitKey(0);
	cv::destroyAllWindows();
	return (0);
}


