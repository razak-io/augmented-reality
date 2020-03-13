#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

int stereo_calib() {
	int numBoards = 0;
	int board_w;
	int board_h;
	float sqSize = 0;
	char filename[20];
	int cam = 0;

	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;
	vector<Point2f> corners;

	printf("Enter number of corners along width: ");
	scanf("%d", &board_w);

	printf("Enter number of corners along height: ");
	scanf("%d", &board_h);

	printf("Enter the size of each square: ");
	scanf("%f", &sqSize);

	printf("Enter number of boards: ");
	scanf("%d", &numBoards);

	printf("Pick a Camera: ");
	scanf("%d", &cam);

	printf("Enter the filename: ");
	scanf("%19s", filename);

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
	int success = 0;
	int k = 0;
	bool found1 = false, found2 = false;

	cout << "Press 'c' to capture image" << dHeight << endl;
	while (success < numBoards) {
		cap >> img;
		cvtColor(img, gray, COLOR_BGR2GRAY);
		gray1 = gray(cv::Rect(0, 0, dWidth / 2, dHeight));
		gray2 = gray(cv::Rect(dWidth / 2, 0, dWidth / 2, dHeight));

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

		imshow("right", gray1);
		imshow("left", gray2);

		if (found1 && found2) {
			k = waitKey(1);
		}

		vector<Point3f> obj;
		for (int j = 0; j < board_h; j++) {
			for (int i = 0; i < board_w; i++) {
				obj.push_back(
						Point3f((float) i * sqSize, (float) j * sqSize, 0));
			}
		}

		if ((k == 'c') && (found1 && found2)) {
			imagePoints1.push_back(corners1); //right
			imagePoints2.push_back(corners2); //left
			objectPoints.push_back(obj);
			printf("Corners stored\n");
			success++;
			if (success >= numBoards) {
				break;
			}
		}
	}

	destroyAllWindows();
	printf("Starting Calibration\n");
	Mat CM1, CM2, D1, D2;
	Mat R, T, E, F;
	FileStorage fsl("left.yml", FileStorage::READ);
	FileStorage fsr("right.yml", FileStorage::READ);

	fsl["camera matrix"] >> CM1;
	fsr["camera matrix"] >> CM2;

	cout << "left camera_matrix: " << CM1 << endl;
	cout << "right camera_matrix: " << CM2 << endl;

	fsl["distortion coefficient"] >> D1;
	fsr["distortion coefficient"] >> D2;

	cout << "left distortion_coefficients: " << D1 << endl;
	cout << "right distortion_coefficients: " << D2 << endl;

	double rms = stereoCalibrate(objectPoints, imagePoints2, imagePoints1, CM1,
			D1, CM2, D2, gray1.size(), R, T, E, F,
			CALIB_FIX_ASPECT_RATIO + CALIB_FIX_PRINCIPAL_POINT
					+ CALIB_ZERO_TANGENT_DIST + CALIB_USE_INTRINSIC_GUESS
					+ CALIB_SAME_FOCAL_LENGTH + CALIB_RATIONAL_MODEL
					+ CALIB_FIX_K4 + CALIB_FIX_K5 + CALIB_FIX_K6,
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];
	stereoRectify(CM1, D1, CM2, D2, gray1.size(), R, T, R1, R2, P1, P2, Q, 0,
			-1, gray1.size(), &validRoi[0], &validRoi[1]);

	FileStorage fs1(filename, FileStorage::WRITE);
	fs1 << "M1" << CM1;
	fs1 << "M2" << CM2;
	fs1 << "D1" << D1;
	fs1 << "D2" << D2;
	fs1 << "R" << R;
	fs1 << "T" << T;
	fs1 << "E" << E;
	fs1 << "F" << F;
	fs1 << "R1" << R1;
	fs1 << "R2" << R2;
	fs1 << "P1" << P1;
	fs1 << "P2" << P2;
	fs1 << "Q" << Q;

	cout << "Done with RMS error=" << rms << endl;

	return(0);
}

