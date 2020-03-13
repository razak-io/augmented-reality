#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

bool save_param(std::string filename, cv::Mat intrinsic, cv::Mat distCoeffs,
		std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs,
		cv::Mat prvecs, cv::Mat ptvecs
		);

int cam_calibrate() {
	int numBoards = 0;
	int numCornersHor;
	int numCornersVer;
	int successes = 0;
	float sqSize = 0;
	char filename[20];
	char cam;

	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;
	std::vector<cv::Point2f> corners;

	printf("Enter number of corners along width: ");
	scanf("%d", &numCornersHor);

	printf("Enter number of corners along height: ");
	scanf("%d", &numCornersVer);

	printf("Enter number of boards: ");
	scanf("%d", &numBoards);

	printf("Enter the size of each square: ");
	scanf("%f", &sqSize);

	printf("Pick a Camera (r) for right (l) for left: ");
	scanf ("%s",&cam);

	printf("Enter Filename: ");
	scanf ("%79s",filename);

	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);
	cv::VideoCapture capture = cv::VideoCapture(0);
	if (!capture.isOpened()) {
		cout << "Cannot open the video camera 1" << endl;
		cin.get(); //wait for any key press
		return -1;
	}

	double dWidth = capture.get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	int pick = 0;
	if(cam == 'l')
		pick = dWidth/2;

	cv::Mat image;
	cv::Mat gray_image;
	capture >> image;
	image = image(cv::Rect(pick, 0, dWidth/2, dHeight));

	std::vector<cv::Point3f> obj;
	for( int j = 0; j < numCornersVer; j++ ){
		for( int i = 0; i < numCornersHor; i++ ){
			obj.push_back(cv::Point3f((float)i*sqSize, (float)j*sqSize, 0));
		}
	}

	int key = 0;
	while (successes < numBoards) {
		cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
		bool found = cv::findChessboardCornersSB(image, board_sz, corners,
				cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY
						| cv::CALIB_CB_EXHAUSTIVE);

		if (found) {
			cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30,
							0.1));
			cv::drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		cv::imshow("win1", image);
		cv::imshow("win2", gray_image);

		capture >> image;
		image = image(cv::Rect(pick, 0, dWidth/2, dHeight));

		key = cv::waitKey(1);
		printf("Found Key!\n");
		if (key == 'c' && found) {
			image_points.push_back(corners);
			object_points.push_back(obj);

			printf("Snap stored!");

			successes++;

			if (successes >= numBoards)
				break;
		}

		if (key == 27)
			return 0;
	}

	std::vector<cv::Point3f> newObjPoints;
    newObjPoints = object_points[0];

	cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 4;
	intrinsic.ptr<float>(1)[1] = 3;

	double rms = cv::calibrateCameraRO(object_points, image_points, image.size(),
	(board_sz.width - 1), intrinsic, distCoeffs, rvecs, tvecs, newObjPoints, cv::CALIB_USE_LU);

	cv::Mat ptvec, prvec;
	cv::solvePnP(object_points[0],image_points[0],intrinsic,distCoeffs, prvec,ptvec);

	std::cout << "\nCalibrated with RMS: " << rms << std::endl;
	save_param(filename,intrinsic,distCoeffs, rvecs, tvecs, prvec, ptvec);

	cv::Mat imageUndistorted;

	while (1) {
		capture >> image;
		image = image(cv::Rect(pick, 0, dWidth/2, dHeight));
		cv::undistort(image, imageUndistorted, intrinsic, distCoeffs);
		cv::imshow("win1", image);
		cv::imshow("win2", imageUndistorted);
		if (cv::waitKey(10) == 27) {
			std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
			break;
		}
	}
	cv::destroyAllWindows();
	capture.release();
	return 0;
}

bool save_param(std::string filename, cv::Mat intrinsic, cv::Mat distCoeffs,
		std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs,
		cv::Mat prvecs, cv::Mat ptvecs
		) {
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "camera matrix" << intrinsic;
		fs << "distortion coefficient" << distCoeffs;
		fs << "Translation Vector" << tvecs;
		fs << "Rotational Vector" << rvecs;
		fs << "distortion coefficient" << distCoeffs;
		fs << "Pnp Translation Vector" << ptvecs;
		fs << "Pnp Rotational Vector" << prvecs;
		printf("Parameters saved!\n");
		return true;
	}
	printf("Parameters not saved!\n");
	return false;
}

