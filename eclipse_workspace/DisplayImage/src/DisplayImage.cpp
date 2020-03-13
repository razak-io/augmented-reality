#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void img_contrast(Mat image);
void img_equalize(Mat image, bool gry);
void capnsave();
void capnsave(int cam);
void split();

int main(int argc, char **argv) {
	//capnsave(0);
	//split();
	capnsave();
	return 0;
}

//char key = (char)waitKey(capture.isOpened() ? 50 : 500);
//putText(result, "Differencing the two images.", cvPoint(30,30),
//    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

void capnsave(int cam) {
	VideoCapture cam_1(cam);
	// if not success, exit program
	if (cam_1.isOpened() == false) { // || cam_2.isOpened() == false) {
		cout << "Cannot open the video camera" << endl;
		cin.get(); //wait for any key press
		return;
	}
    //cam_1.set(cv::CAP_PROP_FRAME_WIDTH, 3040);
    //cam_1.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cam_1.set(cv::CAP_PROP_FPS, 60);

	int i = 10;
	Mat frame1, frame2;

	while (i > 0) {
		bool bSuccess = cam_1.read(frame1);

		if (bSuccess == false) {
			cout << "Video camera is disconnected" << endl;
			cin.get(); //Wait for any key press
			return;
		}

		imshow(("Full Frame"), frame1);
		if (waitKey(10) == 27) {
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}

		Size patternsize(8, 5); //interior number of corners
		Mat gray;

		cv::cvtColor(frame1, gray, COLOR_BGR2GRAY); //source image
		vector<Point2f> lcorners, rcorners; //this will be filled by the detected corners

		Mat rgray = gray(Rect(0, 0, 640, 480));
		Mat lgray = gray(Rect(640, 0, 640, 480));

		bool lpatternfound = findChessboardCorners(lgray, patternsize, lcorners,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

		bool rpatternfound = findChessboardCorners(rgray, patternsize, rcorners,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);

		if (lpatternfound && rpatternfound) {
			cout << "Press c to capture and s to skip" << endl;
			char key = (char) waitKey(cam_1.isOpened() ? 50 : 500);
			if (key == 'c') {
				Mat rightframe = frame1(Rect(0, 0, 640, 480));
				Mat leftframe = frame1(Rect(640, 0, 640, 480));

				std::ostringstream stringStream;
				stringStream << "pic_" << i << ".jpg";
				String left = stringStream.str();

				bool isSuccess = imwrite(left, frame1);
				if (isSuccess == false) {
					cout << "Failed to save " << left << endl;
					cin.get();
					return;
				}
				cout << left << " is succusfully saved to a file" << endl;
				i--;
			} else
				continue;
		}
	}

	return;
}

void split() {
	for (int i = 1; i < 11; i++) {
		std::ostringstream stringStream, lstringStream, rstringStream;
		stringStream << "pic_" << i << ".jpg";
		String filename = stringStream.str();
		Mat tosplit = imread(filename);

		Mat rframe = tosplit(Rect(0, 0, 640, 480));
		Mat lframe = tosplit(Rect(640, 0, 640, 480));

		lstringStream << "left" << i << ".jpg";
		rstringStream << "right" << i << ".jpg";

		String left = lstringStream.str();
		String right = rstringStream.str();

		bool isSuccess = imwrite(left, lframe);
		if (isSuccess == false) {
			cout << "Failed to save " << left << endl;
			cin.get();
			return;
		}

		isSuccess = imwrite(right, rframe);
		if (isSuccess == false) {
			cout << "Failed to save " << right << endl;
			cin.get();
			return;
		}
	}
	cout << "Split and Save Successful " << left << endl;
}

void capnsave() {
	VideoCapture cam_1(0);
	//VideoCapture cam_2(1);

	// if not success, exit program
	if (cam_1.isOpened() == false) {	// || cam_2.isOpened() == false) {
		cout << "Cannot open the video camera" << endl;
		cin.get(); //wait for any key press
		return;
	}

	int i = 10;
	Mat frame1, frame2;

	while (true) {
		bool bSuccess = cam_1.read(frame1); // read a new frame from video
		//bool bSuccess2 = cam_2.read(frame2); // read a new frame from video

		if (bSuccess == false) { //|| bSuccess2 == false) {
			cout << "Video camera is disconnected" << endl;
			cin.get(); //Wait for any key press
			return;
		}

		Size patternsize(7, 7); //interior number of corners
		Mat gray;

		cv::cvtColor(frame1, gray, COLOR_BGR2GRAY); //source image
		vector<Point2f> lcorners, rcorners; //this will be filled by the detected corners

		Mat lgray = gray(Rect(0, 0, 640, 480));
		Mat rgray = gray(Rect(640, 0, 640, 480));
		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
//		bool lpatternfound = findChessboardCorners(lgray, patternsize, lcorners,
//				CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
//						+ CALIB_CB_FAST_CHECK);
//
//		bool rpatternfound = findChessboardCorners(rgray, patternsize, rcorners,
//				CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
//						+ CALIB_CB_FAST_CHECK);
//
//		drawChessboardCorners(frame1, patternsize, Mat(lcorners),
//				lpatternfound);
//		drawChessboardCorners(frame1, patternsize, Mat(rcorners),
//				rpatternfound);

		imshow("Pre-Calibrate", frame1);

		if (waitKey(10) == 27) {
			cout << "Esc key is pressed by user. Calibration Starting......"
					<< endl;
			break;
		}
	}

	i += 4;
	i = 1;

	while (i > 0) {
		bool bSuccess = cam_1.read(frame1); // read a new frame from video
		//bool bSuccess2 = cam_2.read(frame2); // read a new frame from video

		if (bSuccess == false) { //|| bSuccess2 == false) {
			cout << "Video camera is disconnected" << endl;
			cin.get(); //Wait for any key press
			return;
		}

		Size patternsize(7, 7); //interior number of corners
		Mat gray;

		cv::cvtColor(frame1, gray, COLOR_BGR2GRAY); //source image
		vector<Point2f> lcorners, rcorners; //this will be filled by the detected corners

		Mat rgray = gray(Rect(0, 0, 640, 480));
		Mat lgray = gray(Rect(640, 0, 640, 480));
		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool lpatternfound = findChessboardCorners(lgray, patternsize, lcorners,
				CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

		bool rpatternfound = findChessboardCorners(rgray, patternsize, rcorners,
				CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

		//if(lpatternfound && rpatternfound){
		Mat rightframe = frame1(Rect(0, 0, 640, 480));
		Mat leftframe = frame1(Rect(640, 0, 640, 480));

		std::ostringstream lstringStream, rstringStream;

		lstringStream << "contr_left" << i+3 << ".jpg";
		rstringStream << "contr_right" << i+3 << ".jpg";

		String left = lstringStream.str();
		String right = rstringStream.str();

		bool isSuccess = imwrite(left,leftframe); //write the image to a file as JPEG
		//bool isSuccess = imwrite("D:/MyImage.png", image); //write the image to a file as PNG
		if (isSuccess == false) {
			cout << "Failed to save " << left << endl;
			cin.get(); //wait for a key press
			return;
		}
		cout << left << " is succusfully saved to a file" << endl;

		isSuccess = imwrite(right, rightframe); //write the image to a file as JPEG
		//bool isSuccess = imwrite("D:/MyImage.png", image); //write the image to a file as PNG
		if (isSuccess == false) {
			cout << "Failed to save " << right << endl;
			cin.get(); //wait for a key press
			return;
		}
		cout << right << " is succusfully saved to a file" << endl;

		i--;
		//}
		//imshow("Calibrating", frame1);
		//waitKey(20);
	}

	imshow(("Full Frame"), frame1);
	waitKey(0);

	/*
	 string window_name1 = "My Camera Feed 1";
	 namedWindow(window_name1); //create a window called "My Camera Feed"

	 string window_name2 = "My Camera Feed 2";
	 namedWindow(window_name2); //create a window called "My Camera Feed"

	 imshow(window_name1, frame1);

	 Mat leftframe = frame2(Rect(0, 0, 640, 480));
	 Mat rightframe = frame2(Rect(640, 0, 640, 480));

	 imshow(window_name2, frame2);


	 */
	return;
}

void img_equalize(Mat image, bool gry) {
	Mat hist_equalized_image;
	if (gry) {
		cvtColor(image, image, COLOR_BGR2GRAY);
		//equalize the histogram
		equalizeHist(image, hist_equalized_image);
	} else {
		cvtColor(image, hist_equalized_image, COLOR_BGR2YCrCb);
		//Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
		vector<Mat> vec_channels;
		split(hist_equalized_image, vec_channels);
		//Equalize the histogram of only the Y channel
		equalizeHist(vec_channels[0], vec_channels[0]);
		//Merge 3 channels in the vector to form the color image in YCrCB color space.
		merge(vec_channels, hist_equalized_image);
		//Convert the histogram equalized image from YCrCb to BGR color space again
		cvtColor(hist_equalized_image, hist_equalized_image, COLOR_YCrCb2BGR);
	}

	//Define names of windows
	String windowNameOfOriginalImage = "Original Image";
	String windowNameOfHistogramEqualized = "Histogram Equalized Image";
	// Create windows with the above names
	namedWindow(windowNameOfOriginalImage, WINDOW_NORMAL);
	namedWindow(windowNameOfHistogramEqualized, WINDOW_NORMAL);
	// Show images inside created windows.
	imshow(windowNameOfOriginalImage, image);
	imshow(windowNameOfHistogramEqualized, hist_equalized_image);
	waitKey(0); // Wait for any keystroke in one of the windows
	destroyAllWindows(); //Destroy all open windows
}

void img_contrast(Mat image) {

	Mat imageContrastHigh2;
	image.convertTo(imageContrastHigh2, -1, 2, 0); //increase the contrast by 2

	Mat imageContrastHigh4;
	image.convertTo(imageContrastHigh4, -1, 4, 0); //increase the contrast by 4

	Mat imageContrastLow0_5;
	image.convertTo(imageContrastLow0_5, -1, 0.5, 0); //decrease the contrast by 0.5

	Mat imageContrastLow0_25;
	image.convertTo(imageContrastLow0_25, -1, 0.25, 0); //decrease the contrast by 0.25

	//Defining window names for above images
	String windowNameOriginalImage = "Original Image";
	String windowNameContrastHigh2 = "Contrast Increased by 2";
	String windowNameContrastHigh4 = "Contrast Increased by 4";
	String windowNameContrastLow0_5 = "Contrast Decreased by 0.5";
	String windowNameContrastLow0_25 = "Contrast Decreased by 0.25";

	//Create and open windows for above images
	namedWindow(windowNameOriginalImage, WINDOW_NORMAL);
	namedWindow(windowNameContrastHigh2, WINDOW_NORMAL);
	namedWindow(windowNameContrastHigh4, WINDOW_NORMAL);
	namedWindow(windowNameContrastLow0_5, WINDOW_NORMAL);
	namedWindow(windowNameContrastLow0_25, WINDOW_NORMAL);

	//Show above images inside the created windows.
	imshow(windowNameOriginalImage, image);
	imshow(windowNameContrastHigh2, imageContrastHigh2);
	imshow(windowNameContrastHigh4, imageContrastHigh4);
	imshow(windowNameContrastLow0_5, imageContrastLow0_5);
	imshow(windowNameContrastLow0_25, imageContrastLow0_25);

	waitKey(0); // Wait for any key stroke

	destroyAllWindows(); //destroy all open windows

}

