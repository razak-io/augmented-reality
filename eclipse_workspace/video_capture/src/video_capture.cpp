#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
	//Open the default video camera
	VideoCapture cam_1(0);
	//VideoCapture cam_2(1);
	//VideoCapture cam_3(2);

	// if not success, exit program
	if (cam_1.isOpened() == false) {
		cout << "Cannot open the video camera 1" << endl;
		cin.get(); //wait for any key press
		return -1;
	}

	int offset_x = 100;
	int offset_y = 50;

	//cam_1.set(cv::CAP_PROP_FRAME_WIDTH, 1520);
	//cam_1.set(cv::CAP_PROP_FRAME_WIDTH, 1080);
	//cam_1.set(cv::CAP_PROP_FRAME_HEIGHT, 540);
	//cam_1.set(cv::CAP_PROP_FPS, 60);

	double dWidth = cam_1.get(CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cam_1.get(CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	cout << "Resolution of the video 1 : " << dWidth << " x " << dHeight
			<< endl;

	string window_name1 = "My Camera Feed 1";
	namedWindow(window_name1); //create a window called "My Camera Feed"

	while (true) {
		Mat frame1;
		bool bSuccess = cam_1.read(frame1); // read a new frame from video
		cv::Rect roi;
		roi.x = offset_x;
		roi.y = offset_y;

		/* Crop the original image to the defined ROI */
		//Mat rightframe = frame1(Rect(0, 0, 640, 1080));
		//Mat leftframe = frame1(Rect(1520, 0, 1520, 1080));
		//Breaking the while loop if the frames cannot be captured
		if (bSuccess == false) {
			cout << "Video camera is disconnected" << endl;
			cin.get(); //Wait for any key press
			//break;

		}

		//imshow(window_name1, frame1);
		//imshow("left", leftframe);
		imshow("right", frame1);
		if (waitKey(10) == 27) {
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}
	}

	return 0;

}

