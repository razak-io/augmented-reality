#include <stdio.h>
#include <iostream>

using namespace std;

extern int cam_calibrate();
extern int stereo_calib();
extern int rectify();
extern int triangulate();

int main() {
	int option = 0;
	std::cout << "Options available: \n" << "1. Camera Calibration\n"
			<< "2. Stereo Calibration\n" << "3. Rectification\n"
			<< "4. Triangulation Test\n" << "0. Exit\n" << std::endl;
	do {
		printf("Pick an Option: ");
		scanf("%d", &option);
		switch (option) {
			case 0:
				break;
			case 1:
				std::cout << "Camera Calibration: \n" << std::endl;
				cam_calibrate();
				break;
			case 2:
				std::cout << "Stereo Calibration: \n" << std::endl;
				stereo_calib();
				break;
			case 3:
				std::cout << "Rectification: \n" << std::endl;
				rectify();
				break;       // and exits the switch
			case 4:
				std::cout << "Triangulation Test: \n" << std::endl;
				triangulate();
				break;
			default:
				std::cout << "Invalid Option\n" << std::endl;
				break;
		}
	} while (option != 0);
	std::cout << "Exiting\n" << std::endl;
}
