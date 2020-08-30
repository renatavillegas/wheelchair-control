//Main class. This class will be used to initialize all the system. 

#include <stdio.h>
#include "cameraCalibration.cpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.h>
#include <vector>
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

int main (int argc,char *argv[])
{
	//verify if there's a file on args. If not, ask if the user wants to do a calibration. 

	//Start the camera traking - 1 thread
	//Start the User interface - 1 thread
	//Start the communication with Vrep - 1 thread.
	//CameraCalibration cam;
	ifstream inFile (argv[1]);
	if (!inFile)
	{
		cout << "There's no calibration file.\n"
		<<"Do you want to start Camera Calibration?\n"
		<<"Press y to calibrate the Camera\n"
		<<"Press n to cancel\n";  
		char r;
		cin>>r;
		if (r=='n')
			return 0;
		if (r=='y')
		{
			CameraCalibration cam; 
			cam.show_info();
			cam.calibrate();
		}
	} 
	return 0;
}