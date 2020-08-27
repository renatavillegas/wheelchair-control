//Main class. This class will be used to initialize all the system. 

#include <stdio.h>
#include "cameraCalibration.h"

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
	CameraCalibration cam; 
	cout << "Hello cam\n";
	Mat frame;
	VideoCapture cap;
	cam.capture();
	return 0;
}