#ifndef CHARUCOMARKER_H // include guard
#define CHARUCOMARKER_H

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
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <string>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include "cameraCalibration.h"

using namespace cv;
using namespace std;

class CharucoMarker
{
private:
	thread t1;
	//Webcam frame
	Mat frame;
	VideoCapture cap();
	CameraCalibration mcameraParams;
	void show();
	void hello_thread();
public:
	void show_info();
	void start_thread();
	CharucoMarker(CameraCalibration params)
	{
		mcameraParams= params;
	}
	CameraCalibration get_cameraParams();
};

#endif /* CHARUCOMARKER_H */
