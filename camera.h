#ifndef CAMERA_H // include guard
#define CAMERA_H
#define GLM_ENABLE_EXPERIMENTAL
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
#include "/home/renata/V-REP_PRO_EDU_V3_5_0_Linux/programming/remoteApi/extApi.h"
#include "/home/renata/V-REP_PRO_EDU_V3_5_0_Linux/programming/remoteApi/extApiPlatform.h"
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "cameraCalibration.h"
#include "marker.h"
using namespace cv;
using namespace std;
using namespace glm;

class Camera 
{
private: 
	// camera params used to estimate the marker position
	CameraCalibration params;
	// all marker ids of the frame
	vector <int> ids;
	// Used to find the position
	vector<vector<Point2f> > markerCorners, rejected;
	// ARuCo Estimator
	vector<vector<Point2f> > EstimateMarker;
	// if you want to use some flag, or change the parameters of the search
	Ptr<aruco::DetectorParameters> detectorParameters;
	// vector of all detectedMarkers of the frames
	vector <Marker> detectedMarkers; 
	// frame of the current corners and IDs detected
	Mat currentCharucoCorners, currentCharucoIds;
	// size of the markers
	float arucoSquareDimension;
	//Used to show the webcam image
	thread t1;
	void startCamera();
	bool status;
public: 
	void open();
	void draw_markers(Mat frame);
	void add_marker(Mat frame);
	bool get_status();
	void info();


	Camera()
	{
		arucoSquareDimension= 0.12f;
		detectorParameters = aruco::DetectorParameters::create();
		if(!params.loadCameraCalibration())
		{
			cout << "Could not read the camera parameters." << endl;
		}
		status = true;
	}
	void close();
};

#endif /* MARKER_H */