#include "cameraCalibration.h"

using namespace cv;
using namespace std;


	//Capture the images to calibration. 
	void CameraCalibration::capture()
	{
		Ptr<aruco::Dictionary> mDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50 );
		Ptr<aruco::CharucoBoard> mCharucoBoard = aruco::CharucoBoard::create(7,7,0.032,0.016,mDictionary);
		namedWindow ("WebCam", WINDOW_AUTOSIZE);
		VideoCapture cap(0);
		Mat frame;
		while (true)
		{
		 	cap>>frame;
		 	imshow("WebCam", frame);
		 	char key = (char)waitKey(10);
		 	if(key == 'q')
		 	{
		 		destroyWindow("WebCam");
		 		return;
		 	}
		 }
	}
	
	bool CameraCalibration::calibrate(String path)
	{
	// detect and draw Charuco markers in webcam
	namedWindow ("WebCam", WINDOW_AUTOSIZE);
	VideoCapture cap(0);
	vector<Mat> AllImages;
	vector< vector< vector <Point2f> > > allCorners; // vector of all corners detected in all images 
	vector<vector<int> > allIds;	// all ids of all images 
	Mat cameraMatrix, distCoeffs; //Camera Params
	vector <Mat> rvecs, tvecs;  //Used in calibration
	double repError;	// used in calibration 
	Size imgSize;	
	int calibrationFlags =0; //dont especify any CalibrationFlag 
	float aspectRatio =1;   // Used in calibration 
	cout<< "Press 'c' to capture or q to start calibration\n";
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50 );
	Ptr<aruco::CharucoBoard> CharucoBoard = aruco::CharucoBoard::create(7,7,0.032,0.016,dictionary);
	}

	Mat CameraCalibration::drawMarkers(Mat frame)
	{
		Ptr<aruco::DetectorParameters> detectorParameters = aruco::DetectorParameters::create(); 
		vector< vector < Point2f > > corners, rejected;
		vector <int> ids;
		aruco::detectMarkers(frame, dictionary, corners, ids, detectorParameters, rejected);
		aruco::refineDetectedMarkers(frame, CharucoBoard, corners, ids, rejected);
		Mat currentCharucoCorners, currentCharucoIds;
		if (ids.size()>0)
		{
			aruco::interpolateCornersCharuco(corners, ids, frame, CharucoBoard, currentCharucoCorners, currentCharucoIds);
			aruco::drawDetectedMarkers(frame,corners);
		}
		if (currentCharucoCorners.total()>0)
			aruco::drawDetectedCornersCharuco(frame,currentCharucoCorners,currentCharucoIds);
		return frame;
	}