#include "cameraCalibration.h"

using namespace cv;
using namespace std;


	// print some info to the user before calibration. 

	void CameraCalibration::show_info()
	{
		cout<<"Welcome to camera calibration.\n"
			<<"INFO:To calibrate using a ChArUco board," 
			<<"it is necessary to detect the board from different viewpoints.\n"
			<<"Press s to save the webcam image to use on calibration.\n"
			<<"After taking more than 5 files, press x to stop capturing and start calibration.\n";
	}
	//Capture the images to calibration. 
	void CameraCalibration::capture()
	{
		namedWindow ("WebCam", WINDOW_AUTOSIZE);
		VideoCapture cap(0);
		Mat frame;
		Mat image;
		while (true)
		{
		 	cap>>frame;
		 	char key = (char)waitKey(20);
		 	if(key == 'q')
		 	{
		 		destroyWindow("WebCam");
		 		return;
		 	}
		 	imshow("WebCam", drawMarkers(frame));

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
		Ptr<aruco::DetectorParameters> mdetectorParameters = aruco::DetectorParameters::create(); 
		vector< vector < Point2f > > mcorners, mrejected;
		vector <int> mids;
		Ptr<aruco::Dictionary> mDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50 );
		Ptr<aruco::CharucoBoard> mCharucoBoard = aruco::CharucoBoard::create(7,7,0.032,0.016,mDictionary);
		if (!frame.empty())
		{
			aruco::detectMarkers(frame, mDictionary, mcorners, mids, mdetectorParameters, mrejected);
			aruco::refineDetectedMarkers(frame, mCharucoBoard, mcorners, mids, mrejected);
			Mat mcurrentCharucoCorners, mcurrentCharucoIds;
			if (mids.size()>0)
			{
				aruco::interpolateCornersCharuco(mcorners, mids, frame, mCharucoBoard, 
												 mcurrentCharucoCorners, mcurrentCharucoIds);
				aruco::drawDetectedMarkers(frame,mcorners);
			}
			if (mcurrentCharucoCorners.total()>0)
				aruco::drawDetectedCornersCharuco(frame,mcurrentCharucoCorners,mcurrentCharucoIds);
			return frame;
		}
		else 
			cout << "NULL frame\n";
		return frame;
	}