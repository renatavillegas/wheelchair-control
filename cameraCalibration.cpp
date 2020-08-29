#include "cameraCalibration.h"

using namespace cv;
using namespace std;
	
	//Webcam frame
	Mat frame;
	//Vector of images to calibrate
	vector<Mat> mAllImages;
	// vector of all corners detected in all images 
	vector< vector< vector <Point2f> > > mallCorners;
	// all ids of all images  
	vector<vector<int> > mallIds;
	//Camera Params	
	Mat cameraMatrix, mdistCoeffs;
	//Used in calibration 
	vector <Mat> mrvecs, mtvecs;  
	// used in calibration 
	double mrepError =0;	
	// Image Size
	Size mimgSize;
	//dont especify any CalibrationFlag 	
	int mcalibrationFlags=0; 
	// Used in calibration 
	float maspectRatio=0; 
	// Number of images salved 
	int mimageCount=0;
	// Path to resultFile
	String moutput_path = "";
	// detectorParameters of the arUco markers
	Ptr<aruco::DetectorParameters> mdetectorParameters = aruco::DetectorParameters::create();
	// vectors of points of the makers 
	vector< vector < Point2f > > mcorners, mrejected;
	// vector to save the ids of the markers of one image
	vector <int> mids;
	Ptr<aruco::Dictionary> mdictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50 );
	Ptr<aruco::CharucoBoard> mcharucoBoard = aruco::CharucoBoard::create(7,7,0.032,0.016,mdictionary);
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

	}

	Mat CameraCalibration::drawMarkers(Mat frame)
	{
		if (!frame.empty())
		{
			aruco::detectMarkers(frame, mdictionary, mcorners, mids, mdetectorParameters, mrejected);
			aruco::refineDetectedMarkers(frame, mcharucoBoard, mcorners, mids, mrejected);
			Mat mcurrentCharucoCorners, mcurrentCharucoIds;
			if (mids.size()>0)
			{
				aruco::interpolateCornersCharuco(mcorners, mids, frame, mcharucoBoard, 
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