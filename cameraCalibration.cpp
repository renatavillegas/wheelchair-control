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
		while (int i=0 < 10000)
		{
			Mat frame;
			cap.read(frame);
			imshow("WebCam", frame);
			i++;
		}
	}
	
	bool CameraCalibration::calibrate(String path)
	{
			// detect and draw Charuco markers in webcam
		namedWindow ("WebCam", WINDOW_AUTOSIZE);
		VideoCapture cap(0);
	}


