
#ifndef CAMERACALIBRATION_H // include guard
#define CAMERACALIBRATION_H

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

using namespace cv;
using namespace std;

class CameraCalibration
{
	private:
		Mat frame;
		vector<Mat> AllImages;
		vector< vector< vector <Point2f> > > allCorners; // vector of all corners detected in all images 
		vector<vector<int> > allIds;	// all ids of all images 
		Mat cameraMatrix, distCoeffs; //Camera Params
		vector <Mat> rvecs, tvecs;  //Used in calibration
		double repError;	// used in calibration 
		Size imgSize;	
		int calibrationFlags; //dont especify any CalibrationFlag 
		float aspectRatio;   // Used in calibration 
		int imageCount;
		String output_path;
		Ptr<aruco::Dictionary> dictionary;
		Ptr<aruco::CharucoBoard> CharucoBoard;

	public:
		CameraCalibration(){
			Mat frame;
			vector<Mat> AllImages;
			vector< vector< vector <Point2f> > > allCorners; // vector of all corners detected in all images 
			vector<vector<int> > allIds;	// all ids of all images 
			Mat cameraMatrix, distCoeffs; //Camera Params
			vector <Mat> rvecs, tvecs;  //Used in calibration
			double repError =0;	// used in calibration 
			Size imgSize;	
			int calibrationFlags=0; //dont especify any CalibrationFlag 
			float aspectRatio=0;   // Used in calibration 
			int imageCount=0;
			String ROOT_PATH = "/home/renata/Documents/IC/CalibrationImages/";
			String output_path = ROOT_PATH + "resultFile.txt";
			Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50 );
			Ptr<aruco::CharucoBoard> CharucoBoard = aruco::CharucoBoard::create(7,7,0.032,0.016,dictionary);
		};
		// Read .txt file
		bool loadCameraCalibration (String resultFile, Mat& cameraMatrix, Mat& distanceCoeff); 
		// start calibration
		bool start_calibration();
		// capture the images to start_calibration
		void calibrate();
		// save the calibration result on a file
		bool saveCameraParams();
		// draw markers on the webcam view
		Mat drawMarkers(Mat image);
		// some info displayed before start calibration
		void show_info();
		// add the images captured on calibration
		void add_image(Mat image);
		// save the images to an output folder
		void save_images_to_folder(string ImagesPath);
};

#endif /* CAMERACALIBRATION_H */