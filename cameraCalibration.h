
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

#define ROOT_PATH "/home/renata/Documents/IC/CalibrationImages/";
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
			String output_path = ROOT_PATH + "resultFile.txt";
			Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50 );
			Ptr<aruco::CharucoBoard> CharucoBoard = aruco::CharucoBoard::create(7,7,0.032,0.016,dictionary);
		};
		static bool loadCameraCalibration (String resultFile, Mat& cameraMatrix, Mat& distanceCoeff); // Read .txt file
		static bool calibrate(String path);
		void capture();
		bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags, 
								 const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr, 
								 vector <Mat> &rvecs, vector <Mat> &tvecs);
		Mat drawMarkers(Mat image);
		void show_info();
		void add_image(Mat image);
		void save_images_to_folder(string ImagesPath);


};

#endif /* CAMERACALIBRATION_H */