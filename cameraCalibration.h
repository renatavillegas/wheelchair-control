
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

	public:
	//empty constructor 
		CameraCalibration(){};
		CameraCalibration(FileStorage resultFile);
		static bool loadCameraCalibration (String resultFile, Mat& cameraMatrix, Mat& distanceCoeff); // Read .txt file
		bool startCalibration(String path);
		static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags, 
								 const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr, 
								 vector <Mat> &rvecs, vector <Mat> &tvecs);
		void save_images_to_folder(vector<Mat> Images, string ImagesPath);

};

#endif /* CAMERACALIBRATION_H */