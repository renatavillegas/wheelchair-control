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
	String ROOT_PATH = "/home/renata/Documents/IC/CalibrationImages/";
	String moutput_path = ROOT_PATH + "result.txt";
	// detectorParameters of the arUco markers
	Ptr<aruco::DetectorParameters> mdetectorParameters = aruco::DetectorParameters::create();
	// vectors of points of the makers 
	vector< vector < Point2f > > mcorners, mrejected;
	// vector to save the ids of the markers of one image
	vector <int> mids;
	// Dictionary of markers 
	Ptr<aruco::Dictionary> mdictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	// Charuco board
	Ptr<aruco::CharucoBoard> mcharucoBoard = aruco::CharucoBoard::create(7,7,0.032,0.016,mdictionary);
	//all corner and all ids used to calibrate
	vector <Mat> mallCharucoCorners, mallCharucoIds;

	// ---------------------Getter and setter -------------------------
	void CameraCalibration::set_output_path(String output_path)
	{
		moutput_path = output_path;
		return;
	}
	String CameraCalibration::get_output_path()
	{
		return moutput_path;
	}
	Ptr<aruco::Dictionary> CameraCalibration::get_dictionary()
	{
		return mdictionary;
	}
		
	Ptr<aruco::CharucoBoard> CameraCalibration::get_charucoBoard()
	{
		return mcharucoBoard;
	}
	Mat CameraCalibration::get_cameraMatrix()
	{
		return cameraMatrix;
	}
	Mat CameraCalibration::get_distCoeffs()
	{
		return mdistCoeffs;
	}
	bool CameraCalibration::loadCameraCalibration ()
	{
		FileStorage fs (moutput_path, FileStorage::READ);
		if (!fs.isOpened())
			return false;
		fs["camera_matrix"] >> cameraMatrix;
		fs["distortion_coefficients"] >> mdistCoeffs;
		return true;
	}	
	// -----------------------------------------------------------------

	// print some info to the user before calibration. 
	void CameraCalibration::show_info()
	{
		cout<<"Welcome to camera calibration.\n"
			<<"INFO:To calibrate using a ChArUco board," 
			<<"it is necessary to detect the board from different viewpoints.\n"
			<<"Press s to save the webcam image to use on calibration.\n"
			<<"After taking more than 5 pictures, press x to stop capturing and start calibration.\n"
			<<"Press q to cancel calibration.\n";
	}
	//Capture the images to calibration. 
	void CameraCalibration::calibrate()
	{
		namedWindow ("WebCam", WINDOW_AUTOSIZE);
		VideoCapture cap(0);
		while (true)
		{
		 	cap>>frame;
		 	char key = (char)waitKey(20);
		 	if(key == 'q')
		 	{
		 		save_images_to_folder(ROOT_PATH);
		 		destroyWindow("WebCam");
		 		cout << "Calibration canceled.\n";
		 		return;
		 	}
		 	if(key == 's' )
		 	{
		 		add_image(frame);
		 	}
		 	if(key == 'x')
		 	{
		 		save_images_to_folder(ROOT_PATH);
		 		cout << "Starting calibration.\n";
		 		if (start_calibration())
		 		{
		 			destroyWindow("WebCam");
		 			return;
		 		}
		 		else
		 		{
		 			cout << "Calibration Failed.\n"
		 				 << "Try again or press q to cancel.\n";
		 		}

		 	}
		 	imshow("WebCam", drawMarkers(frame));

		 }
	}
	
	bool CameraCalibration::start_calibration()
	{
		if ((int)mAllImages.size()<5)
		{
			cout<< "Not enough images to calibrate\n";
			return false;
		}
		// prepare data for Charuco calibration
		int nImages = (int)mallCorners.size();
		mallCharucoCorners.reserve(nImages);
		mallCharucoIds.reserve(nImages);
		for(int i=0; i< nImages; i++)
		{
			Mat currentCharucoCorners, currentCharucoIds;
			aruco::interpolateCornersCharuco(mallCorners[i], mallIds[i], mAllImages[i], 
											 mcharucoBoard, currentCharucoCorners, 
											 currentCharucoIds, cameraMatrix, mdistCoeffs);
			mallCharucoCorners.push_back(currentCharucoCorners);
			mallCharucoIds.push_back(currentCharucoIds);
		}
		repError= aruco::calibrateCameraCharuco(mallCharucoCorners,mallCharucoIds,mcharucoBoard,mimgSize,
												cameraMatrix, mdistCoeffs, mrvecs, mtvecs, mcalibrationFlags);
		bool saveOk = saveCameraParams();
		if (saveOk)
			cout << "See results in " << moutput_path << endl;
			return true;
		return false;
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

	void CameraCalibration::add_image(Mat frame)
	{
		cout << "ids.size= "<< mids.size();
		cout <<" corners.size= "<< mcorners.size()<< endl;
		if((int)mcorners.size()>4) // minimum of identifiable markers 
		{
			
			mAllImages.push_back (drawMarkers(frame));
			mallCorners.push_back(mcorners);
			mallIds.push_back(mids);
			mimgSize = frame.size ();
			cout << "Saved frame\n AllImagesSize=" << mAllImages.size()<< endl;

		}
	}

	void CameraCalibration::save_images_to_folder(string ImagesPath)
	{
		for(int i=0; i< mAllImages.size(); i++)
		{
			String file_name =  ImagesPath + string("image") + to_string(i) + string(".png");
			bool result = imwrite(file_name, mAllImages[i]); 
			if (!result)
				cout << "Image save fail. \n";
		}
	}

	bool CameraCalibration::saveCameraParams()
	{
		FileStorage fs(moutput_path, FileStorage::WRITE);
	    cout << "fopen\n";
	    if(!fs.isOpened())
	        return false;
	    time_t tt;
	    time(&tt);
	    struct tm *t2 = localtime(&tt);
	    char buf[1024];
	    strftime(buf, sizeof(buf) - 1, "%c", t2);

	    fs << "calibration_time" << buf;
	    fs << "image_width" << mimgSize.width;
	    fs << "image_height" << mimgSize.height;

	    if(mcalibrationFlags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << maspectRatio;

	    if(mcalibrationFlags != 0) {
	        sprintf(buf, "flags: %s%s%s%s",
	                mcalibrationFlags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
	                mcalibrationFlags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
	                mcalibrationFlags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
	                mcalibrationFlags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
	    }
	    fs << "flags" << mcalibrationFlags;
	    fs << "camera_matrix" << cameraMatrix;
	    fs << "cameraMatrixRows" << cameraMatrix.rows;
	    fs << "cameraMatrixCols" << cameraMatrix.cols;
	    fs << "distortion_coefficients" << mdistCoeffs;
	    fs << "avg_reprojection_error" << mrepError;
	    fs << "rotation vectors" << mrvecs;
	    fs << "translation vectors" << mtvecs;
	    return true;
	}

