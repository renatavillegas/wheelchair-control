class CameraCalibration
{
	private:


	public: 
	bool loadCameraCalibration (string resultFile, Mat& cameraMatrix, Mat& distanceCoeff); // Read .txt file
	bool startCalibration(string path); 
	void save_images_to_folder(vector<Mat> Images, string ImagesPath)
}