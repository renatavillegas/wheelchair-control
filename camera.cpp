#include "camera.h"
#include "cameraCalibration.h"
using namespace cv;
using namespace std;
CameraCalibration params;

vector <int> ids;

vector<vector<Point2f> > markerCorners, rejected;

vector<vector<Point2f> > EstimateMarker;

Ptr<aruco::DetectorParameters> detectorParameters;


vector <Marker> detectedMarkers; 

Mat currentCharucoCorners, currentCharucoIds;

Mat image;

float arucoSquareDimension;

void Camera::open()
{
	VideoCapture cap(0);
	namedWindow ("WebCam", WINDOW_AUTOSIZE);
	while(true)
	{
		if(char key = (char)waitKey(20)=='q')
		{
			destroyWindow("WebCam");
			cout << "Camera Closed.\n";
			mstatus=false; 
			return;
		}
	 	cap>>image;
	 	draw_markers(image);
		imshow("WebCam", image);
	}
}

Mat Camera::draw_markers(Mat frame)
{
	if (!frame.empty())
	{
		aruco::detectMarkers(frame, params.get_dictionary(), markerCorners, ids, detectorParameters, rejected);
		aruco::refineDetectedMarkers(frame, params.get_charucoBoard(), markerCorners, ids, rejected);
		Mat currentCharucoCorners, currentCharucoIds;
		if (ids.size()>0)
		{
			aruco::interpolateCornersCharuco(markerCorners, ids, frame, params.get_charucoBoard(), 
											 currentCharucoCorners, currentCharucoIds);
			aruco::drawDetectedMarkers(frame,markerCorners);
		}
		if (currentCharucoCorners.total()>0)
			aruco::drawDetectedCornersCharuco(frame,currentCharucoCorners,currentCharucoIds);
		return frame;
	}
	else 
		cout << "NULL frame\n";
}
void Camera::get_position()
{

}