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

bool status = true;

float arucoSquareDimension =0.12f;


bool Camera::get_status()
{
	return status;
}


void Camera::startCamera()
{
	VideoCapture cap(0);
	namedWindow ("WebCam", WINDOW_AUTOSIZE);
	while(true)
	{
		char key = (char)waitKey(30);
		if(key == 'q')
		{
			destroyWindow("WebCam");
			cout << "Camera Closed.\n";
			status=false; 
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
			add_marker(frame);
		}
		if (currentCharucoCorners.total()>0)
			aruco::drawDetectedCornersCharuco(frame,currentCharucoCorners,currentCharucoIds);
		return frame;
	}
	else 
		cout << "NULL frame\n";
}
Mat Camera::add_marker(Mat frame)
{
	vector<Vec3d> AngleVector;
	vector<Vec3d> DistanceVector;

	for(int i =0; i<ids.size();i++)
	{
		aruco:: estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, params.get_cameraMatrix(), 
	 								  params.get_distCoeffs(), AngleVector, DistanceVector);
		if(!Marker::is_in_list(detectedMarkers, ids[i]))
		{
			Marker new_marker(ids[i], DistanceVector[i], AngleVector[i]);
			aruco::drawAxis(frame, params.get_cameraMatrix(), params.get_distCoeffs(), 
						new_marker.get_angle(),  new_marker.get_position(), 0.01f);
			detectedMarkers.push_back(new_marker);
			new_marker.print();
		}
	}
	return frame;
}
void Camera::open()
{
	thread mt1(&Camera::startCamera, this);
	mt1.detach();
}