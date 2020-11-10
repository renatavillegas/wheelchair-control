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

bool ready= false;

float arucoSquareDimension =0.135f;


bool Camera::get_status()
{
	return status;
}
bool Camera::is_ready()
{
	return ready;
}
void Camera::set_ready(bool state)
{
	ready = state;
}
Marker Camera::get_detectedMarker(int id)
{
	return Marker::get_marker_in_list(detectedMarkers, id);
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
		if(key == 'y')
		{
			ready = true;
		}
	 	cap>>image;
	 	draw_markers(image);
		imshow("WebCam", image);
	}
}

void Camera::draw_markers(Mat frame)
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
	}
	else 
		cout << "NULL frame\n";
}
void Camera::add_marker(Mat frame)
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
			cout << "There is a new door on the way."<<endl
				 << "If you want to open it, please press y." <<endl
				 << "If you don't, just continue moving." <<endl;	
			new_marker.print();
		}
	}
}
void Camera::open()
{
	thread mt1(&Camera::startCamera, this);
	mt1.detach();
}
void Camera::info()
{
	cout<< "Now the chair is in the manual control state." <<endl 
	<<"The user can move through the room." << endl
	<<"If any door is found on the way " 
	<<"the user will be asked if he wants to open or keep moving."<< endl
	<<"Press q to cancel the movement and close the camera." << endl;  
}