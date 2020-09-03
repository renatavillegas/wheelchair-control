#include "charucomarker.h"
#include <unistd.h>
#include "cameraCalibration.h"

CameraCalibration mcameraParams;
bool mstatus=true; 
//get-set
CameraCalibration CharucoMarker::get_cameraParams()
{
	return mcameraParams;
}
bool CharucoMarker::get_status()
{
	return mstatus;
}
// Display some info to user
void CharucoMarker::show_info()
{
	cout << "Each door have an unique CharucoMarker "
	<<"that will be used to define from position from the chair."
	<<"Press q to cancel the search.";
}

void CharucoMarker::hello_thread()
{
	while(true)
	{
		cout << "hello thread!\n";
		sleep(10);
	}
}
void CharucoMarker::show()
{
	namedWindow ("WebCam", WINDOW_AUTOSIZE);
	VideoCapture cap(0);
	while(true)
	{
		if(char key = (char)waitKey(20)=='q')
		{
			destroyWindow("WebCam");
			cout << "Camera Closed.\n";
			mstatus=false; 
			return;
		}
	 	cap>>frame;
		imshow("WebCam", mcameraParams.drawMarkers(frame));
	}
}
void CharucoMarker::start_thread()
{
	thread mt1(&CharucoMarker::show, this);
	mt1.detach();
}




