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
void CharucoMarker::initial_info()
{
	cout << "Each door have an unique CharucoMarker "
	<<"that will be used to define from position from the chair."
	<<"Press q to cancel the search."<<endl;
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
	initial_info();
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




