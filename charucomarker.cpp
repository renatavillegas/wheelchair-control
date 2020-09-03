#include "charucomarker.h"
#include <unistd.h>
#include "cameraCalibration.h"

CameraCalibration mcameraParams;

//get-set
CameraCalibration CharucoMarker::get_cameraParams()
{
	return mcameraParams;
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
	 	cap>>frame;
		 char key = (char)waitKey(20);
		 if(key == 'q')
	 	{
	 		destroyWindow("WebCam");
	 		cout << "Calibration canceled.\n";
	 		return;
	 	}
		imshow("WebCam", drawMarkers(frame));
	}
}
void CharucoMarker::start_thread()
{
	thread mt1(&CharucoMarker::hello_thread, this);
	mt1.detach();
}

