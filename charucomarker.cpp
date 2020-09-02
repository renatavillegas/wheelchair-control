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
	while(true)
	{

	}
}
void CharucoMarker::start_thread()
{
	thread mt1(&CharucoMarker::hello_thread, this);
	mt1.detach();
}

