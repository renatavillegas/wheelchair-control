#include "charucomarker.h"
#include <unistd.h>
#include "cameraCalibration.h"
#include "marker.h"
#include "camera.h"
CameraCalibration mcameraParams;
bool mstatus=true; 
vector<int> markerIds;
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
	Camera cam;
	cam.open();
}
void CharucoMarker::start_thread()
{
	thread mt1(&CharucoMarker::show, this);
	mt1.detach();
}




