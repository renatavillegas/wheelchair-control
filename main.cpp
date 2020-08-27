//Main class. This class will be used to initialize all the system. 

#include <stdio.h>
#include "cameraCalibration.h"
using namespace cv;
int main (int argc,char *argv[])
{
	//verify if there's a file on args. If not, ask if the user wants to do a calibration. 

	//Start the camera traking - 1 thread
	//Start the User interface - 1 thread
	//Start the communication with Vrep - 1 thread.
	//CameraCalibration cam; 
	CameraCalibration cam; 
	cout << "Hello cam\n";
	cam.capture();
	//cam.capture();
	return 0;
}