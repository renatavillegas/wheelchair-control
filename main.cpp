//Main class. This class will be used to initialize all the system. 

#include <stdio.h>
#include "lib/cameraCalibration.cpp"
#include "lib/marker.cpp"
#include "lib/camera.cpp"
#include "lib/remoteapi.cpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.h>
#include <vector>
#include <iostream>
#include <ctime>

using namespace cv;
using namespace std;

int main (int argc,char *argv[])
{
	//verify if there's a file on args. If not, ask if the user wants to do a calibration. 

	//Start the camera traking - 1 thread
	//Start the User interface - 1 thread
	//Start the communication with Vrep - 1 thread.
	//CameraCalibration cam;
	ifstream inFile (argv[1]);
	if (!inFile)
	{
		cout << "There's no calibration file.\n"
		<<"Do you want to start Camera Calibration?\n"
		<<"Press y to calibrate the Camera\n"
		<<"Press n to cancel\n";  
		char r;
		cin>>r;
		if (r=='n')
			return 0;
		if (r=='y')
		{
			CameraCalibration params; 
			params.show_info();
			params.calibrate();
		}
	} 
	else 
	{
		//Load the calibration params 
		CameraCalibration params;
		params.set_output_path(argv[1]); 
		if (!params.loadCameraCalibration())
		{
			cout <<"Could not load the calibration params.\n";
		}
		// Start the maker tracking
		Camera cam;
		cam.info();
		cam.open();
		while(cam.get_status())
		{
			if(cam.is_ready())
			{
				cam.set_ready(false);
				cout << "Ok, user choose to open a door.\n";
				cout<<"Please, enter the ID of tag corresponding"
			     	<< " to the doot you want to open\n";
			    int id=-1;
			    cin>>id;
			    if(id!=-1)
			    {
			    	Marker simTag = cam.get_detectedMarker(id);
			    	RemoteApi simulation; 
					simulation.connect();
					simulation.set_tag_position(simTag);
			    }
				
			}
		}

	}

	//now we have to start the thread to find the marker and follow it.
	//and the tread to start simulation and follow it. 
	return 0;
}