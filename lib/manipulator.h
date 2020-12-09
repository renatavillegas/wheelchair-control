#ifndef MANIPULATOR_H // include guard
#define MANIPULATOR_H
#define GLM_ENABLE_EXPERIMENTAL
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
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <string>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include "/home/renata/V-REP_PRO_EDU_V3_5_0_Linux/programming/remoteApi/extApi.h"
#include "/home/renata/V-REP_PRO_EDU_V3_5_0_Linux/programming/remoteApi/extApiPlatform.h"
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "cameraCalibration.h"
#include "marker.h"
using namespace cv;
using namespace std;
using namespace glm;

class Manipulator
{
	private:
		int clientID;
    	int jacoHandle;
    	int ikTarget;
    	int ikTip;
    	int ikGroup;
    	int target0;
    	int target1;
    	int rmlHandle;
    	simxChar path; 
		simxChar length; 
    	int jh[6];
		int jt[6];
		float jointsUpperVelocityLimits[];

	public:	
		Manipulator(int id);
		Manipulator();
		void getHandPosition();
		void getHandOrientation();
		void setKnobPosition(simxFloat doorPosition[3]);
		void setKnobOrientation(simxFloat doorOrientation[3]);
		void exec();
		int get_JacoHandle();
		int get_target1Handle();
		//motion planning related methods
		void motion_planning();
		void calculate_velocity_factor();
		float get_simStepTime();
		void get_jointsUpperVelocityLimits();
		void execute_motion();
		int get_rmlHandle();
};



#endif /* MANIPULATOR_H */