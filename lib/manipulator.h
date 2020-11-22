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

extern "C" {
    #include "/home/renata/V-REP_PRO_EDU_V3_5_0_Linux/programming/remoteApi/extApi.c"
     #include "/home/renata/V-REP_PRO_EDU_V3_5_0_Linux/programming/remoteApi/extApiPlatform.c"
	#include "/home/renata/V-REP_PRO_EDU_V3_5_0_Linux/programming/common/shared_memory.c"
}
class Manipulator
{
	private:
		int clientID;
		int jh[] = {-1,-1,-1,-1,-1,-1};
		int jt[] = {-1,-1,-1,-1,-1,-1};
    	int jacoHandle=sim.getObjectHandle('Jaco');    
    	int ikTarget=sim.getObjectHandle('Jaco_target')
    	int ikTip=sim.getObjectHandle('Jaco_tip')
    	int ikGroup=sim.getIkGroupHandle('Jaco_ik')
    	int target0=sim.getObjectHandle('jacoTarget0')
    	int target1=sim.getObjectHandle('jacoTarget1')
    print("Jaco initialized.")

	public:	
		Manipulator(id)
		{
			clientID = id;
			jacoHandle=simx.getObjectHandle('Jaco');
		}
		void exec();
};



#endif /* MANIPULATOR_H */