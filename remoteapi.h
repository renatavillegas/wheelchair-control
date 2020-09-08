#ifndef REMOTEAPI_H // include guard
#define REMOTEAPI_H
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
class RemoteApi
{
	private:
		int clientID;
	public:	
		//getter- setter
		int get_clientID();
		void connect();
		RemoteApi()
		{
			clientID = -1;
		}
};



#endif /* REMOTEAPI_H */