#include "remoteapi.h"
using namespace cv;
using namespace std;
using namespace glm;

int clientID=-1;
int wheelChair;
int wheelChairIni=-1;
int arTag;
int arTagIni=-1;
simxFloat TagQuaternion[4];
simxFloat TagPosition[3];
Marker realTag;


int RemoteApi::get_clientID()
{
	return clientID;
}
void RemoteApi::connect()
{
	clientID=simxStart("127.0.0.1",19997,true,true,100,5);
	if(clientID == -1) 
	{
		cout << "ERROR: No connection avaliable\n";
		simxFinish(clientID);
	}
	wheelChairIni = simxGetObjectHandle(clientID, "wheelchair", &wheelChair, simx_opmode_blocking);
	if(wheelChairIni==-1)
	{
		cout << "ERROR: Can't connect to the robot.\n";
		simxFinish(clientID);
	}
	arTagIni = simxGetObjectHandle(clientID, "ARTag", &arTag, simx_opmode_blocking);
	if(arTag == -1)
	{
		cout << "ERROR: Can't find the tag.\n";
		simxFinish(clientID);
	}
}
void RemoteApi::set_tag_position(Marker tag)
{
	Marker realTag;
	realTag = tag; 

	simxFloat RotQuaternion[4] = {(float)realTag.get_quaternion().x,
								  (float)realTag.get_quaternion().y,
								  (float)realTag.get_quaternion().z,
								  (float)realTag.get_quaternion().w};
	simxFloat Position[3] = {(float)realTag.get_position()[0], 
							 (float)realTag.get_position()[1], 
							 (float)realTag.get_position()[2]};
	realTag.print();
	simxSetObjectPosition(clientID, arTag, -1, Position, simx_opmode_oneshot);
	simxSetObjectQuaternion(clientID, arTag, -1, RotQuaternion, simx_opmode_oneshot);
}

