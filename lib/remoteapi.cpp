#include "remoteapi.h"
using namespace cv;
using namespace std;
using namespace glm;

int clientID=-1;
int robot;
int robotIni=-1;
int cameraDummy;
int cameraDummyIni=-1;
int arTag;
int arTagIni=-1;
int door; 
int doorIni = -1;
int goal;
int goalIni = -1;
simxFloat TagQuaternion[4];
simxFloat TagPosition[3];
Marker realTag;


int RemoteApi::get_clientID()
{
	return clientID;
}
void RemoteApi::connect()
{
	clientID=simxStart("127.0.0.1",19999,true,true,100,5);
	simxSynchronous(clientID,true);
	if(clientID == -1) 
	{
		cout << "ERROR: No connection avaliable\n";
		simxFinish(clientID);
	}
	robotIni = simxGetObjectHandle(clientID, "wheelchair2", &robot, simx_opmode_blocking);
	if(robotIni==-1)
	{
		cout << "ERROR: Can't connect to the robot.\n";
		simxFinish(clientID);
	}
	cameraDummyIni = simxGetObjectHandle(clientID, "CameraDummy", &cameraDummy, simx_opmode_blocking);
	if(cameraDummyIni==-1)
	{
		cout << "ERROR: Can't connect to the camera dummy.\n";
		simxFinish(clientID);
	}
	arTagIni = simxGetObjectHandle(clientID, "ARTag", &arTag, simx_opmode_blocking);
	if(arTag == -1)
	{
		cout << "ERROR: Can't find the tag.\n";
		simxFinish(clientID);
	}
	doorIni = simxGetObjectHandle(clientID, "DoorPositionDummy", &door, simx_opmode_blocking);
	if(doorIni == -1)
	{
		cout << "ERROR: Can't find the door.\n";
		simxFinish(clientID);
	}
	goalIni = simxGetObjectHandle(clientID, "GoalCameraDummy", &goal, simx_opmode_blocking);
	if(goalIni == -1)
	{
		cout << "ERROR: Can't find the goal.\n";
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
	simxFloat doorPos[3];
	simxGetObjectPosition(clientID, door, -1, doorPos, simx_opmode_blocking);
	simxFloat robotPos[3];
	simxGetObjectPosition(clientID, cameraDummy, -1, robotPos, simx_opmode_blocking);
	//change the door position based on the TAG position 
	doorPos[0]=robotPos[0]-(float)realTag.get_position()[0];
	doorPos[1]=robotPos[1]-(float)realTag.get_position()[1];
	cout << "doorPos:" <<doorPos[0] <<"; "<< doorPos[1] <<"; "<< doorPos[2] << endl;
		realTag.print();

	simxSetObjectPosition(clientID, door, -1, doorPos, simx_opmode_oneshot);
	//simxSetObjectQuaternion(clientID, doorPosition, wheelChair, RotQuaternion, simx_opmode_blocking);
	// change the goal position based on the door position.
	// get the door position in relation to the world 
	simxGetObjectPosition(clientID, door, -1, doorPos, simx_opmode_blocking);
	cout << "doorPos2:" <<doorPos[0] <<"; "<< doorPos[1] <<"; "<< doorPos[2] << endl;
	//get the goalPosition relative to the world
	simxFloat goalPos[3];
	simxGetObjectPosition(clientID, goal, -1, goalPos, simx_opmode_blocking);
	goalPos[0]=doorPos[0];
	goalPos[1]=doorPos[1]+0.01;
	simxSetObjectPosition(clientID, goal, -1, goalPos, simx_opmode_oneshot);
	//assuming that everything is working, call the path planning 
	simxCallScriptFunction(clientID, "autodrive2", sim_scripttype_childscript ,"path_planning",
							 0, NULL, 0, NULL, 0, NULL, 0, NULL, //inputs
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, //outputs
							 simx_opmode_oneshot);


}

