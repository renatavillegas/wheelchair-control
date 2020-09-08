#include "remoteapi.h"
using namespace cv;
using namespace std;

int clientID=-1;
int wheelChair;
int wheelChairIni=-1;
int arTag;
int arTagIni=-1;

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