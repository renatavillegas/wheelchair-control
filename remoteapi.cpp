#include "remoteapi.h"
using namespace cv;
using namespace std;

int clientID=-1;
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

}