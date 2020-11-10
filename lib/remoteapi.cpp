#include "remoteapi.h"
using namespace cv;
using namespace std;
using namespace glm;

simxFloat TagQuaternion[4];
simxFloat TagPosition[3];
Marker realTag;

simxFloat goalPos[3];
simxFloat robotPos[3];

int RemoteApi::get_clientID()
{
	return clientID;
}
void RemoteApi::connect()
{
	clientID=simxStart("127.0.0.1",19999,true,true,-500000,5);
	simxSynchronous(clientID,true);
	if(clientID == -1) 
	{
		cout << "ERROR: No connection avaliable\n";
		simxFinish(clientID);
	}
}
void RemoteApi::initialize_objects()
{
		(simxGetObjectHandle(clientID, "wheelchair2", &robotHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Robot Connected"<<endl:cout<<"ERROR: Robot Connection Failed"<<endl);

		(simxGetObjectHandle(clientID, "wheelchair_rightMotor", &rightMotorHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Right motor Connected"<<endl:cout<<"ERROR: Right motor Connection Failed"<<endl);

		(simxGetObjectHandle(clientID, "wheelchair_leftMotor", &leftMotorHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Left motor Connected"<<endl:cout<<"ERROR: Left motor Connection Failed"<<endl);
		
		(simxGetObjectHandle(clientID, "Path2", &pathHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Path Connected"<<endl:cout<<"ERROR:Path Connection Failed"<<endl);

		(simxGetObjectHandle(clientID, "wheelchairFrame2", &startDummyHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Wheelchair frame Connected"<<endl:cout<<"ERROR:Wheelchair frame Connection Failed"<<endl);

		(simxGetObjectHandle(clientID, "wheelchairFrameGoal", &goalHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Goal frame Connected"<<endl:cout<<"ERROR: Goal frame Connection Failed"<<endl);
		
		(simxGetObjectHandle(clientID, "ARTag", &arTagHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"ARTag frame Connected"<<endl:cout<<"ERROR: ARTag frame Connection Failed"<<endl);
	
		(simxGetObjectHandle(clientID, "DoorPositionDummy", &doorHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Door frame Connected"<<endl:cout<<"ERROR: Door frame Connection Failed"<<endl);

		(simxGetObjectHandle(clientID, "CameraDummy", &cameraDummyHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Camera frame Connected"<<endl:cout<<"ERROR: Camera frame Connection Failed"<<endl);
		(simxGetObjectHandle(clientID, "GoalCameraDummy", &goalCameraDummyHandle, simx_opmode_oneshot_wait)
							 == simx_return_ok?cout<<"Camera frame Connected"<<endl:cout<<"ERROR: Camera frame Connection Failed"<<endl);		
}
void RemoteApi::set_tag_position(Marker tag)
{
	Marker realTag;
	realTag = tag; 

	simxFloat RotQuaternion[4] = {(float)realTag.get_quaternion().x,
								  (float)realTag.get_quaternion().y,
								  (float)realTag.get_quaternion().z,
								  (float)realTag.get_quaternion().w};
	simxFloat Orientation[3] = {(float)realTag.get_angle()[0], 
							 (float)realTag.get_angle()[1], 
							 (float)realTag.get_angle()[2]};

	simxFloat Position[3] = {(float)realTag.get_position()[0], 
							 (float)realTag.get_position()[1], 
							 (float)realTag.get_position()[2]};
	simxFloat doorPos[3];
	simxFloat doorOri[3];
	simxGetObjectPosition(clientID, doorHandle, -1, doorPos, simx_opmode_oneshot_wait);
	//robot pos
	simxGetObjectPosition(clientID, cameraDummyHandle, -1, robotPos, simx_opmode_oneshot_wait);
	//change the door position based on the TAG position 
	doorPos[0]=robotPos[0]-(float)realTag.get_position()[0];
	doorPos[1]=robotPos[1]+(float)realTag.get_position()[2];
	cout << "doorPos:" <<doorPos[0] <<"; "<< doorPos[1] <<"; "<< doorPos[2] << endl;
	realTag.print();

	simxSetObjectPosition(clientID, doorHandle, -1, doorPos, simx_opmode_oneshot);
	

	//simxSetObjectQuaternion(clientID, doorPosition, wheelChair, RotQuaternion, simx_opmode_blocking);
	// change the goal position based on the door position.
	// get the door position in relation to the world 
	if (simxGetObjectPosition(clientID, doorHandle, -1, doorPos, simx_opmode_oneshot_wait)!=simx_return_ok)
		cout<< "ERROR: get door position failed."<< endl;
	cout << "doorPos2:" <<doorPos[0] <<"; "<< doorPos[1] <<"; "<< doorPos[2] << endl;
	//get the goalPosition relative to the world
	if (simxGetObjectPosition(clientID, goalCameraDummyHandle, -1, goalPos, simx_opmode_oneshot_wait)!=simx_return_ok)
		cout<< "ERROR: get goal position failed."<< endl;
	goalPos[0]=doorPos[0]-0.3;
	goalPos[1]=doorPos[1]-0.7;
	simxSetObjectPosition(clientID, goalCameraDummyHandle, -1, goalPos, simx_opmode_oneshot);
	// change the door orientation based on the camera capture. 
	if (simxGetObjectOrientation(clientID, doorHandle, cameraDummyHandle, doorOri, simx_opmode_oneshot_wait)!=simx_return_ok)
		cout<< "ERROR: get door orientation failed."<< endl;
	cout << "doorOri in relation to the Camera:" <<doorOri[0] <<"; "<< doorOri[1] <<"; "<< doorOri[2] << endl;
	doorOri[3]= Orientation[3];
	simxSetObjectOrientation(clientID, doorHandle, cameraDummyHandle, doorOri, simx_opmode_oneshot);

	cout << "Now that we have the goal, start the path planning" << endl;
}

void RemoteApi::check_collision()
{
	//get the collision object and the robot shape
	(simxGetObjectHandle(clientID, "CollidableForPathPlanning", &collidableForPathPlanningHandle, simx_opmode_oneshot_wait)
					 	== simx_return_ok?cout<<"CollidableForPathPlanning Connected"<<endl:cout<<"ERROR: CollidableForPathPlanning Connection Failed"<<endl);
	(simxGetCollectionHandle(clientID, "p3ObstacleCollection", &obstaclesHandle, simx_opmode_oneshot_wait)
					 	== simx_return_ok?cout<<"obstacles Connected"<<endl:cout<<"ERROR: obstacles Connection Failed"<<endl);

	const int inIntCount = 2;
	int inInt []= {collidableForPathPlanningHandle, obstaclesHandle};
	int outIntCount = 1;
	int *collision;
	int ret = simxCallScriptFunction(clientID, "autodrive2", sim_scripttype_childscript, "check_collision_sim",
										inIntCount, inInt,0, NULL, 0, NULL,0,NULL,
										&outIntCount, &collision, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	(*collision == 0? cout << "We are not in collision = " << *collision << endl : cout <<"ERROR: We are colliding! " << endl);
}	

void RemoteApi::path_planning()
{
	int *pathCalculated;
	int closeToTarget=-1;
	int intCount=1;
	const int inIntCount = 1;
	int inInt []= {planningTaskHandle};
	int result = simxCallScriptFunction(clientID, "autodrive2", sim_scripttype_childscript, "path_planning",
										0, NULL,0, NULL, 0, NULL,0,NULL,
										&intCount, &pathCalculated, NULL, NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(*pathCalculated != 0)
		cout << "Path calculated = " << *pathCalculated << endl;
	else
		cout << "ERROR during the path planning." << endl;
}

void RemoteApi::path_following()
{
	const int inIntCount = 4;
	//Inputs : path handle, robot frame, left motor, right motor
	int inInt []= {pathHandle, startDummyHandle, leftMotorHandle, rightMotorHandle};
	//Outputs: Close to target, vl, vr
	int outIntCount = 1;
	int *closeToTarget =0;
	int outFloatCount = 2;
	float vr = 0;
	float vl =0; 
	float *velocities;
	do
	{
		int result = simxCallScriptFunction(clientID, "autodrive2", sim_scripttype_childscript, "path_following",
											inIntCount, inInt,0, NULL, 0, NULL,0,NULL,
											&outIntCount, &closeToTarget, &outFloatCount , &velocities, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);	
		if (result == simx_return_ok && velocities != NULL)
		{
			cout << "closeToTarget = "<< *closeToTarget << endl;
			vl = velocities[0];
			vr = velocities[1];
			cout << "LeftV = " << vl <<", RightV = " << vr << endl;
			//update velocities 
			simxSetJointTargetVelocity(clientID, rightMotorHandle, vr, simx_opmode_oneshot);
			simxSetJointTargetVelocity(clientID, leftMotorHandle, vl, simx_opmode_oneshot);
		}
	}
	while(*closeToTarget == 0);
}

void RemoteApi::adjust_orientation()
{
	cout << "Now we are close to the object so we will adjust the orientation of the chair." << endl;
	const int inIntCount = 2;
	//Inputs : robot, goal
	int inInt []= {startDummyHandle,goalHandle};
	//Outputs: Stop, vl, vr
	int outIntCount = 1;
	int *stop =0;
	int outFloatCount =2;
	float *velocities;
	float vl, vr;
	do
	{
		int result = simxCallScriptFunction(clientID, "autodrive2", sim_scripttype_childscript, "adjusting_orientation",
											inIntCount, inInt,0, NULL, 0, NULL,0,NULL,
											&outIntCount, &stop, &outFloatCount , &velocities, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);	
		if (result == simx_return_ok && velocities != NULL)
		{
			vl = velocities[0];
			vr = velocities[1];
			cout << "LeftV = " << vl <<", RightV = " << vr << endl;
			//update velocities 
			simxSetJointTargetVelocity(clientID, rightMotorHandle, vr, simx_opmode_oneshot);
			simxSetJointTargetVelocity(clientID, leftMotorHandle, vl, simx_opmode_oneshot);
		}
	}
	while(*stop == 0);
	cout << "The wheelchair now is in the correct position and orientation." << endl;
}
