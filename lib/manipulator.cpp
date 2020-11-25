#include "manipulator.h"
using namespace cv;
using namespace std;
using namespace glm;
Manipulator::Manipulator()
{
	clientID = -1;
	int jh[] = {-1,-1,-1,-1,-1,-1};
	int jt[] = {-1,-1,-1,-1,-1,-1};
	jacoHandle= -1;
   	ikTarget=-1;
    ikTip=-1;
    ikGroup= -1;
    target0= -1;
    target1=-1;
}

Manipulator::Manipulator(int ID)
{
	clientID = ID;
	int jh[] = {-1,-1,-1,-1,-1,-1};
	int jt[] = {-1,-1,-1,-1,-1,-1};
	
	// inputs: None 
	// outputs: joint handles, joint types, 
	//			jacoHandle, ikTarget, ikTip, ikGroup, target0, target1;			
	int outIntCount = 18;
	int *outInt; 
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "initializeJaco",
											0, NULL,0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);	
	for(int i=0; i<6; i++)
	{
		//joint handles
		jh[i] = outInt[i];
		cout << "Joint Connected " << jh[i]<<endl;
	}	
	for(int i=0;i<6;i++)
		jt[i] = outInt[i+6]; // joint type
	jacoHandle= outInt[12];
   	ikTarget=outInt[13];
    ikTip=outInt[14];
    ikGroup= outInt[15];
    target0= outInt[16];
    target1=outInt[17];
}

int Manipulator::get_JacoHandle()
{
	return jacoHandle;
}
int Manipulator::get_target1Handle()
{
	return target1;
}

void Manipulator::setKnobPosition(simxFloat doorPos[3])
{
	simxFloat targetPos[3];
	//experimental shift; 
	targetPos[0]= doorPos[0] - 0.06;
	targetPos[1]= doorPos[1] - 0.02;
	targetPos[2]= doorPos[2] -0.13;
	simxSetObjectPosition(clientID, target1, -1, targetPos,simx_opmode_oneshot);
}
void Manipulator::setKnobOrientation(simxFloat doorOri[3])
{
	simxSetObjectOrientation(clientID, target1, -1, doorOri,simx_opmode_oneshot);
}

void Manipulator::execute_motion()
{
	//execute the motion related function inside the simulation. 
	//inputs: Joint handles, Joint types. 
	//output: Motion executed. 
	int inIntCount=0;
	int *inInt= NULL;
	int
	result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "motionPlanning",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											NULL, NULL, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);	
}