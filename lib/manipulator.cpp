#include "manipulator.h"
using namespace cv;
using namespace std;
using namespace glm;
Manipulator::Manipulator()
{
	clientID = -1;
	for(int i =0; i<6;i++)
	{
		jh[i] =-1;
		jt[i]=-1;
		jointsUpperVelocityLimits[i]=-1;
	}
	jacoHandle= -1;
   	ikTarget=-1;
    ikTip=-1;
    ikGroup= -1;
    target0= -1;
    target1=-1;
    rmlHandle = -1;
}

Manipulator::Manipulator(int ID)
{
	clientID = ID;
	for(int i =0; i<6;i++)
	{
		jh[i] =-1;
		jt[i]=-1;
		jointsUpperVelocityLimits[i]=-1;
	}
	
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
		jt[i] = outInt[i+6]; // joint type
		cout << "Joint Connected " << jh[i]<<endl;
	}	
	jacoHandle= outInt[12];
   	ikTarget=outInt[13];
    ikTip=outInt[14];
    ikGroup= outInt[15];
    target0= outInt[16];
    target1=outInt[17];
    rmlHandle = -1;
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
	if (doorPos!=NULL)
	{
		targetPos[0]= doorPos[0] - 0.01;
		targetPos[1]= doorPos[1] - 0.12;
		targetPos[2]= doorPos[2] -0.1;
		int result = simxSetObjectPosition(clientID, target1, -1, targetPos,simx_opmode_oneshot_wait);
		if (result!=simx_return_ok)
			cout<< "ERROR: setKnobPosition Failed"<< endl;
	}
	else
		cout << "ERROR: setKnobPosition:: doorPos is null"<< endl;
}

void Manipulator::setKnobOrientation(simxFloat doorOri[3])
{
	if (doorOri!=NULL)
	{
		simxFloat targetOri[3];
		targetOri[0]=doorOri[0];
		targetOri[1]= doorOri[1];
		targetOri[2]= doorOri[2] - 1.57;

		int result = simxSetObjectOrientation(clientID, target1, -1, targetOri,simx_opmode_oneshot_wait);
		if (result!=simx_return_ok)
			cout<< "ERROR: setKnobOrientation Failed"<< endl;
	}
	else
		cout << "ERROR: setKnobOrientation:: doorOri is null"<< endl;
}

// Motion planning remote API related methods: 
// calculate path. 
void Manipulator::motion_planning()
{
	//inputs: None
	//outPuts: pathFound - 0=true, 1=false ; 
	int outIntCount=1;
	int *outInt;
	outInt=NULL; 
	int found;
//	int outStringCount = 2; 
//	simxChar *outString;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "motionPlanning",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result==simx_return_ok && outInt!=NULL)
	{
		found = *outInt;
		if(found == 1)
			cout << "Motion path found."<< endl;
	}	
}

// get simulation step time to use on motion execution. 
float Manipulator::get_simStepTime()
{
	int outFloatCount = 1;
	float *outFloat;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "getSimStep",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											NULL, NULL, &outFloatCount , &outFloat, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outFloat!=NULL)
		return *outFloat;
	cout << "ERROR: get_simStepTime failed."<<endl;
}
// get the max vel of each joint 
void Manipulator::get_jointsUpperVelocityLimits(float jointsUpperVelocityLimits[6])
{
	//inputs: joint handles; 
	//outputs: joitUpperVelocityLimits, sucess; 
	int inIntCount = 1; 
	int *inInt; 
	int outIntCount = 1; 
	int *outInt; 
	int outFloatCount= 1; 
	float *outFloat = NULL;
	int result;
	int i =0;
	do
	{
		*inInt = jh[i];
		result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "getJointsUpperVelocityLimits",
											inIntCount, inInt, 0, NULL, 0, NULL,0,NULL,
											&outIntCount,&outInt, &outFloatCount , &outFloat, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
		if(result == simx_return_ok && outFloat!=NULL && *outInt ==1)
		{
				jointsUpperVelocityLimits[i] = *outFloat;
				cout << "joitUpperVelocityLimits= " << jointsUpperVelocityLimits[i]<<endl;
		}
		else 
			cout << "ERROR: get_jointsUpperVelocityLimits failed." << endl;	
		i++;
	}
	while(i<6);
}
// get rml handle to calculate the steps of the motion 
int Manipulator::get_rmlHandle(float velCorrection)
{
	//inputs: velCorrection
	//outputs: rml handle
	int inIntCount =1;
	int *inInt; 
	*inInt = 1;
	int outIntCount = 1; 
	int *outInt;
	int handle;
	outInt = NULL;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "getRmlHandle",
											inIntCount, inInt, 0, NULL, 0, NULL,0,NULL,
											&outIntCount,&outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outInt !=NULL)
	{
		cout << "rmlHandle = " << *outInt << endl;
		handle = *outInt;
		return handle;
	}
	cout << "ERROR: get_rmlHandle failed. "<< endl;
	return -1;
}
// execute a rmlStep 
int Manipulator:: execute_rmlStep(float posVelAccel[3], int rmlHandle)
{
	//inputs: rmlHandle
	//output: res(0: Final state not reached yet; 1: final state reached)
	int inIntCount = 1;
	int *inInt;
	inInt[0]= rmlHandle;
	int outIntCount = 1;
	int *outInt;
	int outFloatCount = 3;
	float *outFloat;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "executeRmlStep",
											inIntCount, inInt, 0, NULL, 0, NULL,0,NULL,
											&outIntCount,&outInt, &outFloatCount , &outFloat, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outInt !=NULL && outFloat !=NULL)
	{
		cout << "res = " << outInt[0] << endl;
		for(int i=0; i<outFloatCount;i++)
		{
			posVelAccel[i]=outFloat[i];
			cout << posVelAccel[i] << endl;
		}
		return outInt[0];
	}
	cout << "ERROR: execute_rmlStep failed."<< endl;
	return -1;
}
//get path legth 
int Manipulator::get_lengthSize()
{
	//inputs: None 
	//Outputs: #length 
	int outIntCount=1;
	int *outInt;
	outInt = NULL;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "getLengthSize",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount,&outInt, 0 , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outInt!=NULL && *outInt !=-1)
		return *outInt;
	cout <<("ERROR: get_lengthSize failed.")<< endl; 
	return -1; 
}
// calculate the velocity correction factor
void Manipulator::calculate_velocity_factor()
{
	int i=0; //path
	int j=0; //joint
	int lengthSize = get_lengthSize();
	cout << "lengthSize=" << lengthSize << endl;
	int inIntCount = 4; 
	int inInt[4] = {-1,-1,-1, -1};
	int inFloatCount = 1; 
	float velCorrection= 1.0;
	float inFloat;
	inFloat = velCorrection;
	int outIntCount =1; 
	int *outInt = 0;
	int outFloatCount = 1; 
	float *outFloat;
	outFloat= NULL;
	rmlHandle = get_rmlHandle(velCorrection);
	inInt[3]= rmlHandle;
	int result;
	float r=0;
	int res=0;
	for(i=0; i<lengthSize-2 && res==0; i++)
	{
		for(j=0;j<6 && res==0;j++)
		{
			inInt[0] = jh[j];
			cout << "j= " << j;
			inInt[1] = i+1;
			inInt[2] = j+1;
			result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "calculateCorrectionFactor2",
											inIntCount, inInt, inFloatCount, &inFloat, 0, NULL,0,NULL,
											&outIntCount,&outInt, &outFloatCount, &outFloat, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
			if (result == simx_return_ok && outFloat != NULL)
				r = *outFloat;
				res = *outInt;
				// if(res==1)
				//{
				//	cout << "get out 1 " << endl;
				//	break;
				//}

				cout << "r = " <<r<< endl;
		}
		//if(res==1)
		//{	
		//	cout << "get out 2" << endl;
		//	break;
		//}
	}
	cout << "Get ouuttt" << endl;
}

void Manipulator::execute_motion()
{
	//execute the motion related function inside the simulation.  
	// verify if the hand is close to the target. 
	int inIntCount=0;
	int *inInt= NULL;
	int outIntCount=1;
	int *outInt=NULL;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "motionPlanning",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											0, NULL, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);	
	if(result==simx_return_ok)
	{
		do
		{
			result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "finishMotion", 
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
			cout << "Not so close yet. close = " << *outInt <<endl;
		}
		while(outInt!= NULL &&*outInt ==1);		
	}
	if(outInt!=NULL &&*outInt ==0)
	{
		cout<< "Now the hand is close to the knob and the orientation is correct."<<endl;		
	}
}