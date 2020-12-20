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
    target2=-1;
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
	int outIntCount = 19;
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
    target2=outInt[18];
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
	// set the targets position to do the planning
	simxFloat target1Pos[3];
	simxFloat target2Pos[3];
	//experimental shift; 
	if (doorPos!=NULL)
	{
		target1Pos[0]= doorPos[0] - 0.08;
		target1Pos[1]= doorPos[1] - 0.1;
		target1Pos[2]= doorPos[2] -0.1;
		target2Pos[0]= target1Pos[0]+ 0.01;
		target2Pos[1]= target1Pos[1];
		target2Pos[2]= target1Pos[2] - 0.07;
		int result = simxSetObjectPosition(clientID, target1, -1, target1Pos,simx_opmode_oneshot_wait);
		if (result!=simx_return_ok)
			cout<< "ERROR: setKnobPosition target 1 Failed"<< endl;
		result = simxSetObjectPosition(clientID, target2, -1, target2Pos,simx_opmode_oneshot_wait);
		if (result!=simx_return_ok)
			cout<< "ERROR: setKnobPosition target 2 Failed"<< endl;

	}
	else
		cout << "ERROR: setKnobPosition:: doorPos is null"<< endl;
}

void Manipulator::setKnobOrientation(simxFloat doorOri[3])
{
	if (doorOri!=NULL)
	{
		simxFloat target1Ori[3];
		simxFloat target2Ori[3];

		target1Ori[0]=doorOri[0];
		target1Ori[1]= doorOri[1];
		target1Ori[2]= doorOri[2] - 1.57;
		target2Ori[0]=target1Ori[0];
		target2Ori[1]=target1Ori[1]+ 0.03;
		target2Ori[2]=target1Ori[2];

		int result = simxSetObjectOrientation(clientID, target1, -1, target1Ori,simx_opmode_oneshot_wait);
		if (result!=simx_return_ok)
			cout<< "ERROR: setKnobOrientation target1 Failed"<< endl;
		result = simxSetObjectOrientation(clientID, target2, -1, target2Ori,simx_opmode_oneshot_wait);
		if (result!=simx_return_ok)
			cout<< "ERROR: setKnobOrientation target2 Failed"<< endl;
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
//get legths size  
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
// get path size 
int Manipulator::get_pathSize(int pathID)
{
	//inputs: pathID
	int inIntCount =1;
	int inInt = pathID;
	//outputs: path size
	int outIntCount=1;
	int *outInt;
	int size = -1;
	outInt = NULL;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "getPathSize",
											inIntCount, &inInt, 0, NULL, 0, NULL,0,NULL,
											&outIntCount,&outInt, 0 , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outInt!=NULL && *outInt !=-1)
		size = *outInt;
	else 
		cout <<("ERROR: get_pathSize failed.")<< endl; 
	return size; 
}


void Manipulator::follow_path(int pathID)
{
	//Each path point is a robot configuration.
	int l = get_pathSize(pathID);
	cout << "path size = " << l;
	//get the config of all of the joints on the path position 
	int j = 0;
	//inputs: point on path (j)
	//outputs: config of all joints in this point
	int inIntCount = 2;
	int outFloatCount = 6;
	int inInt[2] = {j,pathID};
	float *outFloat;
	outFloat=NULL;
	float configs[6]= {-1,-1,-1,-1,-1,-1};
	int result;
	do
	{
			result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "followPath", 
											inIntCount, inInt, 0, NULL, 0, NULL,0,NULL,
											0, NULL, &outFloatCount , &outFloat, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
			if(result== simx_return_ok && outFloat != NULL)
			{
				for(int i =0;i<outFloatCount;i++)
				{
					configs[i]= outFloat[i];
					//cout << "config " << i << " = " << configs[i] << endl;
					//cout << "jh " << i << jh[i] << endl;
					if (configs[i]!=-111) 
						result = simxSetJointTargetPosition(clientID, jh[i],configs[i],simx_opmode_oneshot);
					if(result != simx_return_ok)
						cout<< "something wrong on SetJointTargetVelocity" <<endl;
				}
					
			}
			inInt[0]++;
	}
		while(inInt[0]<l/6);
}

void Manipulator::approach()
{
	//inputs: None
	//outPuts: pathFound - 0=true, 1=false ; 
	int outIntCount=1;
	int *outInt;
	outInt=NULL; 
	int found;
	//	int outStringCount = 2; 
	//	simxChar *outString;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "approach",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result==simx_return_ok && outInt!=NULL)
	{
		found = *outInt;
		if(found == 1)
			cout << "Approach path found."<< endl;
	}	

}

void Manipulator::close_hand()
{
	//inputs: None 
	//output: None
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "closeHand", 
										0, NULL, 0, NULL, 0, NULL,0,NULL,
										NULL, NULL, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok)
		cout << "Hand closed." << endl;

}

void Manipulator::open_door()
{
	//inputs: None
	//outPuts: pathFound - 0=true, 1=false ; 
	int outIntCount=1;
	int *outInt;
	outInt=NULL; 
	int found=0;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "openDoorMotionPlanning",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result==simx_return_ok && outInt!=NULL)
	{
		found = *outInt;
		if(found == 1)
			cout << "Open door path found."<< endl;
	}	
}

void Manipulator::push()
{
	//inputs: None
	//outPuts: pathFound - 0=true, 1=false ; 
	int outIntCount=1;
	int *outInt;
	outInt=NULL; 
	int found=0;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "pushMotionPlanning",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result==simx_return_ok && outInt!=NULL)
	{
		found = *outInt;
		if(found == 1)
			cout << "Push door path found."<< endl;
	}	
}

void Manipulator::open_hand()
{
	//inputs: None 
	//output: None
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "openHand", 
										0, NULL, 0, NULL, 0, NULL,0,NULL,
										NULL, NULL, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok)
		cout << "Hand opened." << endl;

}