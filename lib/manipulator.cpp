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
		targetPos[0]= doorPos[0] + 0.05;
		targetPos[1]= doorPos[1] - 0.1;
		targetPos[2]= doorPos[2] -0.07;
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
	//		   pathString(name of custom block which has the path table); lengthString (name of custom block which has the length table)
	int outIntCount=1;
	int *outInt=NULL; 
	int outStringCount = 2; 
	simxChar *outString;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "motionPlanning",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, NULL , NULL, &outStringCount, &outString, NULL, NULL, simx_opmode_oneshot_wait);
	if(result==simx_return_ok && *outInt==0 && outString!=NULL)
	{
		cout << "Motion path found."<< endl;
		path = outString[0];
		length = outString[1];
		cout << "path Object name =" << path << endl; 
		cout << "length Object name =" << length << endl; 
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
void Manipulator::get_jointsUpperVelocityLimits()
{
	//inputs: joint handles; 
	//outputs: joitUpperVelocityLimits, sucess; 
	int inIntCount = 6; 
	int *inInt = jh; 
	int outIntCount = 1; 
	int *outInt; 
	int outFloatCount= 6; 
	float *outFloat; 
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "getJointsUpperVelocityLimits",
											inIntCount, inInt, 0, NULL, 0, NULL,0,NULL,
											&outIntCount,&outInt, &outFloatCount , &outFloat, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outFloat!=NULL && *outInt ==1)
	{
		for(int i =0; i<6;i++)
		{
			jointsUpperVelocityLimits[i] = outFloat[i];
			//cout << "joitUpperVelocityLimits= " << jointsUpperVelocityLimits[i]<<endl;
		}
	}
	else 
		cout << "ERROR: get_jointsUpperVelocityLimits failed." << endl;	
}
// get rml handle to calculate the steps of the motion 
int Manipulator::get_rmlHandle()
{
	//inputs: length string
	//outputs: rml handle
	int inStringCount = 1;
	simxChar *inString; 
	inString[0] = length;
	cout << "inString= "<< inString[0] << endl;
	int outIntCount = 1; 
	int *outInt = NULL;
	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "getRmlHandle",
											0, NULL, 0, NULL, inStringCount, inString,0,NULL,
											&outIntCount,&outInt, NULL , NULL, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outInt !=NULL)
	{
		cout << "rmlHandle = " << endl;
	}

}
// calculate the velocity correction factor
void Manipulator::calculate_velocity_factor()
{
	//inputs: path, lengths, jh, velCorrection, . 
	//outputs: rmax, res
	int outIntCount =1; 
	int *outInt;
	int outFloatCount = 1; 
	float *outFloat;

/*	int result = simxCallScriptFunction(clientID, "Jaco", sim_scripttype_childscript, "calculateCorrectionFactor",
											0, NULL, 0, NULL, 0, NULL,0,NULL,
											&outIntCount, &outInt, &outFloatCount , &outFloat, NULL, NULL, NULL, NULL, simx_opmode_oneshot_wait);
	if(result == simx_return_ok && outFloat!=NULL && outInt !=NULL)
	{
		cout << "rmax=" << *outFloat<< endl;
		cout << "res=" << *outInt<<endl;
	}
*/
	float dt = get_simStepTime();
	cout << "dt = " << dt <<endl;
	get_jointsUpperVelocityLimits();
	float velCorrection =1; 
	cout << "rmlHandle = " << get_rmlHandle() << endl;
	
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