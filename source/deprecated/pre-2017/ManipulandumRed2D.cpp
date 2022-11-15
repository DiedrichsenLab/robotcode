/////////////////////////////////////////////////////////////////////////////
// ManipulandumRed2D.cpp 
//
// written for the 2DoF variant of the 3DoM pantograph robot under a s626 io board 
//
// Derived from Manipulandum.h originally by Joern Diedrichsen, 2009
// j.diedrichsen@ucl.ac.uk
// 
// Modified by Nick Roach and Julius Klein, 2011
// n.roach@imperial.ac.uk, j.klein@imperial.ac.uk
//
/////////////////////////////////////////////////////////////////////////////

// include corresponding header file
#include "ManipulandumRed2D.h"
#include "TextDisplay.h"
#include "S626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 
using namespace std; 
extern S626sManager s626;
extern TextDisplay tDisp;
ATI_DAQ gATI;					///< ATI Force transducer 
// defining constants for Manipulandum 
//#define CONST_abs 1638400			// How many counts for full circle in 100x interpolation at encoder and 4x on board
#define CONST_abs 65536			// How many counts for full circle in 4x interpolation at encoder and 4x on board
#define CONST_abs_wrist 40000	// How many counts for full circle for wrist enc
#define CONST_LEN 0.358
#define Pi 3.141592654


////////////////////////////////////////////////////////////////////////////
/// The Constructor really doesn't do anything 
///  
////////////////////////////////////////////////////////////////////////////
ManipulandumRed2D::ManipulandumRed2D()  {
	isFirstUpdate=1;
}  

////////////////////////////////////////////////////////////////////////////
/// Destructor does nothing 
///  Should probably free the channels allocated on the s626 board 
////////////////////////////////////////////////////////////////////////////
ManipulandumRed2D::~ManipulandumRed2D()  {

	gATI.stop();
	
}  

////////////////////////////////////////////////////////////////////////////
/// Reads a configuration file for the robot and initializes io channels
/// \param paramfile Parameter files
/// The file contains
///  
/// \li  Board for Shoulder Encoder 
/// \li  Channel for Shoulder Encoder 
/// \li  Direction for Shoulder Encoder 
/// \li  Board for Elbow Encoder 
/// \li  Channel for Elbow Encoder 
/// \li  Direction for Elbow Encoder 
/// \li  Board for Wrist Encoder 
/// \li  Channel for Wrist Encoder 
/// \li  Direction for Wrist Encoder 
/// \li  Board for DA-piston forces 
/// \li  Offset for Shoulder encoder [deg]
/// \li  Offset for Elbow Enconder [deg]
/// \li  Offset for Wrist Enconder [deg]
/// \li  Upper arm left [cm]
/// \li  Upper arm Right [cm]
/// \li  Lower arm Left [cm]
/// \li  Lower arm Right [cm]
/// \li  Robot distance [cm]
/// \li  Length of wrist[cm]
/// \li  Tranformation Nm to V at shoulder
/// \li  Tranformation Nm to V at elbow
/// \li  Tranformation Nm to V at wrist
/// \li  Angle of ATI transducer to arm 
/// \li	 Shoulder safe volt limit
/// \li	 Wrist safe volt limit
/// \li	 Right Shoulder -ive Limit Angle [Deg]
/// \li	 Right Shoulder +ive Limit Angle [Deg]
/// \li	 Left Shoulder -ive Limit Angle [Deg]
/// \li	 Left Shoulder +ive Limit Angle [Deg]
/// \li	 Wrist -ive Limit Angle [Deg]
/// \li	 Wrist +ive Limit Angle [Deg]
/// \li	 End point -x envelope limit [cm]
/// \li	 End point +x envelope limit [cm]
/// \li	 End point -y envelope limit [cm]
/// \li  End point +y envelope limit [cm]
///
////////////////////////////////////////////////////////////////////////////
void ManipulandumRed2D::init(string paramfile)
{

	double tempDouble;
	string s; 
	paramfilename=paramfile; 
	ifstream inputFile(paramfilename.c_str(),ios::in);
	if(inputFile ==0){
		cout<<"ManipulandumRed2D.init: Robot parameter file could not be opened\n";
		exit(-1);
	} else{
		inputFile>>board_cnt[0]; getline(inputFile,s);		// 1 Board for Shoulder Encoder 1
		inputFile>>channel[0]; getline(inputFile,s);		// 2 Channel for Shoulder Encoder 2
		inputFile>>dir_cnt[0]; getline(inputFile,s);		// 3 Direction for Shoulder Encoder 3 
		inputFile>>board_cnt[1]; getline(inputFile,s);		// 4 Board for Shoulder  Encoder 4
		inputFile>>channel[1]; getline(inputFile,s);		// 5 Channel for Shoulder  Encoder 5
		inputFile>>dir_cnt[1]; getline(inputFile,s);		// 6 Direction for Shoulder  Encoder 6
		inputFile>>board_cnt[2]; getline(inputFile,s);		// 7 Board for Wrist Encoder 7
		inputFile>>channel[2]; getline(inputFile,s);		// 8 Channel for Wrist Encoder 8
		inputFile>>dir_cnt[2]; getline(inputFile,s);		// 9 Direction for Wrist Encoder 9
		inputFile>>board_da; getline(inputFile,s);			// 10 Board for DA-piston forces 10
		inputFile>>da_offset; getline(inputFile,s);			// 11 Offset for DA channels 11
		inputFile>>offset[0]; getline(inputFile,s);			// 12 Offset for right Shoulder  encoder [count] 12
		inputFile>>offset[1]; getline(inputFile,s);			// 13 Offset for left Shoulder  encoder [count] 13 
		inputFile>>offset[2]; getline(inputFile,s);			// 14 Offset for WristEnconder [count] 14
		inputFile>>jointLengthUpper[0]; getline(inputFile,s);	// 15 Upper arm length Left 15
		inputFile>>jointLengthUpper[1]; getline(inputFile,s);	// 16 Upper arm length Right 16
		inputFile>>jointLengthLower[0]; getline(inputFile,s);	// 17 Lower arm length Left 17
		inputFile>>jointLengthLower[1]; getline(inputFile,s);	// 18 Lower arm length right 18
		inputFile>>robotDistance; getline(inputFile,s);			// 19 Distance of the two robots 19 
		inputFile>>wristLength; getline(inputFile,s);		// 20 Length of Wrist [cm] 20
		inputFile>>torque2volts[0];	getline(inputFile,s);       // 21 Tranformation Nm to V at Right Shoulder  21
		inputFile>>torque2volts[1];	getline(inputFile,s);       // 22 Tranformation Nm to V at Left Shoulder  22
		inputFile>>torque2volts[2];	getline(inputFile,s);       // 23 Tranformation Nm to V at wrist 23
		inputFile>>ang_ati2arm;getline(inputFile,s);			// 24 Angle of ATI transducer to arm 24
		inputFile>>shouldVoltLimit;getline(inputFile,s);		//25 Shoulder safe volt limit
		inputFile>>wristVoltLimit;getline(inputFile,s); 		//26 Wrist safe volt limit
		inputFile>>tempDouble; getline(inputFile,s); rShouldLimitAngle[0]=tempDouble*(Pi/180);   //27 Right Shoulder -ive Limit Angle [Deg]
		inputFile>>tempDouble; getline(inputFile,s); rShouldLimitAngle[1]=tempDouble*(Pi/180);   //28 Right Shoulder +ive Limit Angle [Deg]
		inputFile>>tempDouble; getline(inputFile,s); lShouldLimitAngle[0]=tempDouble*(Pi/180);   //29 Left Shoulder -ive Limit Angle [Deg]
		inputFile>>tempDouble; getline(inputFile,s); lShouldLimitAngle[1]=tempDouble*(Pi/180);   //30 Left Shoulder +ive Limit Angle [Deg]	
		inputFile>>tempDouble; getline(inputFile,s); wristLimitAngle[0]=tempDouble*(Pi/180);	    //31 Wrist -ive Limit Angle [Deg]
		inputFile>>tempDouble; getline(inputFile,s); wristLimitAngle[1]=tempDouble*(Pi/180);	    //32 Wrist +ive Limit Angle [Deg]
	
		if (inputFile.bad() || inputFile.eof()){
			cout<<"ManipulandumRed2D.init: check parameter file in wrong format\n";
			exit(-1);
		} 
	}  	
	
	s626.initCounter(channel[0],board_cnt[0]);  // reset s626 encoder counters 
	s626.initCounter(channel[1],board_cnt[1]); 
	s626.initCounter(channel[2],board_cnt[2]);

	wdt_pulse_state=0;
	first_cycle=1;
	
	motorPos[0] = Vector2D(robotDistance/2,0);   // Position of the left robot motor 
	motorPos[1] = Vector2D(-robotDistance/2,0);   // Position of the right robot motor 

	safeVoltGain=0;				// position cut out voltage gain
	safeVoltRiseTime=0.5;		// position cut out voltage gain recovery duartion in seconds (set to 500ms)

	//init force values
	force=Vector2D(0 ,0);
	forceProd=Vector2D(0,0);
	filter.init(Vector2D(0,0));

	wristfilter.init(Vector2D(0,0));

	torqueDemanded=Vector3D(0,0,0);

	//init wrist pos contol vars
	wristAPTerm=0; //proptional term
	wristAITerm=0; //integral term
	wristADTerm=0; //integral term
	wristAErrLast=0; //last wrist error
	wristAError=0; //wrist error
	wristAKp=0.063; //prop gain
	wristAKi=0.0030; //intergral gain
	wristAKd=0.00074; //derivative gain

	wristADemandTorque=0; //demand torque signal to wrist motor  
	wristASetAngle=0; //angle set point
	wristFBOn=0; //pid active

	outputEnable = 0; //Set Master output enable to off

	gATI.init(1000, "C:\\robot\\calib\\FT10172.cal"); //initialise ati data aquisition

	s626.outDIO(0, 0, board_cnt[0]); //confirm first four DIO line as inputs by diabling output transistor
	s626.outDIO(1, 0, board_cnt[0]);
	s626.outDIO(2, 0, board_cnt[0]);
	s626.outDIO(3, 0, board_cnt[0]);

	gATI.start(); //start ati data aquisition
} 


//////////////////////////////////////////////
/// Set the current position of the robot to be (0,0) 
//////////////////////////////////////////////
void ManipulandumRed2D::recenter(){ 
	positionOffset=rawPosition;	

};	  

//////////////////////////////////////////////
/// Set the current position of the robot to be (0,0) 
//////////////////////////////////////////////
void ManipulandumRed2D::recenter(const Vector2D &x){ 
	positionOffset=x;	
};	  

//////////////////////////////////////////////
/// Gets the raw counter values by reference 
/// \param sh this is where the shoulder counter will be dumped
/// \param el this is where the elbow counter will be dumped
//////////////////////////////////////////////
void ManipulandumRed2D::getCounter(long &rSh,long &lSh,long &wr){ 

	rSh=rShCnt;
	lSh=lShCnt;
	wr=wrCnt;
} 	  

//////////////////////////////////////////////
/// Gets the calibrated shoudler and wrist motor angles in radians
/// \param theAngles 3D Vector by which angles are passed
//////////////////////////////////////////////
void ManipulandumRed2D::getAngles(Vector3D &theAngles){

	theAngles.x[0]=theta.x[0];
	theAngles.x[1]=theta.x[1];
	theAngles.x[2]=theta.x[2];
	
}

//////////////////////////////////////////////
/// Gets the x and y end point position in cm via forwardKinePos
/// \param thePos 2D Vector by which co-ordinates are passed
//////////////////////////////////////////////

void ManipulandumRed2D::getPosition(Vector2D &thePos)			
{
	thePos[0]=position[0];
	thePos[1]=position[1];
}

//////////////////////////////////////////////
/// Gets the Kalman filter estimated x and y end effector velocity in cm/s 
/// \param thePos 2D Vector by which velocity is passed
//////////////////////////////////////////////

void ManipulandumRed2D::getVelocity(Vector2D &theVel)				
{
	theVel[0]=velocityFilt[0];
	theVel[1]=velocityFilt[1];
}

//////////////////////////////////////////////
/// Current calibrated force measurements from ATI transducer in sensor frame (N)
/// \return 3D vector giving force in sensor coordinates 
//////////////////////////////////////////////

Vector3D ManipulandumRed2D::getLocalFTForce()  
{
	return(localFTForce);
}

//////////////////////////////////////////////
/// Current calibrated force measurements from ATI transducer in sensor frame (N)
/// \param theForce 3D Vector by which force data is passed
//////////////////////////////////////////////

void ManipulandumRed2D::getLocalFTForce(Vector3D &theFTForce)   
{
	theFTForce[0]=localFTForce[0];
	theFTForce[1]=localFTForce[1];
	theFTForce[2]=localFTForce[2];
}

//////////////////////////////////////////////
/// Current calibrated force measurements from ATI transducer in glocal robot frame (N)
/// (sensor frame rotated by wrist angle)
/// \return theForce 3D Vector giving force in robot coordinates
//////////////////////////////////////////////

Vector3D ManipulandumRed2D::getGlobalFTForce() 
{
	return(globalFTForce);
}

//////////////////////////////////////////////
/// Current calibrated force measurements from ATI transducer in glocal robot frame (N)
/// (sensor frame rotated by wrist angle)
/// \param theForce 3D Vector by which force data is passed
//////////////////////////////////////////////

void ManipulandumRed2D::getGlobalFTForce(Vector3D &theFTForce)
{
	theFTForce[0]=globalFTForce[0]; // force measured by ft in global frame
	theFTForce[1]=globalFTForce[1];
	theFTForce[2]=globalFTForce[2];
}

//////////////////////////////////////////////
/// calculate robot local end-effector position in cm given elbow angles in rads
/// 
/// This function calculates the position coordinates x and y in function of
/// the given Left and Right encoder angles (Theta1, Theta5) [degree].
///
/// Geometry of the pantograph
///
///         P1  (O) a5  (O) P5
///             /         \
///         a1 /           \ a4
///           /             \
///      P2   o.... Ph .....o  P4
///             \         /
///           a2  \     /a3
///                 \o/
///                  P3=(x3,y3)
///
///************** ROBOT FRAME (definition) ******************
///
///    [0,0] is in the middle of "a5" (midpoint between P1 and P5)
///    [X is positive to the right]
///    [Y is positive upward]
///
/// ************* GLOBAL FRAWE (definition) ******************
///		The same as above, but (0,0) is shifted by positionOffset.
///     Use the command recenter(Vector2D(x,y)) to reset the origin
/// \param shoulderAngle 2D Vector containing Angles of robot Shoulder motors in radians (Right then Left)
/// \return 2D Vector giving x,y position of endpoint in Robot coordinates (cm)
//////////////////////////////////////////////
Vector2D ManipulandumRed2D::forwardKinePos(Vector3D shoulderAngle){

	// Intermediate variables 
	double P2P4;  //'tween elbow vector
	double P2Ph;  //elbow1 to midpoint
 	double P3Ph; //elbow2 to end point
	Vector2D Ph; //midpoint 

	Vector2D endPos; //temp storage for end effector pos

	//------------------------Forward kinematic model for end effector position--------------------------
	elbowPos[0] = motorPos[0] - Vector2D(jointLengthUpper[0]*cos(shoulderAngle[0]),jointLengthUpper[0]*sin(shoulderAngle[0]));
	elbowPos[1] = motorPos[1] - Vector2D(jointLengthUpper[1]*cos(shoulderAngle[1]),jointLengthUpper[1]*sin(shoulderAngle[1]));
	
	P2P4 = norm(elbowPos[0]-elbowPos[1]); 
	P2Ph = 0.5*P2P4;
	Ph = elbowPos[1] + (elbowPos[0]-elbowPos[1])*(P2Ph/P2P4);
	P3Ph = sqrt(jointLengthLower[0]*jointLengthLower[0]-P2Ph*P2Ph);
	
	// robot end effector postion, in Cartesian coord centered at robot shoulder joint
	endPos[0] =  (Ph[0] + (P3Ph/P2P4)*(elbowPos[0][1]-elbowPos[1][1]));
	endPos[1] =  (Ph[1] - (P3Ph/P2P4)*(elbowPos[0][0]-elbowPos[1][0]));

	return (endPos);

}

//////////////////////////////////////////////
/// Main routine that does all the heavy work with update frequency 
/// \param dt time in ms since last update 
//////////////////////////////////////////////
void ManipulandumRed2D::update(double dt){

	//safety checks
	checkLimitSwitch(); //check limits
	checkMotorHardEnable(); //check motors are enabled

	pollWDT(); //if output is active then 

	//servo loop
	currentRate=dt; //update current cycle rate 

	lastPosition=position;  // Store the last position 
	
	rShCnt=s626.getCounter(channel[0],board_cnt[0]);  // Get the counters 
	lShCnt=s626.getCounter(channel[1],board_cnt[1]);
	wrCnt=s626.getCounter(channel[2],board_cnt[2]);
    
	theta[0]=  dir_cnt[0]*(((double)(rShCnt)-offset[0])/(CONST_abs/360))*(Pi/180);      // calculate the angles and convert to radians
	theta[1]=  dir_cnt[1]*(((double)(lShCnt)-offset[1])/(CONST_abs/360))*(Pi/180);
	theta[2]=  dir_cnt[2]*(((double)(wrCnt)-offset[2])/(CONST_abs_wrist/360))*(Pi/180);

	rawPosition=forwardKinePos(theta); // calculate robot end effector position 

	position=rawPosition-positionOffset; //apply centre offset to pos (set by recentre)
	
	velocity=(position-lastPosition)/dt; // calculate raw velocity 

	filter.update(position,dt); // update Kalman filter with raw position and time 
	filter.position(positionFilt); // retrieve Kalman filter estimated position 
	filter.velocity(velocityFilt); // retrieve Kalman filter estimated velocity 
	
	gATI.update(); //get ATI FT readings

	localFTForce[0]=gATI.force[0]; //copy local force data 
	localFTForce[1]=gATI.force[1];
	localFTForce[2]=gATI.force[2];

	Vector2D tempFT=ati2World(Vector2D(localFTForce[0],localFTForce[1])); //Rotate force readings to robot frame

	globalFTForce[0]=tempFT[0]; //copy global force data
	globalFTForce[1]=tempFT[1];
	globalFTForce[2]=localFTForce[2];

	updateWristControl(dt); //Update wrist controller 
	
	// JD: 10/4/12: Update should only do the sensory side 
	// of things. The program needs to then compute the force and 
	// call updateOutput. Otherwise the effective update will be 1ms delayed
	// Thus, it needs to be a public funtion and be called in the end of 
	// update haptics 
	// updateOutput(); //Send all torques to output
	
} 

////////////////////////////////////////////////////////////////////////////////
/// Clamps voltage output for 'safer' testing
/// 
////////////////////////////////////////////////////////////////////////////////
//clamps volt limits
void ManipulandumRed2D::checkVoltLimits(void){
	//limits
	if (volts[0]>shouldVoltLimit) {volts[0]=shouldVoltLimit;cout<<"shoulder voltage limit active!"<<endl;}
	if (volts[0]<-shouldVoltLimit) {volts[0]=-shouldVoltLimit;cout<<"shoulder voltage limit active!"<<endl;}
	if (volts[1]>shouldVoltLimit) {volts[1]=shouldVoltLimit;cout<<"elbow voltage limit active!"<<endl;}
	if (volts[1]<-shouldVoltLimit) {volts[1]=-shouldVoltLimit;cout<<"elbow voltage limit active!"<<endl;}
	if (volts[2]>wristVoltLimit) {volts[2]=wristVoltLimit;cout<<"wrist voltage limit active!"<<endl;}
	if (volts[2]<-wristVoltLimit) {volts[2]=-wristVoltLimit;cout<<"wrist voltage limit active!"<<endl;}
}


////////////////////////////////////////////////////////////////////////////////
/// Cuts drive when position is outside desired envelope, ramps force back up on return
/// 
////////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::checkPosLimits(void){				//check position is in operating range

	int armEnvFlag=1;

	//wrist angle soft limit
	if (theta[2]<=wristLimitAngle[0]) armEnvFlag=0; 
	if (theta[2]>=wristLimitAngle[1]) armEnvFlag=0;

	//shoudler angle soft limit
	if (theta[0]<=rShouldLimitAngle[0]) armEnvFlag=0; 
	if (theta[0]>=rShouldLimitAngle[1]) armEnvFlag=0; 
	if (theta[1]<=lShouldLimitAngle[0]) armEnvFlag=0; 
	if (theta[1]>=lShouldLimitAngle[1]) armEnvFlag=0; 

	
	if (armEnvFlag==0) //soft cut power 
	{
		safeVoltGain=0;
	}
	else
	{
	
		safeVoltGain=safeVoltGain+((s626.getUpdateRate()*0.001)/safeVoltRiseTime);
		if (safeVoltGain>=1)
		{
			safeVoltGain=1;
		}

	}


	//check bounds for out of range values!!
	if (safeVoltGain>1) safeVoltGain=1;
	if (safeVoltGain<0) safeVoltGain=0;

	safeVoltGainOut=safeVoltGain; // copy volt gain to allow for read only interegation
 
	volts[0]=volts[0]*safeVoltGain;
	volts[1]=volts[1]*safeVoltGain;
	volts[2]=volts[2]*safeVoltGain;
			
}

////////////////////////////////////////////////////////////////////////////////
/// Sets the torque output for each motor in Nm 
///	Converts the value held in torqueDemanded to volts by motor torque constant torque2volts
/// torque2volts set in calibration file
////////////////////////////////////////////////////////////////////////////////
void ManipulandumRed2D::updateOutput(){

	torqueDemandedOut=torqueDemanded; //copy of torque for interrogation

	volts[0]=torqueDemanded[0]*torque2volts[0]; 
	volts[1]=torqueDemanded[1]*torque2volts[1];  
	volts[2]=torqueDemanded[2]*torque2volts[2];  

	setVolts(volts[0],volts[1],volts[2]);

}

////////////////////////////////////////////////////////////////////////////////
/// Sets the voltage output to the Motors directly
/// Actual output is subject to voltage and postion limits imposed by software/hardware
/// \param v0 volts to the right shoulder motor
/// \param v1 volts to the left shoulder motor 
/// \param v2 volts to the wrist motor
////////////////////////////////////////////////////////////////////////////////
void ManipulandumRed2D::setVolts(double v0,double v1,double v2) { 

	volts[0]=v0;
	volts[1]=v1; 
	volts[2]=v2; 

	//enforce limits
	checkPosLimits();
	checkVoltLimits();

	if (outputEnable==1)
	{
		s626.outDA(0+da_offset,volts[0],board_da);
		s626.outDA(1+da_offset,volts[1],board_da);
		s626.outDA(2+da_offset,volts[2],board_da);
	}
	else
	{
		s626.outDA(0+da_offset,0,board_da);
		s626.outDA(1+da_offset,0,board_da);
		s626.outDA(2+da_offset,0,board_da);
	}

} 

////////////////////////////////////////////////////////////////////////////////
/// Sets the torque output for each motor in Nm 
/// Output overidden by calls to setForce
///
/// \param torque 3D Vector of desired motor torques
////////////////////////////////////////////////////////////////////////////////
void ManipulandumRed2D::setTorque(Vector3D torque){

	torqueDemanded[0]=torque[0];
	torqueDemanded[1]=torque[1];
	torqueDemanded[2]=torque[2];

}

////////////////////////////////////////////////////////////////////////////////
/// Calculates the force output for the robot in Eucledian coordinates 
/// using the current state of the robot. This needs to be called again as position of the robot changes.
/// see Jacobian for details of method. Output overidden by calls to setTorque
///
/// \param force Vector of desired forces in N 
////////////////////////////////////////////////////////////////////////////////
void ManipulandumRed2D::setForce(Vector2D theForce){


	Matrix2D Jt_trans(0,0,0,0);

	curJdTheta=Jacobian(Vector2D(theta[0],theta[1])); //Calculate Jacobian for current angles

	Jt_trans=curJdTheta.getTranspose(); //Transpose

	force[0]=theForce[0]; //Copy current force
	force[1]=theForce[1];


	Vector2D tempJTorqueDemanded = Jt_trans.operator * (Vector2D(theForce[0],theForce[1])); //Calculate Torque Demand

	torqueDemanded[1]=tempJTorqueDemanded[0];
	torqueDemanded[0]=tempJTorqueDemanded[1];

}


////////////////////////////////////////////////////////////////////////////////
/// transforms the ATI readings into world coordinates   
/// \param forcereading in ATI coordinate frame
/// \return Force in world coordinate 
/// 
////////////////////////////////////////////////////////////////////////////////
Vector2D ManipulandumRed2D::ati2World(Vector2D atiforce){ 
	 
	Matrix2D ATI2Arm=rotationMatrixRad(ang_ati2arm*(Pi/180)); 
    Matrix2D Arm2Local=rotationMatrixRad(-theta[2]); 
	Vector2D out_force=ATI2Arm*atiforce; 
	out_force=Arm2Local*out_force; 
	
	return out_force; 
} 
////////////////////////////////////////////////////////////////////////////////
/// Needs a textdisplay to be effective.    
/// \param col column in the text display that will be used. 
/// 
////////////////////////////////////////////////////////////////////////////////
	void ManipulandumRed2D::printState(int col){ 
	char manipbuffer[60];

	sprintf(manipbuffer,"Counter: %d %d %d",rShCnt,lShCnt,wrCnt);
	tDisp.setText(manipbuffer,1,col);
	sprintf(manipbuffer,"Theta: %3.2f %3.2f %3.2f",theta[0]/(Pi/180),theta[1]/(Pi/180),theta[2]/(Pi/180));
	tDisp.setText(manipbuffer,2,col);

	sprintf(manipbuffer,"Force: %3.2f %3.2f",force[0],force[1]);
	tDisp.setText(manipbuffer,4,col);
	sprintf(manipbuffer,"Position: %3.2f %3.2f",position[0],position[1]);
	tDisp.setText(manipbuffer,5,col);
	sprintf(manipbuffer,"Set Torque: %3.2f %3.2f %3.2f",torqueDemanded[0],torqueDemanded[1],torqueDemanded[2]);
	tDisp.setText(manipbuffer,6,col);

	sprintf(manipbuffer,"Force: x=%3.4fN y=%3.4fN z=%3.4fN",gATI.force[0],gATI.force[1],gATI.force[2]);
	tDisp.setText(manipbuffer,8,col);
	sprintf(manipbuffer,"Torque: x=%3.4fNmm y=%3.4fNmm z=%3.4fNmm",gATI.torque[0],gATI.torque[1],gATI.torque[2]);
	tDisp.setText(manipbuffer,9,col);

	sprintf(manipbuffer,"Safe v gain=%3.4f, Soft Enable=%i, Wrist PID Active=%i",safeVoltGainOut,outputEnable,wristFBOn);
	tDisp.setText(manipbuffer,11,col);
	sprintf(manipbuffer,"Hard limit switch=%i, Hard enable signal=%i",hardLimitOut,hardEnableOut);
	tDisp.setText(manipbuffer,12,col);
	sprintf(manipbuffer,"Cycle time=%3.4fs",currentRate);
	tDisp.setText(manipbuffer,13,col);

	//sprintf(manipbuffer,"Kp=%3.5fs, Ki=%3.5fs, Kd=%3.5fs, wd=%f",wristAKp,wristAKi,wristAKd, wristADTerm);
	//tDisp.setText(manipbuffer,15,col);

	
	
		
} 

////////////////////////////////////////////////////////////////////////////////
/// Master enable function for motors, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::enableOutput()
{
	resetWDT();
	safeVoltGain=0; //de-ramp output to prevent jump!
	outputEnable=1; //Master Enable DAC output 
	outputEnableOut=outputEnable;  //copy for interogation
	
}		


////////////////////////////////////////////////////////////////////////////////
/// Master disable function for motors, deramps output for safety when called
/// Forces all DAC outputs to 0v
////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::disableOutput()
{
	safeVoltGain=0; //de-ramp output to prevent jump!
	outputEnable=0; //Master Disable DAC output 
	outputEnableOut=outputEnable;  //copy for interogation
}	

////////////////////////////////////////////////////////////////////////////////
/// Enable for wrist angle contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::enableWristControl()
{
	wristFBOn=1; //enable pid wrist pos control 
	safeVoltGain=0; //de-ramp output to prevent jump!

}		


////////////////////////////////////////////////////////////////////////////////
/// Disable for wrist angle contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::disableWristControl()
{
	wristFBOn=0; //enable pid wrist pos control 
	safeVoltGain=0; //de-ramp output to prevent jump!

}	


////////////////////////////////////////////////////////////////////////////////
/// Updates PID contoller for constant wrist angle, ensures ATI is held in constant 
/// 0o orientation with respect to global frame 
////////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::updateWristControl(double dt)
{

	

	if (wristFBOn==1) //is control enabled
	{

		//trap overrange
		if (wristASetAngle<(wristLimitAngle[0]/(Pi/180))) wristASetAngle=wristLimitAngle[0]/(Pi/180);
		if (wristASetAngle>(wristLimitAngle[1]/(Pi/180))) wristASetAngle=wristLimitAngle[1]/(Pi/180);

		//basic PID for wrist
		wristAErrLast=wristAError;
		wristAError=(theta[2]/(Pi/180))-wristASetAngle;

		//filter.position(positionFilt); // retrieve Kalman filter estimated position 
		wristfilter.update(Vector2D(wristAError,0),dt);

		Vector2D wristfiltvel;
		wristfilter.velocity(wristfiltvel);
		
		wristAPTerm=wristAError*wristAKp; //proportional gain
		wristAITerm=wristAITerm+wristAError; //integrate 
		
		//wristADTerm=wristAError-wristAErrLast; //differentiate
		wristADTerm=wristfiltvel[0];
		wristAITerm=wristAITerm*0.98; 	//leak

		wristAITerm=wristAITerm*safeVoltGain; //purge integrator when out of range. Prevents run away when output is dead!!! 

		wristADemandTorque=wristAPTerm+(wristAITerm*wristAKi)+(wristADTerm*wristAKd); //output
			
		torqueDemanded[2]=wristADemandTorque;
	}
	else
	{
		torqueDemanded[2]=0;
	}
}		

////////////////////////////////////////////////////////////////////////////////
/// Jacobian calulates transformation between cartesian force and required motor torques for given
/// configuartion of robot. 
///
/// Use:
///
///		EndEffectorVelocity = J * MotorAngularVelocities
///		MotorAngularVelocities = inverse(J) * EndEffectorVelocity
///		MotorTorques   = transpose(J) *  EndEffectorForces
///
///
///	      [ d_x3/d_thL     d_x3/d_thR  ]
///	  J = [                            ]
///       [ d_y3/d_thL     d_y3/d_thR  ]
///
/// \param shoulderAngles 2D Vector describing current angle of shoulder motors in radians
/// \return 2x2 Matrix giving current values of J
////////////////////////////////////////////////////////////////////////////////

Matrix2D ManipulandumRed2D::Jacobian(Vector2D shoulderAngles) //calculates static motor torques for given motor angles and desired forces
{

	Matrix2D Jdtheta(0,0,0,0);

	double ThR=shoulderAngles[0];
	double ThL=shoulderAngles[1];


Jdtheta[0][0] = -((9849*cos(2*ThL))/40 + (1029*cos(3*ThL))/10 + (7203*cos(2*ThR))/40 - (42189*cos(2*ThL - ThR))/160 + (7203*cos(2*ThL - 2*ThR))/40 
				- (7203*cos(3*ThL - ThR))/40 + (50421*cos(3*ThL - 2*ThR))/640 - (9849*cos(ThL + ThR))/40 - (340977*cos(ThL))/320 + (73059*cos(ThR))/160 
				+ (6321*cos(ThL - ThR))/10 - (36015*cos(ThL - 2*ThR))/128 + (1029*cos(ThL + 2*ThR))/10 - (1029*cos(2*ThL + ThR))/5 
				- (21*sin(ThL)*pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) + 49*cos(ThL 
				- ThR) + 207,0.5))/160 - 32487/40)/(pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) 
			    + 49*cos(ThL - ThR) + 207,0.5));

Jdtheta[0][1] = -((7203*cos(2*ThL))/40 + (9849*cos(2*ThR))/40 - (1029*cos(3*ThR))/10 + (36015*cos(2*ThL - ThR))/128 + (7203*cos(2*ThL - 2*ThR))/40
				- (50421*cos(2*ThL - 3*ThR))/640 - (9849*cos(ThL + ThR))/40 - (73059*cos(ThL))/160 + (340977*cos(ThR))/320 + (6321*cos(ThL - ThR))/10 
				+ (42189*cos(ThL - 2*ThR))/160 + (1029*cos(ThL + 2*ThR))/5 - (1029*cos(2*ThL + ThR))/10 - (7203*cos(ThL - 3*ThR))/40 
				- (21*sin(ThR)*pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) + 49*cos(ThL - ThR) + 207,0.5))/160 
				- 32487/40)/(pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) + 49*cos(ThL - ThR) + 207,0.5));

Jdtheta[1][0] = -((9849*sin(2*ThL))/40 + (1029*sin(3*ThL))/10 + (7203*sin(2*ThR))/40 - (42189*sin(2*ThL - ThR))/160 + (7203*sin(2*ThL - 2*ThR))/40 
				- (7203*sin(3*ThL - ThR))/40 + (50421*sin(3*ThL - 2*ThR))/640 - (9849*sin(ThL + ThR))/40 - (19761*sin(ThL))/320 + (27783*sin(ThR))/32 
				- (22491*sin(ThL - ThR))/20 + (311787*sin(ThL - 2*ThR))/640 + (1029*sin(ThL + 2*ThR))/10 - (1029*sin(2*ThL + ThR))/5 
				+ (21*cos(ThL)*pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) + 49*cos(ThL - ThR) 
				+ 207,0.5))/160)/(pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) + 49*cos(ThL - ThR) + 207,0.5));

Jdtheta[1][1] = -((7203*sin(2*ThL))/40 + (9849*sin(2*ThR))/40 - (1029*sin(3*ThR))/10 + (311787*sin(2*ThL - ThR))/640 - (7203*sin(2*ThL - 2*ThR))/40 
				+ (50421*sin(2*ThL - 3*ThR))/640 - (9849*sin(ThL + ThR))/40 - (27783*sin(ThL))/32 + (19761*sin(ThR))/320 + (22491*sin(ThL - ThR))/20 
				- (42189*sin(ThL - 2*ThR))/160 + (1029*sin(ThL + 2*ThR))/5 - (1029*sin(2*ThL + ThR))/10 + (7203*sin(ThL - 3*ThR))/40 
				+ (21*cos(ThR)*pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) + 49*cos(ThL - ThR) 
				+ 207,0.5))/160)/(pow(56*cos(ThL) - 56*cos(ThR) - 49*cos(ThL - ThR) + 81,1.5)*pow(56*cos(ThR) - 56*cos(ThL) + 49*cos(ThL - ThR) + 207,0.5));


return Jdtheta;

}

////////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::resetWDT()
{
	//Reset Watchdog 
	//outDIO(int channel, short setting, int board)
	s626.outDIO(1, 0, board_cnt[0]); 
	Sleep(1);
	s626.outDIO(1, 1, board_cnt[0]); //pulse DIO line 1 (Watchdog reset)
	Sleep(1);
	s626.outDIO(1, 0, board_cnt[0]);
	Sleep(1);
	s626.outDIO(1, 1, board_cnt[0]); //pulse DIO line 1 (Watchdog reset)
	Sleep(1);
	s626.outDIO(1, 0, board_cnt[0]);
	Sleep(1);
	
}

////////////////////////////////////////////////////////////////////////////////

void ManipulandumRed2D::pollWDT()
{
	//pulse watchdog
	//outDIO(int channel, short setting, int board) //pulse wdt
	if (outputEnable==1) 
	{
		if (wdt_pulse_state==0)
		{
			wdt_pulse_state=1;	
			s626.outDIO(0, 1, board_cnt[0]);
		}
		else
		{
			wdt_pulse_state=0;
			s626.outDIO(0, 0, board_cnt[0]);
		};
	}
}
////////////////////////////////////////////////////////////////////////////////	

void ManipulandumRed2D::checkLimitSwitch(){

		int limitVal = s626.readDIO(3,  board_cnt[0]); //limit switch line (1 normal, 0 at limit)

		if (limitVal!=1) // if limit detected kill robot!
		{
			disableOutput();
			hardLimitOut=1;
		}
		else
		{
			hardLimitOut=0;
		}
 
}

////////////////////////////////////////////////////////////////////////////////	

void ManipulandumRed2D::checkMotorHardEnable(){

		int HEVal = s626.readDIO(2,  board_cnt[0]); //enable status (0 enable, 1 disable)

		if (HEVal!=0) // if hard enable off then pull down safe ramp
		{
			safeVoltGain=0;  
			hardEnableOut=0;
		}
		else
		{
			hardEnableOut=1;
		}
    
}