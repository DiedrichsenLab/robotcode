/////////////////////////////////////////////////////////////////////////////
// ManipulandumRed_API3D.cpp 
//
// written for the 3DoF variant of the 3DoM pantograph robot under a s626 io board 
//
// Derived from Manipulandum.h originally by Joern Diedrichsen, 2009
// j.diedrichsen@ucl.ac.uk
// 
// Modified by Nick Roach and Julius Klein, 2011
// n.roach@imperial.ac.uk, j.klein@imperial.ac.uk
//
/////////////////////////////////////////////////////////////////////////////

// include corresponding header file
#include "ManipulandumRed3D.h"
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
#define CONST_abs 65536			// How many counts for full circle for internal kollmorgan encoders (set via motor firmware tool) 
#define CONST_abs_wrist 40000	// How many counts for full circle for wrist enc
//#define CONST_rel_wrist 20480 
#define CONST_rel_wrist 11263
#define CONST_LEN 0.358
#define Pi 3.141592654


//	double fit_vel=0;
////////////////////////////////////////////////////////////////////////////
/// The Constructor really doesn't do anything 
///  
////////////////////////////////////////////////////////////////////////////
Manipulandum::Manipulandum()  {
	isFirstUpdate=1;
}  

////////////////////////////////////////////////////////////////////////////
/// Destructor does nothing 
///  Should probably free the channels allocated on the s626 board 
////////////////////////////////////////////////////////////////////////////
Manipulandum::~Manipulandum()  {

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
/// \li	 Wrist Torque Control P gain [no units]
/// \li	 Wrist Torque Control I gain [no units]
/// \li  Wrist Torque Control D gain [no units]
/// \li	 Wrist Angle Control P gain [no units]
/// \li	 Wrist Angle Control I gain [no units]
/// \li  Wrist Angle Control D gain [no units]
/// \li  Feedforward dead zone for friction [N]
/// \li  Feedforward slope for friction [Deg\sec\N]
///
////////////////////////////////////////////////////////////////////////////
void Manipulandum::init(string paramfile)
{

	double tempDouble;
	string s; 
	paramfilename=paramfile; 
	ifstream inputFile(paramfilename.c_str(),ios::in);
	if(inputFile ==0){
		cout<<"Manipulandum.init: Robot parameter file could not be opened\n";
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

		inputFile>>tempDouble; getline(inputFile,s); wristTKp=tempDouble; //33 Wrist Torque Control P gain [no units] 
		inputFile>>tempDouble; getline(inputFile,s); wristTKi=tempDouble; //34 Wrist Torque Control I gain [no units]
		inputFile>>tempDouble; getline(inputFile,s); wristTKd=tempDouble; //35 Wrist Torque Control D gain [no units]
		inputFile>>tempDouble; getline(inputFile,s); wristAKp=tempDouble; //36  Wrist Angle Control P gain [no units]
		inputFile>>tempDouble; getline(inputFile,s); wristAKi=tempDouble; //37	Wrist Angle Control I gain [no units]
		inputFile>>tempDouble; getline(inputFile,s); wristAKd=tempDouble; //38	Wrist Angle Control D gain [no units]

		inputFile>>tempDouble; getline(inputFile,s); wristFFDeadZone=tempDouble; //39	feedforward dead zone for friction 
		inputFile>>tempDouble; getline(inputFile,s); wristFFSlopeZone=tempDouble; //40	feedforward slope for friction 

		//omega parameters for dynamics model
		inputFile>>tempDouble; getline(inputFile,s); OmegaReal[0]=tempDouble; //41	w0 
		inputFile>>tempDouble; getline(inputFile,s); OmegaReal[1]=tempDouble; //42	w1
		inputFile>>tempDouble; getline(inputFile,s); OmegaReal[2]=tempDouble; //43	w2
		inputFile>>tempDouble; getline(inputFile,s); OmegaReal[3]=tempDouble; //44	w3
		inputFile>>tempDouble; getline(inputFile,s); OmegaReal[4]=tempDouble; //45	w4
		inputFile>>tempDouble; getline(inputFile,s); OmegaReal[5]=tempDouble; //46	w5
		inputFile>>tempDouble; getline(inputFile,s); OmegaReal[6]=tempDouble; //47	w6
		
		
		//added for wristInt new encoder
		inputFile>>board_cnt[3]; getline(inputFile,s);		// 48 Board for Wrist Internal Encoder 
		inputFile>>channel[3]; getline(inputFile,s);		// 49 Channel for Wrist Internal Encoder 
		inputFile>>dir_cnt[3]; getline(inputFile,s);		// 50 Direction for Wrist Internal Encoder 
		inputFile>>offset[3]; getline(inputFile,s);			// 51 Offset for Wrist Internal Encoder [count]
		

		if (inputFile.bad() || inputFile.eof()){
			cout<<"Manipulandum.init: check parameter file in wrong format\n";
			Sleep(2000);
			exit(-1);
		} 
	}  	
	
	s626.initCounter(channel[0],board_cnt[0]);  // reset s626 encoder counters 
	s626.initCounter(channel[1],board_cnt[1]); 
	s626.initCounter(channel[2],board_cnt[2]);
	s626.initCounter(channel[3],board_cnt[3]);

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
	filterEP.init(Vector2D(0,0));

	shoulderEncoderFilter.init(Vector2D(0,0));

	wristAngleFilter.init(Vector2D(0,0));
	wristIntAngleFilter.init(Vector2D(0,0)); //not used in control
	wristTorqueFilter.init(Vector2D(0,0));
	
	double dummy=0; 
	wristAngleFilter.init(&dummy);
	wristIntAngleFilter.init(&dummy); //not used in control
	wristTorqueFilter.init(&dummy);
	

	torqueDemanded=Vector3D(0,0,0);

	//init wrist Angle contol vars
	wristAITerm=0; //integral term
	wristAErrLast=0; //last wrist error
	wristAError=0; //wrist error

	wristTITerm=0; //integral term
	wristTErrLast=0; //last wrist error
	wristTError=0; //wrist error

	//default wrist control loop gains
	
	WristControlGain=1; //default wrist control gain is unity

	wristTSetTorqueLimit=4; //Maximum +/- wrist torque target for force conrol (Nm)

	wristADemandTorque=0; //demand torque signal to wrist motor  
	wristASetAngle=0; //angle set point
	wristTSetTorque=0; //angle set point
	wristFBOn=0; //pid inactive
	wristControlMode=WRISTANGLE; //set wrist position mode

	//init pd FF control vars
    shoul_a_pk=18.3; //shoulder pk proportional gain
	shoul_a_dk=1.39; //shoulder pd derivative gain

	l_step=0.0001;
	l_g_step=0.0000001;

	en_traj=0; //wait to start trajectory forward off
	compGain=1; //dynamics compensation gain 

	outputEnable = 0; //Set Master output enable to off


	//omega 4 and 6 should be 7.5 (units?)

	AngularAccFilt=Vector2D(0,0);  // Angular velocity estimate for shoudler motors after Kalman filtering 
	AngularVelocityFilt=Vector2D(0,0);  // Angular velocity estimate for shoudler motors after Kalman filtering 
	LastAngularVelocityFilt=Vector2D(0,0);  // Last Angular velocity estimate for shoudler motors after Kalman filtering 
	AngleFilt=Vector2D(0,0);  // Angle estimate for shoulder motors after Kalman filtering 

	// These are the dynamic compensation values determined by Nick 3/2012, should calib file
	// OmegaReal=Vector7D(0.020140, 0.049171, 1.401095, 0.273960, 0.315758, 0.347740, 0.074067); //omega from learning session1;


	gATI.init(1000, "C:\\robot\\calib\\FT10172.cal"); //initialise ati data aquisition robot ft
	//	gATI.init(1000, "C:\\robot\\calib\\FT10140.cal"); //initialise ati data aquisition aux ft

	s626.outDIO(0, 0, board_cnt[0]); //confirm first four DIO line as inputs by diabling output transistor
	s626.outDIO(1, 0, board_cnt[0]);
	s626.outDIO(2, 0, board_cnt[0]);
	s626.outDIO(3, 0, board_cnt[0]);

	gATI.start(); //start ati data aquisition
} 


//////////////////////////////////////////////
/// Set the current position of the robot to be (0,0) 
//////////////////////////////////////////////
void Manipulandum::recenter(){ 
	positionOffset=rawPosition;	

};	  

//////////////////////////////////////////////
/// Set the current position of the robot to be (0,0) 
//////////////////////////////////////////////
void Manipulandum::recenter(const Vector2D &x){ 
	positionOffset=x;	
};	  

//////////////////////////////////////////////
/// Gets the raw counter values by reference 
/// \param sh this is where the shoulder counter will be dumped
/// \param el this is where the elbow counter will be dumped
//////////////////////////////////////////////
void Manipulandum::getCounter(long &rSh,long &lSh,long &wr,long &wrI){ 

	rSh=rShCnt;
	lSh=lShCnt;
	wr=wrCnt;
	wrI=wrICnt;
} 	  

//////////////////////////////////////////////
/// Gets the calibrated shoudler and wrist motor angles in radians
/// \param theAngles 3D Vector by which angles are passed
//////////////////////////////////////////////
/*void Manipulandum::getAngles(double &theAngles){

	theAngles[0]=theta[0]; //right sh
	theAngles[1]=theta[1]; //left sh
	theAngles[2]=theta[2]; //wrist abs
	theAngles[3]=theta[3]; //wrist rel
	
}*/

//////////////////////////////////////////////
/// Gets the x and y end point position in cm via forwardKinePos
/// \param thePos 2D Vector by which co-ordinates are passed
//////////////////////////////////////////////

void Manipulandum::getPosition(Vector2D &thePos)			
{
	thePos[0]=position[0];
	thePos[1]=position[1];
}

//////////////////////////////////////////////
/// Gets the Kalman filter estimated x and y end effector velocity in cm/s 
/// \param thePos 2D Vector by which velocity is passed
//////////////////////////////////////////////

void Manipulandum::getVelocity(Vector2D &theVel)				
{
	theVel[0]=velocityFilt[0];
	theVel[1]=velocityFilt[1];
}

//////////////////////////////////////////////
/// Current calibrated force measurements from ATI transducer in sensor frame (N)
/// \return 3D vector giving force in sensor coordinates 
//////////////////////////////////////////////

Vector3D Manipulandum::getLocalFTForce()  
{
	return(localFTForce);
}

//////////////////////////////////////////////
/// Current calibrated force measurements from ATI transducer in sensor frame (N)
/// \param theForce 3D Vector by which force data is passed
//////////////////////////////////////////////

void Manipulandum::getLocalFTForce(Vector3D &theFTForce)   
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

Vector3D Manipulandum::getGlobalFTForce() 
{
	return(globalFTForce);
}

//////////////////////////////////////////////
/// Current calibrated force measurements from ATI transducer in glocal robot frame (N)
/// (sensor frame rotated by wrist angle)
/// \param theForce 3D Vector by which force data is passed
//////////////////////////////////////////////

void Manipulandum::getGlobalFTForce(Vector3D &theFTForce)
{
	theFTForce[0]=globalFTForce[0]; // force measured by ft in global frame
	theFTForce[1]=globalFTForce[1];
	theFTForce[2]=globalFTForce[2];
}

//////////////////////////////////////////////
/// Current calibrated torque measurements from ATI transducer in glocal robot frame (Nmm)
/// (sensor frame rotated by wrist angle)
/// \return theForce 3D Vector giving force in robot coordinates
//////////////////////////////////////////////

Vector3D Manipulandum::getGlobalFTTorque() 
{
	return(Vector3D(gATI.torque[0],gATI.torque[1],gATI.torque[2]));
}

//////////////////////////////////////////////
/// Current calibrated torque measurements from ATI transducer in glocal robot frame (Nmm)
/// (sensor frame rotated by wrist angle)
/// \param theForce 3D Vector by which force data is passed
//////////////////////////////////////////////

void Manipulandum::getGlobalFTTorque(Vector3D &theFTTorque)
{
	theFTTorque[0]=gATI.torque[0]; // force measured by ft in global frame
	theFTTorque[1]=gATI.torque[1];
	theFTTorque[2]=gATI.torque[2];
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
///************** GLOBAL FRAME (definition) ******************
///
///    [0,0] is in the middle of "a5" (midpoint between P1 and P5)
///    [X is positive to the right]
///    [Y is positive upward]
///
/// \param shoulderAngle 2D Vector containing Angles of robot Shoulder motors in radians (Right then Left)
/// \return 2D Vector giving x,y position of endpoint in cm
//////////////////////////////////////////////

Vector2D Manipulandum::forwardKinePos(Vector3D shoulderAngle){

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
void Manipulandum::update(double dt){
	double dummy; 

	//safety checks
	checkLimitSwitch(); //check limits
	checkMotorHardEnable(); //check motors are enabled

	pollWDT(); //if output is active then 

	currentRate=dt; //update current cycle rate 
	lastPosition=position;  // Store the last position 
	

	// Get the counters and compute forward kinematics 
	rShCnt=s626.getCounter(channel[0],board_cnt[0]);  // Get the counters 
	lShCnt=s626.getCounter(channel[1],board_cnt[1]);
	wrCnt=s626.getCounter(channel[2],board_cnt[2]);
	wrICnt=s626.getCounter(channel[3],board_cnt[3]);
    
	theta[0]=  dir_cnt[0]*(((double)(rShCnt)-offset[0])/(CONST_abs/360))*(Pi/180);      // calculate the angles and convert to radians
	theta[1]=  dir_cnt[1]*(((double)(lShCnt)-offset[1])/(CONST_abs/360))*(Pi/180);
	theta[2]=  dir_cnt[2]*(((double)(wrCnt)-offset[2])/(CONST_abs_wrist/360))*(Pi/180);
	theta[3]=  Pi+dir_cnt[3]*(((double)(wrICnt)-offset[3])/(CONST_rel_wrist/360))*(Pi/180);

	WristAngle=theta[2]; //update public wrist angle
	WristIntAngle=theta[3];
	rawPosition=forwardKinePos(Vector3D(theta[0],theta[1],theta[2])); // calculate position of the wrist (3d) or endpoint (2d)
	position=rawPosition-positionOffset; //apply centre offset to pos (set by recentre)
	velocity=(position-lastPosition)/dt; // calculate raw velocity 
	
	// For 3d robot, calculate the position of the fingertip s
	positionEP[0]=wristLength*sin(-WristAngle)+position[0];
	positionEP[1]=wristLength*cos(-WristAngle)+position[1]; 
	
	// Update Kalman filters 
	// Filter for end position 
	filterEP.update(positionEP,dt); 
	filterEP.position(positionFiltEP);
	filterEP.velocity(velocityFiltEP);

	// filter for Wrist position 
	filter.update(position,dt); // update Kalman filter with raw position and time 
	filter.position(positionFilt); // retrieve Kalman filter estimated position 
	filter.velocity(velocityFilt); // retrieve Kalman filter estimated velocity 

	// filter for Wrist Angle
	wristAngleFilter.update(&theta[2],dt); //update wrist angle filter
	wristAngleFilter.position(&wristAngleFilt);
	wristAngleFilter.velocity(&wristVelocityFilt);
	 
	wristIntAngleFilter.update(&theta[3],dt);
	wristIntAngleFilter.position(&WristIntAngleFilt);
	wristIntAngleFilter.velocity(&wristVelocityFilt); //?? do I need this?
			
	gATI.update(); //get ATI FT readings

	localFTForce[0]=gATI.force[0]; //copy local force data 
	localFTForce[1]=gATI.force[1];
	localFTForce[2]=gATI.force[2];

	Vector2D tempFT=ati2World(Vector2D(localFTForce[0],localFTForce[1])); //Rotate force readings to robot frame

	globalFTForce[0]=tempFT[0]; //copy global force data
	globalFTForce[1]=tempFT[1];
	globalFTForce[2]=localFTForce[2];

	///< Update Kalman filter for wrist Torque 
	dummy=gATI.torque[2]*0.001;
	wristTorqueFilter.update(&dummy,dt); 
	wristTorqueFilter.position(&wristTorqueFilt);
	wristTorqueFilter.velocity(&wristTorqueDFilt);

	//calculate compensation factor
	Vector2D CompTau = updateDynamicsComp(dt);

	dymCompTorqueDemanded[0]=CompTau[0];
	dymCompTorqueDemanded[1]=CompTau[1];
	
} 

////////////////////////////////////////////////////////////////////////////////
/// updateDynamicsComp does dynamic compensation for the 2-dof device
////////////////////////////////////////////////////////////////////////////////

Vector2D Manipulandum::updateDynamicsComp(double dt){ ///<Updates Dynamics compensation


	Vector2D TauTemp; //compensation torque temp
	double AvClip=0.05; //clipping level for dither 
	double Avmag; //normalised angular velocity
	double AvClipGain; //gain to stop dithering around zero angular velocity  

	if (compEnabled==1)
	{
		LastAngularVelocityFilt=AngularVelocityFilt; //update previous angular velocity

		shoulderEncoderFilter.update(Vector2D(theta[1],theta[0]),dt); //update kalman filter for shoulder motor angle
		shoulderEncoderFilter.velocity(AngularVelocityFilt);  ///< Angular velocity estimate for shoudler motors after Kalman filtering 
		shoulderEncoderFilter.position(AngleFilt);  ///< Angle estimate for shoulder motors after Kalman filtering 

		//AngularAccFilt=(AngularVelocityFilt-LastAngularVelocityFilt)/dt;  // Badly calculate motor acceleration
		AngularAccFilt=Vector2D(0,0); //Force to zero for now!

		//calculate Psi for current angle and angular velocity, no acc
		PsiReal = CalcPsi(AngleFilt,AngularVelocityFilt,AngularAccFilt);

		//Calculate Tau
		TauTemp=PsiReal*OmegaReal;

		 //limit for maximum torque(Nm) command for compendation
			double TauLim=4;
			if (TauTemp[1]>TauLim) TauTemp[1]=TauLim;
			if (TauTemp[1]<-TauLim) TauTemp[1]=-TauLim;
			if (TauTemp[0]>TauLim) TauTemp[0]=TauLim;
			if (TauTemp[0]<-TauLim) TauTemp[0]=-TauLim;

			//angular velocity ramp to stop dithering around zero angular velocity 
			Avmag=AngularVelocityFilt.norm(); //get normalised velocity
			if (Avmag>AvClip) Avmag=AvClip; //clamp at clip limit

			AvClipGain=0.5*cos(((Avmag/AvClip)+1)*PI)+0.5; 

			TauTemp[0]=TauTemp[0]*AvClipGain; //apply anti dither gain
			TauTemp[1]=TauTemp[1]*AvClipGain;
			
			TauTemp[0]=TauTemp[0]*compGain; //apply final gain
			TauTemp[1]=TauTemp[1]*compGain;

	}
	else
	{
				TauTemp[0]=0; //no compensation
				TauTemp[1]=0;
	}

	//Return compensatory torque
	return(TauTemp);

}

////////////////////////////////////////////////////////////////////////////////
/// Clamps voltage output for 'safer' testing
/// 
////////////////////////////////////////////////////////////////////////////////
//clamps volt limits
void Manipulandum::checkVoltLimits(void){
	//limits
	if (volts[0]>shouldVoltLimit) volts[0]=shouldVoltLimit;
	if (volts[0]<-shouldVoltLimit) volts[0]=-shouldVoltLimit;
	if (volts[1]>shouldVoltLimit) volts[1]=shouldVoltLimit;
	if (volts[1]<-shouldVoltLimit) volts[1]=-shouldVoltLimit;
	if (volts[2]>wristVoltLimit) volts[2]=wristVoltLimit;
	if (volts[2]<-wristVoltLimit) volts[2]=-wristVoltLimit;
}


////////////////////////////////////////////////////////////////////////////////
/// Cuts drive when position is outside desired envelope, ramps force back up on return
/// 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::checkPosLimits(void){				//check position is in operating range

	int armEnvFlag=1;

	//wrist angle soft limit
	if (theta[2]<=wristLimitAngle[0]) 
	{
		//disable robot at wrist limit
		disableOutput();
		armEnvFlag=0; 
	}
	if (theta[2]>=wristLimitAngle[1]) 
	{
		disableOutput();
		armEnvFlag=0;
	}

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
/// At this point it adds the dynamic compensation torques to the toruqes coming 
/// from the forces demanded by the program 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::updateOutput(){

	Vector3D sumTorques(torqueDemanded[0]+dymCompTorqueDemanded[0], //sum user demanded and dynamics model torque(zero when disabled)
						torqueDemanded[1]+dymCompTorqueDemanded[1],
						torqueDemanded[2]);

	torqueDemandedOut=sumTorques; //copy of torque for interrogation

	volts[0]=sumTorques[0]*torque2volts[0]; 
	volts[1]=sumTorques[1]*torque2volts[1];  
	volts[2]=sumTorques[2]*torque2volts[2];  

	setVolts(volts[0],volts[1],volts[2]);

}

////////////////////////////////////////////////////////////////////////////////
/// Sets the voltage output to the Motors directly
/// Actual output is subject to voltage and postion limits imposed by software/hardware
/// \param v0 volts to the right shoulder motor
/// \param v1 volts to the left shoulder motor 
/// \param v2 volts to the wrist motor
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::setVolts(double v0,double v1,double v2) { 

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
void Manipulandum::setTorque(Vector3D torque){

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
void Manipulandum::setForce(Vector2D theForce){


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
Vector2D Manipulandum::ati2World(Vector2D atiforce){ 
	 
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
	void Manipulandum::printState(int col){ 
	char manipbuffer[60];

	
	sprintf(manipbuffer,"TIndex: %i",traj_index);
	tDisp.setText(manipbuffer,0,col);
	sprintf(manipbuffer,"Counter: %d %d %d %d",rShCnt,lShCnt,wrCnt,wrICnt);
	tDisp.setText(manipbuffer,1,col);
	sprintf(manipbuffer,"Theta: %3.2f %3.2f %3.2f %3.2f",theta[0]/(Pi/180),theta[1]/(Pi/180),theta[2]/(Pi/180),theta[3]/(Pi/180));
	tDisp.setText(manipbuffer,2,col);
	sprintf(manipbuffer,"ThetaFilt: %3.2f %3.2f",wristAngleFilt/(Pi/180),WristIntAngleFilt/(Pi/180));
	tDisp.setText(manipbuffer,3,col);

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


	//show current tuning parameters
	if (wristControlMode==WRISTTORQUE)
	{
		//sprintf(manipbuffer,"TKp=%3.6f, TKi=%3.6f, TKd=%3.6f",wristTKp,wristTKi,wristTKd);
		sprintf(manipbuffer,"ffdead=%3.6f, ffslope=%3.6f",wristFFDeadZone,wristFFSlopeZone);
	}

	if (wristControlMode==WRISTANGLE)
	{
		sprintf(manipbuffer,"AKp=%3.6f, AKi=%3.6f, AKd=%3.6f",wristAKp,wristAKi,wristAKd);
	}
	
	if (wristFBOn==0)
	{
	sprintf(manipbuffer,"Tuning off");
	}

	tDisp.setText(manipbuffer,10,col);

	sprintf(manipbuffer,"Safe v gain=%3.4f, Soft Enable=%i, Wrist PID Active=%i",safeVoltGainOut,outputEnable,wristFBOn);
	tDisp.setText(manipbuffer,11,col);
	sprintf(manipbuffer,"Hard limit switch=%i, Hard enable signal=%i",hardLimitOut,hardEnableOut);
	tDisp.setText(manipbuffer,12,col);
	sprintf(manipbuffer,"wrist set angle=%3.1f, wrist set torque=%3.1f",wristASetAngle,wristTSetTorque);
	tDisp.setText(manipbuffer,13,col);

	//sprintf(manipbuffer,"TKp=%3.8f, TKi=%3.8f, TKd=%3.8f",wristTKp,wristTKi,wristTKd);
	//tDisp.setText(manipbuffer,14,col);

	sprintf(manipbuffer,"WristControlGain=%3.8f",WristControlGain);
	tDisp.setText(manipbuffer,14,col);


		//sprintf(manipbuffer,"Elb right: %3.2f %3.2f",elbowPos[0][0],elbowPos[0][1]);
	//sprintf(manipbuffer,"Shol kp: %3.4f Shol kd: %3.4f",shoul_a_pk,shoul_a_dk);
	//tDisp.setText(manipbuffer,16,col);

//	sprintf(manipbuffer,"Theta err(rad): %3.2f Theta err(rad): %3.2f",qe[0],qe[1]);
//	tDisp.setText(manipbuffer,17,col);
		
	//sprintf(manipbuffer,"ATI FT: %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f",gATI.raw_sample[0],gATI.raw_sample[1],gATI.raw_sample[2],gATI.raw_sample[3],gATI.raw_sample[4],gATI.raw_sample[5]);
//	sprintf(manipbuffer,"FB: %3.2f %3.2f",FB[0],FB[1]);
//	tDisp.setText(manipbuffer,18,col);

	//learned omega
	/*sprintf(manipbuffer,"W1:%3.3f, W2:%3.3f, W3:%3.3f, W4:%3.3f",Omega[0],Omega[1],Omega[2],Omega[3]);
	tDisp.setText(manipbuffer,16,col);

	sprintf(manipbuffer,"W5:%3.3f, W6:%3.3f, W7:%3.3f",Omega[4],Omega[5],Omega[6]);
	tDisp.setText(manipbuffer,17,col);*/

	//fixed omega


	/*	if(GetAsyncKeyState(VK_NUMPAD1))
			{
				OmegaReal[3]=OmegaReal[3]-om_step;
				OmegaReal[5]=OmegaReal[5]-om_step;
			}

	if(GetAsyncKeyState(VK_NUMPAD4))
			{
				OmegaReal[3]=OmegaReal[3]+om_step;
				OmegaReal[5]=OmegaReal[5]+om_step;
			}

	if(GetAsyncKeyState(VK_NUMPAD2))
			{
				OmegaReal[4]=OmegaReal[4]-om_step;
				OmegaReal[6]=OmegaReal[6]-om_step;
			}

	if(GetAsyncKeyState(VK_NUMPAD5))
			{
				OmegaReal[4]=OmegaReal[4]+om_step;
				OmegaReal[6]=OmegaReal[6]+om_step;
			}*/
		
} 

////////////////////////////////////////////////////////////////////////////////
/// Enable function for Dynamics compensation, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::enableDynamicsComp()
{
	dymCompTorqueDemanded=Vector2D(0,0);
	safeVoltGain=0; //de-ramp output to prevent jump!
	compEnabled=1;
}

////////////////////////////////////////////////////////////////////////////////
/// Disable function for Dynamics compensation, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::disableDynamicsComp()
{
	dymCompTorqueDemanded=Vector2D(0,0);
	safeVoltGain=0; //de-ramp output to prevent jump!
	compEnabled=0;
}

////////////////////////////////////////////////////////////////////////////////
/// Sets final gain for dynamics compensation 
/// \param Gain of compensation torque, limited to range 0-1
///
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::setDynamicsCompGain(double gain)
{
	if (gain>1) gain=1;
	if (gain<0) gain=0;

	compGain=gain;
	
}

////////////////////////////////////////////////////////////////////////////////
/// Master enable function for motors, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::enableOutput()
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

void Manipulandum::disableOutput()
{
	safeVoltGain=0; //de-ramp output to prevent jump!
	outputEnable=0; //Master Disable DAC output 
	outputEnableOut=outputEnable;  //copy for interogation
}	

////////////////////////////////////////////////////////////////////////////////
/// Enable for wrist angle contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::enableWristControl()
{
	wristFBOn=1; //enable pid wrist pos control 
	safeVoltGain=0; //de-ramp output to prevent jump!
	wristAITerm=0; //Zero Angle integrator
	wristTITerm=0; //Zero Torque integrator

}		

////////////////////////////////////////////////////////////////////////////////
/// Checks wrist mode, returns zero if control disabled, otherwise wristControlMode
/// 
////////////////////////////////////////////////////////////////////////////////

int Manipulandum::getWristMode()	///>checks current mode of wrist contoller
{

	if (wristFBOn==0) return (0);
	return (wristControlMode);


}

////////////////////////////////////////////////////////////////////////////////
/// Disable for wrist angle contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::disableWristControl()
{
	wristFBOn=0; //enable pid wrist pos control 
	safeVoltGain=0; //de-ramp output to prevent jump!
	wristAITerm=0; //Zero integrator
	wristTITerm=0; //Zero Torque integrator

}

////////////////////////////////////////////////////////////////////////////////
/// Switch to wrist torque contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::selectWristTorqueControlSafe()		///> Sets wrist motor control to torque mode
{
	if (wristFBOn==1) safeVoltGain=0; //de-ramp output to prevent jump!
	wristControlMode=WRISTTORQUE;
	//MAYBE RESET TARGET POSITION HERE!

}

////////////////////////////////////////////////////////////////////////////////
/// Unsafe switch to wrist angle contoller, WARNING: SWITCHES DIRECTLY WITH NO SAFETY RAMP
/// 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::selectWristTorqueControl()		///> Sets wrist motor control to torque mode
{
	wristControlMode=WRISTTORQUE;
}

////////////////////////////////////////////////////////////////////////////////
/// Switch to wrist angle contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::selectWristAngleControlSafe()		///> Sets wrist motor to position contol mode
{
	if (wristFBOn==1) safeVoltGain=0; //de-ramp output to prevent jump!
	wristControlMode=WRISTANGLE;
}

void Manipulandum::selectWristAngleControl()		///> Sets wrist motor to position contol mode
{
	wristControlMode=WRISTANGLE;
}

////////////////////////////////////////////////////////////////////////////////
/// Sets final gain for wrist controller
/// \param Gain of torque output, limited to range 0-1
///
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::setWristControlGain(double gain)
{
	if (gain>1) gain=1; //limit between 0 and 1 
	if (gain<0) gain=0;

	WristControlGain=gain;
}


////////////////////////////////////////////////////////////////////////////////
/// for constant wrist angle, attempts to hold wrist angle at wristASetAngle degrees
/// (Set by call to setWristAngle(double) ) with respect to global frame. 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::updateWristAngleControl(double wristASetAngle) ///< updates wrist angle PID controller
{
	
		//basic PID for wrist angle
		wristAErrLast=wristAError;
		wristAError=(wristAngleFilt/(PI/180))-(wristASetAngle/(PI/180));	///< Get wrist angle error
		wristAErrorD=wristVelocityFilt/(PI/180);							///< Get wrist angle error change 
		
		wristAITerm=wristAITerm+wristAError; //integrate 
		wristAITerm=wristAITerm*0.98; 	//leak
		
		//wristADTerm=wristAError-wristAErrLast; //differentiate

		wristAITerm=wristAITerm*safeVoltGain; //purge integrator when out of range. Prevents run away when output is dead!!! 

		wristADemandTorque=wristAError*wristAKp+(wristAITerm*wristAKi)+(wristAErrorD*wristAKd); //output

		double fit_vel=0; //calculate feedforward term from friction paras
		fit_vel=wristFFDeadZone*sign(wristAErrorD)+((wristFFSlopeZone)*wristAErrorD);

		wristTDemandTorque=wristTDemandTorque+fit_vel; //sum output components

			
		torqueDemanded[2]=wristADemandTorque*WristControlGain;

}

////////////////////////////////////////////////////////////////////////////////
/// for constant wrist Torque, attempts to maintain constant torque of wristASetTorque
/// (Set by setWristTorque(double) ) wrt ATI z axis.
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::updateWristTorqueControl(double wristTSetTorque) ///< updates wrist angle PID controller
{

		//basic PID for wrist angle
		wristTErrLast=wristTError;
		wristTError=wristTSetTorque-wristTorqueFilt;
		wristTErrorD=-wristTorqueDFilt;
		
		wristTITerm=0.98*(wristTITerm+wristTError); //integrate 

		wristTITerm=wristTITerm*safeVoltGain; //purge integrator when out of range. Prevents run away when output is dead!!! 

		wristTDemandTorque=wristTError*wristTKp+(wristTITerm*wristTKi)+(wristTErrorD*wristTKd); //output

		double fit_vel=0; //calculate feedforward term from friction paras
		fit_vel=wristFFDeadZone*sign(wristVelocityFilt)+((wristFFSlopeZone)*wristVelocityFilt);


		wristTDemandTorque=wristTDemandTorque+fit_vel; //sum output components
			
		torqueDemanded[2]=wristTDemandTorque*WristControlGain;
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

Matrix2D Manipulandum::Jacobian(Vector2D shoulderAngles) //calculates static motor torques for given motor angles and desired forces
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

void Manipulandum::resetWDT()
{
	//Reset Watchdog 
	//outDIO(int channel, short setting, int board)
	s626.outDIO(1, 0, board_cnt[0]); 
	Sleep(5); //MUST REMOVE SLEEP TERMS!
	s626.outDIO(1, 1, board_cnt[0]); //pulse DIO line 1 (Watchdog reset)
	Sleep(5);
	s626.outDIO(1, 0, board_cnt[0]);
	Sleep(1);
	s626.outDIO(1, 1, board_cnt[0]); //pulse DIO line 1 (Watchdog reset)
	Sleep(1);
	s626.outDIO(1, 0, board_cnt[0]);
	Sleep(1);
	
}

////////////////////////////////////////////////////////////////////////////////

void Manipulandum::pollWDT()
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

void Manipulandum::checkLimitSwitch(){

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

void Manipulandum::checkMotorHardEnable(){

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

////////////////////////////////////////////////////////////////////////////////

Matrix7x2D Manipulandum::CalcPsi(Vector2D q,Vector2D qd,Vector2D qdd)
{
	Matrix2D J=Jacobian(Vector2D(q[1],q[0]));
	
	Matrix2D JT=J.getTranspose();

//	printf("%f %f\n", J[0][0], J[0][1]);0
//	printf("%f %f\n\n", J[1][0], J[1][1]);

	Matrix2D dJdt=J_dot(Vector2D(q[0],q[1]),Vector2D(qd[0],qd[1]));
	//Matrix2D dJdt=J_dot(Vector2D(1,2),Vector2D(3,4));
	dJdt=dJdt.getTranspose();

//	printf("%f %f\n", dJdt[0][0], dJdt[0][1]);
//	printf("%f %f\n\n", dJdt[1][0], dJdt[1][1]);
//	getchar();

	Vector2D xdd = dJdt*Vector2D(qd[0],qd[1]) + J*Vector2D(qdd[0], qdd[1]);


	Psi=Matrix7x2D(0,0,0,0,0,0,0,0,0,0,0,0,0,0);


	//Psi=Matrix7x2D(0,qdd[0], qdd[1] ,0, JT[1][0]*xdd[0]+JT[1][1]*xdd[1], JT[0][0]*xdd[0]+JT[0][1]*xdd[1],0, sign(qd[0]),0,qd[0],sign(qd[0]),0,qd[1],0);
	Psi=Matrix7x2D(0,qdd[0], qdd[1] ,0, JT[1][0]*xdd[0]+JT[1][1]*xdd[1], JT[0][0]*xdd[0]+JT[0][1]*xdd[1],0, sign(qd[0]),0,qd[0],sign(qd[1]),0,qd[1],0);
																		//	JT(1,1)*x1dd+JT(1,2)*x2dd
	//	[q1dd 0 JT(1,1)*xdd[0]+JT(1,2)*xdd[1] sign(q1d) q1d 0 0;...
	//	  0 q2dd JT(2,1)*xdd[0]+JT(2,2)*xdd[1] 0 0 sign(q2d) q2d];

	//Psi=[q1dd 0 JT(1,1)*x1dd+JT(1,2)*x2dd sign(q1d) q1d 0 0;...
    //0 q2dd JT(2,1)*x1dd+JT(2,2)*x2dd 0 0 sign(q2d) q2d];

	return(Psi);
}

////////////////////////////////////////////////////////////////////////////////
Matrix2D Manipulandum::J_dot(Vector2D q,Vector2D qd) //calculates the time derivative of the Jacobian
{ 

//This functions calculates the time derivative of the Jacobian
//
//to be used like this:     xdd = dJdt*qd + J*qdd
//
//
// J=Jacobian(q1,q2);
// 
// dJdt=[jacobian(J(1,1),[q1 q2])*[q1d;q2d] jacobian(J(1,2),[q1 q2])*[q1d;q2d];...
//     jacobian(J(2,1),[q1 q2])*[q1d;q2d] jacobian(J(2,2),[q1 q2])*[q1d;q2d]];
// 
//dJdt = [[jacobian(J(1,1))*qd    jacobian(J(1,2))*qd];
//        [jacobian(J(2,1))*qd    jacobian(J(2,2))*qd]];

double q1=q[0];
double q2=q[1];
double q1d=qd[0];
double q2d=qd[1];
Matrix2D dJdt(0,0,0,0);

dJdt[0][0] = q1d*(((6321*sin(q1 - q2))/10 - (36015*sin(q1 - 2*q2))/128 + (1029*sin(q1 + 2*q2))/10 - (2058*sin(2*q1 + q2))/5 + (21609*sin(q2 - 3*q1))/40 + (9849*sin(2*q1))/20 + (3087*sin(3*q1))/10 - (42189*sin(2*q1 - q2))/80 + (7203*sin(2*q1 - 2*q2))/20 + (4157885683807027*sin(3*q1 - 2*q2))/17592186044416 - (9849*sin(q1 + q2))/40 - (2343176101901107*sin(q1))/2199023255552 + (21*cos(q1)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 + (63*sin(q1)*(49*sin(q1 - q2) - 56*sin(q1))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,0.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/320 - (21*sin(q1)*(49*sin(q1 - q2) - 56*sin(q1))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5))/(320*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)) - (((49*sin(q1 - q2))/2 - 28*sin(q1))*((6321*cos(q1 - q2))/10 - (36015*cos(q1 - 2*q2))/128 + (1029*cos(q1 + 2*q2))/10 - (1029*cos(2*q1 + q2))/5 - (7203*cos(q2 - 3*q1))/40 + (9849*cos(2*q1))/40 + (1029*cos(3*q1))/10 + (7203*cos(2*q2))/40 - (42189*cos(2*q1 - q2))/160 + (7203*cos(2*q1 - 2*q2))/40 + (50421*cos(3*q1 - 2*q2))/640 - (9849*cos(q1 + q2))/40 - (2343176101901107*cos(q1))/2199023255552 + (73059*cos(q2))/160 - (21*sin(q1)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 - 32487/40))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5)) + (((147*sin(q1 - q2))/2 - 84*sin(q1))*((6321*cos(q1 - q2))/10 - (36015*cos(q1 - 2*q2))/128 + (1029*cos(q1 + 2*q2))/10 - (1029*cos(2*q1 + q2))/5 - (7203*cos(q2 - 3*q1))/40 + (9849*cos(2*q1))/40 + (1029*cos(3*q1))/10 + (7203*cos(2*q2))/40 - (42189*cos(2*q1 - q2))/160 + (7203*cos(2*q1 - 2*q2))/40 + (50421*cos(3*q1 - 2*q2))/640 - (9849*cos(q1 + q2))/40 - (2343176101901107*cos(q1))/2199023255552 + (73059*cos(q2))/160 - (21
*sin(q1)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 - 32487/40))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))) - (q2d*(1185408*sin(2*q1 + q2) - 1185408*sin(q1 + 2*q2) - (2409101803074355*sin(q1 - 2*q2))/268435456 + 1231713*sin(q1 - 3*q2) + 1231713*sin(q2 - 3*q1) + 777924*sin(2*q1) + 777924*sin(2*q2) - (2409101803074355*sin(2*q1 - q2))/268435456 + (8621991*sin(2*q1 - 3*q2))/8 + (8621991*sin(3*q1 - 2*q2))/8 + (2571072327961805*sin(q1 + q2))/268435456 - (9640701*sin(q1))/4 + (9640701*sin(q2))/4))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5));

dJdt[1][0] = - q2d*(((6321*sin(q1 - q2))/10 + (42189*sin(q1 - 2*q2))/80 - (2058*sin(q1 + 2*q2))/5 + (1029*sin(2*q1 + q2))/10 - (21609*sin(q1 - 3*q2))/40 - (9849*sin(2*q2))/20 + (3087*sin(3*q2))/10 + (36015*sin(2*q1 - q2))/128 + (7203*sin(2*q1 - 2*q2))/20 - (4157885683807027*sin(2*q1 - 3*q2))/17592186044416 + (9849*sin(q1 + q2))/40 - (2343176101901107*sin(q2))/2199023255552 - (21*cos(q2)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 + (63*sin(q2)*(49*sin(q1 - q2) - 56*sin(q2))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,0.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/320 - (21*sin(q2)*(49*sin(q1 - q2) - 56*sin(q2))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5))/(320*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)) - (((49*sin(q1 - q2))/2 - 28*sin(q2))*((6321*cos(q1 - q2))/10 + (42189*cos(q1 - 2*q2))/160 + (1029*cos(q1 + 2*q2))/5 - (1029*cos(2*q1 + q2))/10 - (7203*cos(q1 - 3*q2))/40 + (7203*cos(2*q1))/40 + (9849*cos(2*q2))/40 - (1029*cos(3*q2))/10 + (36015*cos(2*q1 - q2))/128 + (7203*cos(2*q1 - 2*q2))/40 - (50421*cos(2*q1 - 3*q2))/640 - (9849*cos(q1 + q2))/40 - (73059*cos(q1))/160 + (2343176101901107*cos(q2))/2199023255552 - (21*sin(q2)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 - 32487/40))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5)) + (((147*sin(q1 - q2))/2 - 84*sin(q2))*((6321*cos(q1 - q2))/10 + (42189*cos(q1 - 2*q2))/160 + (1029*cos(q1 + 2*q2))/5 - (1029*cos(2*q1 + q2))/10 - (7203*cos(q1 - 3*q2))/40 + (7203*cos(2*q1))/40 + (9849*cos(2*q2))/40 - (1029*cos(3*q2))/10 + (36015*cos(2*q1 - q2))/128 + (7203*cos(2*q1 - 2*q2))/40 - (50421*cos(2*q1 - 3*q2))/640 - (9849*cos(q1 + q2))/40 - (73059*cos(q1))/160 + (2343176101901107*cos(q2))/2199023255552 - (
21*sin(q2)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 - 32487/40))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))) - (q1d*(1185408*sin(2*q1 + q2) - 1185408*sin(q1 + 2*q2) - (2409101803074355*sin(q1 - 2*q2))/268435456 + 1231713*sin(q1 - 3*q2) + 1231713*sin(q2 - 3*q1) + 777924*sin(2*q1) + 777924*sin(2*q2) - (2409101803074355*sin(2*q1 - q2))/268435456 + (8621991*sin(2*q1 - 3*q2))/8 + (8621991*sin(3*q1 - 2*q2))/8 + (2571072327961805*sin(q1 + q2))/268435456 - (9640701*sin(q1))/4 + (9640701*sin(q2))/4))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5));

dJdt[0][1] = q1d*(((22491*cos(q1 - q2))/20 - (8570335797234893*cos(q1 - 2*q2))/17592186044416 - (1029*cos(q1 + 2*q2))/10 + (2058*cos(2*q1 + q2))/5 + (21609*cos(q2 - 3*q1))/40 - (9849*cos(2*q1))/20 - (3087*cos(3*q1))/10 + (42189*cos(2*q1 - q2))/80 - (7203*cos(2*q1 - 2*q2))/20 - (4157885683807027*cos(3*q1 - 2*q2))/17592186044416 + (9849*cos(q1 + q2))/40 + (19761*cos(q1))/320 + (21*sin(q1)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 - (63*cos(q1)*(49*sin(q1 - q2) - 56*sin(q1))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,0.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/320 + (21*cos(q1)*(49*sin(q1 - q2) - 56*sin(q1))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5))/(320*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)) - (((49*sin(q1 - q2))/2 - 28*sin(q1))*((8570335797234893*sin(q1 - 2*q2))/17592186044416 - (22491*sin(q1 - q2))/20 + (1029*sin(q1 + 2*q2))/10 - (1029*sin(2*q1 + q2))/5 + (7203*sin(q2 - 3*q1))/40 + (9849*sin(2*q1))/40 + (1029*sin(3*q1))/10 + (7203*sin(2*q2))/40 - (42189*sin(2*q1 - q2))/160 + (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(3*q1 - 2*q2))/640 - (9849*sin(q1 + q2))/40 - (19761*sin(q1))/320 + (27783*sin(q2))/32 + (21*cos(q1)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5)) + (((147*sin(q1 - q2))/2 - 84*sin(q1))*((8570335797234893*sin(q1 - 2*q2))/17592186044416 - (22491*sin(q1 - q2))/20 + (1029*sin(q1 + 2*q2))/10 - (1029*sin(2*q1 + q2))/5 + (7203*sin(q2 - 3*q1))/40 + (9849*sin(2*q1))/40 + (1029*sin(3*q1))/10 + (7203*sin(2*q2))/40 - (42189*sin(2*q1 - q2))/160 + (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(3*q1 - 2*q2))/640 - (9849*sin(q1 + q2))/40 - (19761*sin(q1))/320 + (27783*sin(q2))/32 + (21*cos(q1
)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))) - q2d*(((22491*cos(q1 - q2))/20 - (8570335797234893*cos(q1 - 2*q2))/8796093022208 + (1029*cos(q1 + 2*q2))/5 - (1029*cos(2*q1 + q2))/5 + (7203*cos(q2 - 3*q1))/40 + (7203*cos(2*q2))/20 + (42189*cos(2*q1 - q2))/160 - (7203*cos(2*q1 - 2*q2))/20 - (50421*cos(3*q1 - 2*q2))/320 - (9849*cos(q1 + q2))/40 + (27783*cos(q2))/32 - (63*cos(q1)*(49*sin(q1 - q2) - 56*sin(q2))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,0.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/320 + (21*cos(q1)*(49*sin(q1 - q2) - 56*sin(q2))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5))/(320*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)) - (((49*sin(q1 - q2))/2 - 28*sin(q2))*((8570335797234893*sin(q1 - 2*q2))/17592186044416 - (22491*sin(q1 - q2))/20 + (1029*sin(q1 + 2*q2))/10 - (1029*sin(2*q1 + q2))/5 + (7203*sin(q2 - 3*q1))/40 + (9849*sin(2*q1))/40 + (1029*sin(3*q1))/10 + (7203*sin(2*q2))/40 - (42189*sin(2*q1 - q2))/160 + (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(3*q1 - 2*q2))/640 - (9849*sin(q1 + q2))/40 - (19761*sin(q1))/320 + (27783*sin(q2))/32 + (21*cos(q1)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5)) + (((147*sin(q1 - q2))/2 - 84*sin(q2))*((8570335797234893*sin(q1 - 2*q2))/17592186044416 - (22491*sin(q1 - q2))/20 + (1029*sin(q1 + 2*q2))/10 - (1029*sin(2*q1 + q2))/5 + (7203*sin(q2 - 3*q1))/40 + (9849*sin(2*q1))/40 + (1029*sin(3*q1))/10 + (7203*sin(2*q2))/40 - (42189*sin(2*q1 - q2))/160 + (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(3*q1 - 2*q2))/640 - (9849*sin(q1 + q2))/40 
- (19761*sin(q1))/320 + (27783*sin(q2))/32 + (21*cos(q1)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)));

dJdt[1][1] = q2d*(((22491*cos(q1 - q2))/20 - (42189*cos(q1 - 2*q2))/80 - (2058*cos(q1 + 2*q2))/5 + (1029*cos(2*q1 + q2))/10 + (21609*cos(q1 - 3*q2))/40 - (9849*cos(2*q2))/20 + (3087*cos(3*q2))/10 + (8570335797234893*cos(2*q1 - q2))/17592186044416 - (7203*cos(2*q1 - 2*q2))/20 + (4157885683807027*cos(2*q1 - 3*q2))/17592186044416 + (9849*cos(q1 + q2))/40 - (19761*cos(q2))/320 + (21*sin(q2)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160 + (63*cos(q2)*(49*sin(q1 - q2) - 56*sin(q2))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,0.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/320 - (21*cos(q2)*(49*sin(q1 - q2) - 56*sin(q2))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5))/(320*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)) + (((49*sin(q1 - q2))/2 - 28*sin(q2))*((22491*sin(q1 - q2))/20 - (42189*sin(q1 - 2*q2))/160 + (1029*sin(q1 + 2*q2))/5 - (1029*sin(2*q1 + q2))/10 + (7203*sin(q1 - 3*q2))/40 + (7203*sin(2*q1))/40 + (9849*sin(2*q2))/40 - (1029*sin(3*q2))/10 + (8570335797234893*sin(2*q1 - q2))/17592186044416 - (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(2*q1 - 3*q2))/640 - (9849*sin(q1 + q2))/40 - (27783*sin(q1))/32 + (19761*sin(q2))/320 + (21*cos(q2)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5)) - (((147*sin(q1 - q2))/2 - 84*sin(q2))*((22491*sin(q1 - q2))/20 - (42189*sin(q1 - 2*q2))/160 + (1029*sin(q1 + 2*q2))/5 - (1029*sin(2*q1 + q2))/10 + (7203*sin(q1 - 3*q2))/40 + (7203*sin(2*q1))/40 + (9849*sin(2*q2))/40 - (1029*sin(3*q2))/10 + (8570335797234893*sin(2*q1 - q2))/17592186044416 - (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(2*q1 - 3*q2))/640 - (9849*sin(q1 + q2))/40 - (27783*sin(q1))/32 + (19761*sin(q2))/320 + (21*cos(q2
)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))) - q1d*(((22491*cos(q1 - q2))/20 - (42189*cos(q1 - 2*q2))/160 + (1029*cos(q1 + 2*q2))/5 - (1029*cos(2*q1 + q2))/5 + (7203*cos(q1 - 3*q2))/40 + (7203*cos(2*q1))/20 + (8570335797234893*cos(2*q1 - q2))/8796093022208 - (7203*cos(2*q1 - 2*q2))/20 + (50421*cos(2*q1 - 3*q2))/320 - (9849*cos(q1 + q2))/40 - (27783*cos(q1))/32 + (63*cos(q2)*(49*sin(q1 - q2) - 56*sin(q1))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,0.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/320 - (21*cos(q2)*(49*sin(q1 - q2) - 56*sin(q1))*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5))/(320*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)) + (((49*sin(q1 - q2))/2 - 28*sin(q1))*((22491*sin(q1 - q2))/20 - (42189*sin(q1 - 2*q2))/160 + (1029*sin(q1 + 2*q2))/5 - (1029*sin(2*q1 + q2))/10 + (7203*sin(q1 - 3*q2))/40 + (7203*sin(2*q1))/40 + (9849*sin(2*q2))/40 - (1029*sin(3*q2))/10 + (8570335797234893*sin(2*q1 - q2))/17592186044416 - (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(2*q1 - 3*q2))/640 - (9849*sin(q1 + q2))/40 - (27783*sin(q1))/32 + (19761*sin(q2))/320 + (21*cos(q2)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,1.5)) - (((147*sin(q1 - q2))/2 - 84*sin(q1))*((22491*sin(q1 - q2))/20 - (42189*sin(q1 - 2*q2))/160 + (1029*sin(q1 + 2*q2))/5 - (1029*sin(2*q1 + q2))/10 + (7203*sin(q1 - 3*q2))/40 + (7203*sin(2*q1))/40 + (9849*sin(2*q2))/40 - (1029*sin(3*q2))/10 + (8570335797234893*sin(2*q1 - q2))/17592186044416 - (7203*sin(2*q1 - 2*q2))/40 + (50421*sin(2*q1 - 3*q2))/640 - (9849*sin(q1 + q2))/40 
- (27783*sin(q1))/32 + (19761*sin(q2))/320 + (21*cos(q2)*pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,1.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5))/160))/(pow(56*cos(q1) - 49*cos(q1 - q2) - 56*cos(q2) + 81,2.5)*pow(49*cos(q1 - q2) - 56*cos(q1) + 56*cos(q2) + 207,0.5)));

return dJdt;
}
