/////////////////////////////////////////////////////////////////////////////
// ManipulandumRed_API2D.cpp 
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
#include "ManipulandumRed3D_incom2.h"
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
#define CONST_LEN 0.358
#define Pi 3.141592654

#define RECORD_LIMIT 300000 //max size of trajectory file

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


		if (inputFile.bad() || inputFile.eof()){
			cout<<"Manipulandum.init: check parameter file in wrong format\n";
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

	shoulderEncoderFilter.init(Vector2D(0,0));

	wristAngleFilter.init(Vector2D(0,0));
	wristTorqueFilter.init(Vector2D(0,0));

	torqueDemanded=Vector3D(0,0,0);

	//init wrist Angle contol vars
	wristAPTerm=0; //proptional term
	wristAITerm=0; //integral term
	wristADTerm=0; //integral term
	wristAErrLast=0; //last wrist error
	wristAError=0; //wrist error

	wristTPTerm=0; //proptional term
	wristTITerm=0; //integral term
	wristTDTerm=0; //integral term
	wristTErrLast=0; //last wrist error
	wristTError=0; //wrist error

	//default wrist control loop gains
	//Torque loop
	//wristTKp=0.0900; //prop gain
	//wristTKi=0.03000; //intergral gain
	//wristTKd=0.000300; //derivative gain
	//Angle loop
	//wristAKp=0.08; //prop gain /
	//wristAKi=0.0030; //intergral gain
	//wristAKd=0.00074; //derivative gain
	
	wristKpStep=0.00001; ///< Proportional gain step for wrist angle control
	wristKiStep=0.000001; ///< Integral gain step for wrist angle control
	wristKdStep=0.00000001; ///< Deriavtive step for wrist angle control

	WristControlGain=1; //deafult wrist control gain is unity

	wristTSetTorqueLimit=4; //Maximum +/- wrist torque target for force conrol (Nm)

	wristADemandTorque=0; //demand torque signal to wrist motor  
	wristASetAngle=0; //angle set point
	wristTSetTorque=0; //angle set point
	wristFBOn=0; //pid inactive
	wristControlMode=WRISTANGLE; //set wrist position mode

	//init pd FF control vars

	filter_q.init(Vector2D(0,0)); //intialise angular velocity filter

    shoul_a_pk=18.3; //shoulder pk proportional gain
	shoul_a_dk=1.39; //shoulder pd derivative gain

	l_step=0.0001;
	l_g_step=0.0000001;

	ff_on=1;	//feed forward off
	en_traj=0; //wait to start trajectory forward off

	outputEnable = 0; //Set Master output enable to off

//	Lmat[0]=2.5; //matrix of preset Learning factors
//	Lmat[1]=2.5;
//	Lmat[2]=10;
//	Lmat[3]=0.5;
//	Lmat[4]=10;
//	Lmat[5]=0.5;
//	Lmat[6]=10;

	//Lgain = 0.0;
//	Lgain = 0.000010;

	//Lmat[0]=0.06; //matrix of preset Learning factors
	//Lmat[1]=0.06;
	//Lmat[2]=2.5;
	//Lmat[3]=0.2;
	//Lmat[4]=0.10;
	//Lmat[5]=0.2;
	//Lmat[6]=0.10;

	Lmat[0]=0.06; //matrix of preset Learning factors
	Lmat[1]=0.06;
	Lmat[2]=5;
	Lmat[3]=0.4;
	Lmat[4]=0.20;
	Lmat[5]=0.4;
	Lmat[6]=0.40; //6 is different to 4 due to asymetrical learning rate

	//omega 4 and 6 should be 7.5 (units?)

	AngularAccFilt=Vector2D(0,0);  // Angular velocity estimate for shoudler motors after Kalman filtering 
	AngularVelocityFilt=Vector2D(0,0);  // Angular velocity estimate for shoudler motors after Kalman filtering 
	LastAngularVelocityFilt=Vector2D(0,0);  // Last Angular velocity estimate for shoudler motors after Kalman filtering 
	AngleFilt=Vector2D(0,0);  // Angle estimate for shoulder motors after Kalman filtering 

	
	//Lgain = 0.0002;
	Lgain = 0.0;
		
	Omega=Vector7D(0,0,0,0,0,0,0); //intial value for omega zero;
	OmegaReal=Vector7D(0.020140, 0.049171, 1.401095, 0.273960, 0.315758, 0.347740, 0.074067); //omega from learning session;

	int oi=0;
	while (oi<2000)
	{
		omega_hist[oi][0]=0;
		omega_hist[oi][1]=0;
		omega_hist[oi][2]=0;
		omega_hist[oi][3]=0;
		omega_hist[oi][4]=0;
		omega_hist[oi][5]=0;
		omega_hist[oi][6]=0;
		oi++;
	}
	omega_hi=0;

	//load pd traj and output
	load_trajectory_data("C:\\robot\\testdata\\feedfor_tests\\DesiredTrajectory_Spirograph_4seconds_1000Hz.dat");
	open_fb_file("C:\\robot\\testdata\\feedfor_tests\\test_comp3_080312_dummy.txt");

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
void Manipulandum::getCounter(long &rSh,long &lSh,long &wr){ 

	rSh=rShCnt;
	lSh=lShCnt;
	wr=wrCnt;
} 	  

//////////////////////////////////////////////
/// Gets the calibrated shoudler and wrist motor angles in radians
/// \param theAngles 3D Vector by which angles are passed
//////////////////////////////////////////////
void Manipulandum::getAngles(Vector3D &theAngles){

	theAngles.x[0]=theta.x[0];
	theAngles.x[1]=theta.x[1];
	theAngles.x[2]=theta.x[2];
	
}

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

	WristAngle=theta[2]; //update public wrist angle

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

	if (en_traj==1)
	{
	//	update_pd_torque(dt);
		
		//save data
		time_elapsed=time_elapsed+dt;
		write_fb_file(dt);

		traj_index++;

		if(traj_index>traj_length) //reset trajectory index at limit
		{
			traj_index=0; 
			en_traj=0;
			fclose(FB_file_out);
		}

	
	}

	
	//calculate compensation factor
	Vector2D testTau = updateDynamicsComp(dt);
	torqueDemanded[0]=torqueDemanded[0]+testTau[0];
	torqueDemanded[1]=torqueDemanded[1]+testTau[1];
	

	updateWristControl(dt); //Update wrist controller 
	

	
} 

////////////////////////////////////////////////////////////////////////////////
/// Clamps voltage output for 'safer' testing
/// 
////////////////////////////////////////////////////////////////////////////////

Vector2D Manipulandum::updateDynamicsComp(double dt){ ///<Updates Dynamics compensation


	Vector2D TauTemp; //compensation torque temp

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
			double TauLim=2;
			if (TauTemp[1]>TauLim) TauTemp[1]=TauLim;
			if (TauTemp[1]<-TauLim) TauTemp[1]=-TauLim;
			if (TauTemp[0]>TauLim) TauTemp[0]=TauLim;
			if (TauTemp[0]<-TauLim) TauTemp[0]=-TauLim;

		//angular velocity magnitude limit to clamp stationary "tweaking"
			double AvClip=0.05; 
			if (AngularVelocityFilt.norm()<AvClip)
			{
				TauTemp[0]=0;
				TauTemp[1]=0;
			}
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
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::updateOutput(){

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


	//show current tuning parameters
	if (wristControlMode==WRISTTORQUE)
	{
	sprintf(manipbuffer,"TKp=%3.6f, TKi=%3.6f, TKd=%3.6f",wristTKp,wristTKi,wristTKd);
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

	sprintf(manipbuffer,"traj index %i, qdes[0]=%3.8f, qdes[1]=%3.8f",traj_index,trajectory_list[traj_index][0],trajectory_list[traj_index][1]);
	tDisp.setText(manipbuffer,15,col);

		//sprintf(manipbuffer,"Elb right: %3.2f %3.2f",elbowPos[0][0],elbowPos[0][1]);
	//sprintf(manipbuffer,"Shol kp: %3.4f Shol kd: %3.4f",shoul_a_pk,shoul_a_dk);
	//tDisp.setText(manipbuffer,16,col);

//	sprintf(manipbuffer,"Theta err(rad): %3.2f Theta err(rad): %3.2f",qe[0],qe[1]);
//	tDisp.setText(manipbuffer,17,col);
		
	//sprintf(manipbuffer,"ATI FT: %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f",gATI.raw_sample[0],gATI.raw_sample[1],gATI.raw_sample[2],gATI.raw_sample[3],gATI.raw_sample[4],gATI.raw_sample[5]);
//	sprintf(manipbuffer,"FB: %3.2f %3.2f",FB[0],FB[1]);
//	tDisp.setText(manipbuffer,18,col);

	sprintf(manipbuffer,"W1:%3.3f, W2:%3.3f, W3:%3.3f, W4:%3.3f",Omega[0],Omega[1],Omega[2],Omega[3]);
	tDisp.setText(manipbuffer,16,col);

	sprintf(manipbuffer,"W5:%3.3f, W6:%3.3f, W7:%3.3f",Omega[4],Omega[5],Omega[6]);
	tDisp.setText(manipbuffer,17,col);


	sprintf(manipbuffer,"L1:%3.3f, L2:%3.3f, L3:%3.3f, L4:%3.3f",Lmat[0],Lmat[1],Lmat[2],Lmat[3]);
	tDisp.setText(manipbuffer,18,col);

	sprintf(manipbuffer,"L5:%3.3f, L6:%3.3f, L7:%3.3f, Lgain:%3.8f",Lmat[4],Lmat[5],Lmat[6],Lgain);
	tDisp.setText(manipbuffer,19,col);

		
} 

////////////////////////////////////////////////////////////////////////////////
/// Enable function for Dynamics compensation, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::enableDynamicsComp()
{
	safeVoltGain=0; //de-ramp output to prevent jump!
	compEnabled=1;
}

////////////////////////////////////////////////////////////////////////////////
/// Disable function for Dynamics compensation, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::disableDynamicsComp()
{
	safeVoltGain=0; //de-ramp output to prevent jump!
	compEnabled=0;
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
/// Disable for wrist angle contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::selectWristTorqueControl()		///> Sets wrist motor control to torque mode
{
	if (wristFBOn==1) safeVoltGain=0; //de-ramp output to prevent jump!
	wristControlMode=WRISTTORQUE;
	//MAYBE RESET TARGET POSITION HERE!

}

////////////////////////////////////////////////////////////////////////////////
/// Disable for wrist angle contoller, deramps output for safety when called
/// 
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::selectWristAngleControl()		///> Sets wrist motor to position contol mode
{
	if (wristFBOn==1) safeVoltGain=0; //de-ramp output to prevent jump!
	wristControlMode=WRISTANGLE;
	//MAYBE RESET TARGET TORQUE HERE!
}

////////////////////////////////////////////////////////////////////////////////
/// Sets final gain for wrist controller
/// \param Gain of torque output, limited to range 0-1
///
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::setWristControlGain(double Gain)
{
	if (Gain>1) Gain=1;
	if (Gain<0) Gain=0;

	WristControlGain=Gain;
	
}

////////////////////////////////////////////////////////////////////////////////
/// Updates PID contoller, Calls either updateWristAngleControl or 
/// updateWristTorqueControl depending on configuration 
///
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::updateWristControl(double dt)
{

	if (wristFBOn==1) //is control enabled
	{
		//select control mode
		if (wristControlMode==WRISTANGLE) updateWristAngleControl(dt);
		if (wristControlMode==WRISTTORQUE) updateWristTorqueControl(dt);


	}
	else
	{
		torqueDemanded[2]=0;
	}
}		

////////////////////////////////////////////////////////////////////////////////
/// Sets Angle setpoint for wrist contant angle mode, checks against limits
/// \param theAngle Desired angle in Radians
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::setWristAngle(double theAngle) ///> Sets desired wrist angle in Radians. Only fuctional when wrist is in constant angle mode
{
	wristASetAngle=theAngle;
	//trap overrange
	if (wristASetAngle<(wristLimitAngle[0])) wristASetAngle=wristLimitAngle[0];
	if (wristASetAngle>(wristLimitAngle[1])) wristASetAngle=wristLimitAngle[1];
}

////////////////////////////////////////////////////////////////////////////////
/// Sets Angle setpoint for wrist contant torque mode, checks against limits 
///  \param theTorque Desired torque in Nm
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::setWristTorque(double theTorque) ///> Sets desired wrist torque in Nm. Only fuctional when wrist is in constant torque mode
{
		wristTSetTorque=theTorque;
	    //trap overrange
		if (wristTSetTorque<(-wristTSetTorqueLimit)) wristTSetTorque=-wristTSetTorqueLimit;
		if (wristTSetTorque>(wristTSetTorqueLimit)) wristTSetTorque=wristTSetTorqueLimit;

}

////////////////////////////////////////////////////////////////////////////////
/// for constant wrist angle, attempts to hold wrist angle at wristASetAngle degrees
/// (Set by call to setWristAngle(double) ) with respect to global frame. 
////////////////////////////////////////////////////////////////////////////////

void Manipulandum::updateWristAngleControl(double dt) ///< updates wrist angle PID controller 
{

	
		//basic PID for wrist angle
		wristAErrLast=wristAError;
		wristAError=(theta[2]/(PI/180))-(wristASetAngle/(PI/180));

		//filter.position(positionFilt); // retrieve Kalman filter estimated position 
		wristAngleFilter.update(Vector2D(wristAError,0),dt);

		Vector2D wristfiltvel;
		Vector2D wristfiltpos;
		wristAngleFilter.velocity(wristfiltvel);
		wristAngleFilter.position(wristfiltpos);
		
		wristAPTerm=wristfiltpos[0]*wristAKp; //proportional gain
		wristAITerm=wristAITerm+wristfiltpos[0]; //integrate 
		
		//wristADTerm=wristAError-wristAErrLast; //differentiate
		wristADTerm=wristfiltvel[0];
		wristAITerm=wristAITerm*0.98; 	//leak

		wristAITerm=wristAITerm*safeVoltGain; //purge integrator when out of range. Prevents run away when output is dead!!! 

		wristADemandTorque=wristAPTerm+(wristAITerm*wristAKi)+(wristADTerm*wristAKd); //output
			
		torqueDemanded[2]=wristADemandTorque*WristControlGain;

}

////////////////////////////////////////////////////////////////////////////////
/// for constant wrist Torque, attempts to maintain constant torque of wristASetTorque
/// (Set by setWristTorque(double) ) wrt ATI z axis.
////////////////////////////////////////////////////////////////////////////////
void Manipulandum::updateWristTorqueControl(double dt) ///< updates wrist angle PID controller
{

		//basic PID for wrist angle
		wristTErrLast=wristTError;
		wristTError=wristTSetTorque-(gATI.torque[2]*0.001);

		// update Kalman filter estimated position 
		wristTorqueFilter.update(Vector2D(wristTError,0),dt);

		Vector2D wristfiltvel;
		// retrieve Kalman filter estimated position 
		wristTorqueFilter.velocity(wristfiltvel);
		
		wristTPTerm=wristTError*wristTKp; //proportional gain
		wristTITerm=wristTITerm+wristTError; //integrate 
		
		//wristADTerm=wristAError-wristAErrLast; //differentiate
		wristTDTerm=wristfiltvel[0];
		wristTITerm=wristTITerm*0.98; 	//leak

		wristTITerm=wristTITerm*safeVoltGain; //purge integrator when out of range. Prevents run away when output is dead!!! 

		wristTDemandTorque=wristTPTerm+(wristTITerm*wristTKi)+(wristTDTerm*wristTKd); //output
			
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
	Sleep(1); //MUST REMOVE SLEEP TERMS!
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

////////////////////////////////////////////////////////////////////////////////

void Manipulandum::load_trajectory_data(char fin[])
{

	FILE * traj_file_in;
	char traj_fn_in[255];
	//vars for force cal
	strcpy (traj_fn_in,fin);

	float curVal[6]={0};

	//lenght of trajectory data
	traj_length=0;

	//load test data
	traj_file_in = fopen (traj_fn_in,"r");

	if (traj_file_in!=NULL)
	{
		int fd=0;
		while ((fd!=EOF)&(traj_length<RECORD_LIMIT))
		{
		  
		  fd=fscanf (traj_file_in, "%f,%f,%f,%f,%f,%f\n",&curVal[0],&curVal[1],&curVal[2],&curVal[3],&curVal[4],&curVal[5]);
		  //printf("%f,%f,%f,%f,%f,%f\n",curVal[0],curVal[1],curVal[2],curVal[3],curVal[4],curVal[5]);
			
		  trajectory_list[traj_length][0]=curVal[0];
		  trajectory_list[traj_length][1]=curVal[1];
		  trajectory_list[traj_length][2]=curVal[2];
		  trajectory_list[traj_length][3]=curVal[3];
		  trajectory_list[traj_length][4]=curVal[4];
		  trajectory_list[traj_length][5]=curVal[5];

		  traj_length++;
		}
		fclose(traj_file_in);

		traj_length--;

		printf("loaded %i lines from trajectory list\n",traj_length);
	}
	else
	{
		printf("Failed to trajectory list!\n");
	}

}

////////////////////////////////////////////////////////////////////////////////
void Manipulandum::open_fb_file(char fin[]){

		//open data file for forcecal test
	FB_file_out = fopen (fin,"w");
	fprintf(FB_file_out, "Time xpos ypos q1 q2 qd1 qd2 FF1 FF2 FB1 FB2 w1 w2 w3 w4 w5 w6 w7 L1 L2 L3 L4 L5 L6 l7 des_q1 des_q2 des_dq1 des_dq2 des_ddq1 des_ddq2 Torque_d0 Torque_d1 Torque_d2 ATIFx ATIFy ATIFz ATITx ATITy ATITz V0 V1 V2\n");

}

////////////////////////////////////////////////////////////////////////////////

void Manipulandum::write_fb_file(double dt){

	fprintf(FB_file_out, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
							time_elapsed, position[0], position[1],
							q[0],q[1],q_vel_filt[0],q_vel_filt[1],FF[0],FF[1],FB[0],FB[1],
							Omega[0],Omega[1],Omega[2],Omega[3],Omega[4],Omega[5],Omega[6],
							Psi.L[0][0],Psi.L[1][1],Psi.L[2][2],Psi.L[3][3],Psi.L[4][4],Psi.L[5][5],Psi.L[6][6],
							trajectory_list[traj_index][0],trajectory_list[traj_index][1],
							trajectory_list[traj_index][2],trajectory_list[traj_index][3],
							trajectory_list[traj_index][4],trajectory_list[traj_index][5],
							torqueDemandedOut[0],torqueDemandedOut[1],torqueDemandedOut[2],
							globalFTForce[0],globalFTForce[1],globalFTForce[2],
							gATI.torque[0],gATI.torque[1],gATI.torque[2],
							volts[0],volts[1],volts[2]
							);
}

////////////////////////////////////////////////////////////////////////////////
void Manipulandum::update_pd_torque(double dt)
{

	Matrix2D Kp(shoul_a_pk,0,0,shoul_a_pk);
	Matrix2D Kd(shoul_a_dk,0,0,shoul_a_dk);
	
//	Matrix2D JDotReal = J_dot(Vector2D()q,Vector2D() qd) //calc Jdot for actual motor velocity and pos

	PsiReal[0][0]=0; //load psi for real values
	PsiReal[0][1]=0;

	PsiReal[1][0]=0;
	PsiReal[1][1]=0;

	PsiReal[2][0]=0;
	PsiReal[2][1]=0;

	PsiReal[3][0]=0;
	PsiReal[3][1]=0;

	PsiReal[4][0]=0;
	PsiReal[4][1]=0;

	PsiReal[5][0]=0;
	PsiReal[5][1]=0;

	PsiReal[6][0]=0;
	PsiReal[6][1]=0;
	

if (en_traj==1)
	{

		
		q=Vector2D(theta[1],theta[0]); //current measured angle
		qd=Vector2D(q_last-q)*200; //current anglular velocity as derivative
				
		q_des=Vector2D(trajectory_list[traj_index][0],trajectory_list[traj_index][1]); //target angle
		qd_des=Vector2D(trajectory_list[traj_index][2],trajectory_list[traj_index][3]); //target anglular velocity
		qdd_des=Vector2D (trajectory_list[traj_index][4],trajectory_list[traj_index][5]); //target anglular acceleration

	
		//	printf("q_des=[%f %f];\n",q_des[0],q_des[1]);
		//	printf("qd_des=[%f %f];\n",qd_des[0],qd_des[1]);
		//	printf("qdd_des=[%f %f];\n",qdd_des[0],qdd_des[1]);
		
	
	    filter_q.update(q,dt);

		filter_q.velocity(q_vel_filt);
		
		qe=q_des-q; //target angle error

		
		//	printf("q=[%f %f];\n",q[0],q[1]);
		//	printf("qe=[%f %f];\n",qe[0],qe[1]);
		
		
		//Vector2D qe_d=qd-qd_des; //target velocity error from step diff
		Vector2D qe_d=qd_des-q_vel_filt; //target velocity error from kalman filter
		
		//	printf("qd=[%f %f];\n",q_vel_filt[0],q_vel_filt[1]);
		//	printf("qe_d=[%f %f];\n",qe_d[0],qe_d[1]);
	
	
		//UPDATE PSI
		CalcPsi(q_des, qd_des, qdd_des);

		//LOAD LEARNING MATRIX
		Psi.L[0][0]=Lmat[0]*Lgain;
		Psi.L[1][1]=Lmat[1]*Lgain;
		Psi.L[2][2]=Lmat[2]*Lgain;
		Psi.L[3][3]=Lmat[3]*Lgain;
		Psi.L[4][4]=Lmat[4]*Lgain;
		Psi.L[5][5]=Lmat[5]*Lgain;
		Psi.L[6][6]=Lmat[6]*Lgain;
		
		//	printf("Psi(1,:)=[%f %f %f %f %f %f %f];\n",Psi[0][1],Psi[1][1],Psi[2][1],Psi[3][1],Psi[4][1],Psi[5][1],Psi[6][1]);
		//	printf("Psi(2,:)=[%f %f %f %f %f %f %f];\n",Psi[0][0],Psi[1][0],Psi[2][0],Psi[3][0],Psi[4][0],Psi[5][0],Psi[6][0]);

		//printf ("%f %f %f %f %f %f %f\n",Psi.L[0][0],Psi.L[0][1],Psi.L[0][2],Psi.L[0][3],Psi.L[0][4],Psi.L[0][5],Psi.L[0][6]);
		//printf ("%f %f %f %f %f %f %f\n",Psi.L[1][0],Psi.L[1][1],Psi.L[1][2],Psi.L[1][3],Psi.L[1][4],Psi.L[1][5],Psi.L[1][6]);
		//printf ("%f %f %f %f %f %f %f\n",Psi.L[2][0],Psi.L[2][1],Psi.L[2][2],Psi.L[2][3],Psi.L[2][4],Psi.L[2][5],Psi.L[2][6]);
		//printf ("%f %f %f %f %f %f %f\n",Psi.L[3][0],Psi.L[3][1],Psi.L[3][2],Psi.L[3][3],Psi.L[3][4],Psi.L[3][5],Psi.L[3][6]);
		//printf ("%f %f %f %f %f %f %f\n",Psi.L[4][0],Psi.L[4][1],Psi.L[4][2],Psi.L[4][3],Psi.L[4][4],Psi.L[4][5],Psi.L[4][6]);
		//printf ("%f %f %f %f %f %f %f\n",Psi.L[5][0],Psi.L[5][1],Psi.L[5][2],Psi.L[5][3],Psi.L[5][4],Psi.L[5][5],Psi.L[5][6]);
		//printf ("%f %f %f %f %f %f %f\n",Psi.L[6][0],Psi.L[6][1],Psi.L[6][2],Psi.L[6][3],Psi.L[6][4],Psi.L[6][5],Psi.L[6][6]);

		
		//CALCULATE FF TERM
		FF=Psi*Omega;
	
		//	printf("Omega=[%f %f %f %f %f %f %f];\n",Omega[0],Omega[1],Omega[2],Omega[3],Omega[4],Omega[5],Omega[6]);
		//	printf("FF=[%f,%f];\n",FF[0],FF[1]);
	
			
		FB=(Kp*qe)+(Kd*qe_d); //Feedback as errors weighted by prop and der gains
		
		
		//	printf("FB=[%f,%f];\n",FB[0],FB[1]);
	

		//CALCULATE TAU AS SUM OF FF and FB

		if (ff_on==1)
		{
			Tau[0]=FB[0]+FF[1];	
			Tau[1]=FB[1]+FF[0];
		}	
		else
		{
			Tau=FB;
		}

	
		//	printf("Tau=[%f,%f];\n",Tau[0],Tau[1]);
	

		//CALCULATE CHANGE IN OMEGA FOR NEXT CYCLE
		 	Matrix7x2D TransLmult=(Psi.TraceMult());


		//	printf("TransLmult(1,:)=[%f %f %f %f %f %f %f];\n",TransLmult[0][0],TransLmult[1][0],TransLmult[2][0],TransLmult[3][0],TransLmult[4][0],TransLmult[5][0],TransLmult[6][0]);
		//	printf("TransLmult(2,:)=[%f %f %f %f %f %f %f];\n",TransLmult[0][1],TransLmult[1][1],TransLmult[2][1],TransLmult[3][1],TransLmult[4][1],TransLmult[5][1],TransLmult[6][1]);
	
		DeltaOmega=TransLmult*FB;
		
		//	printf("DeltaOmega=[%f,%f,%f,%f,%f,%f,%f];\n",DeltaOmega[0],DeltaOmega[1],DeltaOmega[2],DeltaOmega[3],DeltaOmega[4],DeltaOmega[5],DeltaOmega[6]);
		
	
		Omega=Omega+DeltaOmega; //UPDATE OMEGA 
		
		//en_traj=0;
		//Sleep(100);
		
		omega_hist[omega_hi][0]=Omega[0];
		omega_hist[omega_hi][1]=Omega[1];
		omega_hist[omega_hi][2]=Omega[2];
		omega_hist[omega_hi][3]=Omega[3];
		omega_hist[omega_hi][4]=Omega[4];
		omega_hist[omega_hi][5]=Omega[5];
		omega_hist[omega_hi][6]=Omega[6];
		
		omega_hi++;
		if(omega_hi>=OMEGA_BUFF) omega_hi=0;

				
		q_last=q; //update q for deriavtive calc

	
	}	
	else
	{
		FB=Vector2D(0,0);
	}



	
	//getchar();


	//WRIST CONTROL 
	//double wrist_torque_demand=update_wrist_control(dt);
	torqueDemanded[0]=Tau[1];
	torqueDemanded[1]=Tau[0];

	//setTorque(Vector3D (Tau[1], Tau[0], 0)); //call to set force and update 
		
}