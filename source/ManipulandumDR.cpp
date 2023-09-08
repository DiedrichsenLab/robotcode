////////////////////////////////////////////////////////////////////////////
// ManipulandumDR.cpp
// 
// written for a 2-link robot with absolute rotatry encoders (MicroESystem) 
// under a s626 iocard 
// an electrical motors 
// 
// Uses Vector2D and Matrix2D classes for compact code; 
// ManipulandumDR
// Joern Diedrichsen, 2009 
// j.diedrichsen@ucl.ac.uk
////////////////////////////////////////////////////////////////////////////

// include corresponding header file
#include "ManipulandumDR.h"
#include "TextDisplay.h"
#include "S626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 

using namespace std; 
extern S626sManager s626;
extern TextDisplay tDisp;

// defining constants for Manipulandum 
#define CONST_abs 1638400			// How many counts for full circle in 100x interpolation at encoder and 4x on board
//#define CONST_abs 65536			// How many counts for full circle in 4x interpolation at encoder and 4x on board
#define CONST_LEN 0.358
#define Pi 3.141592654


////////////////////////////////////////////////////////////////////////////
/// The Constructor really doesn't anything 
///  
////////////////////////////////////////////////////////////////////////////
ManipulandumDR::ManipulandumDR()  {
	errorState = 0;	
	isFirstUpdate=1;
	setFromLocal(Matrix2D(1,0,0,1),Vector2D(0,0));		// Assume that user wants local (robot) coordinates 
}  

////////////////////////////////////////////////////////////////////////////
/// Destructor does nothing 
///  Should probably free the channels allocated on the s626 board 
////////////////////////////////////////////////////////////////////////////
ManipulandumDR::~ManipulandumDR()  {
}  

////////////////////////////////////////////////////////////////////////////
/// Reads a configuration file for the robot and initializes io channels
/// \param paramfile Parameter files
/// The file contains
///  
/// \li Board for Shoulder Encoder 
/// \li  Channel for Shoulder Encoder 
/// \li  Direction for Shoulder Encoder 
/// \li  Board for Elbow Encoder 
/// \li  Channel for Elbow Encoder 
/// \li  Direction for Elbow Encoder 
/// \li  Board for DA-piston forces 
/// \li  Board for DA-piston forces 
/// \li  Offset for Shoulder encoder [deg]
/// \li  Offset for Elbow Enconder [deg]
/// \li  Length of upper arm length [m]
/// \li  Length of lower arm [m]
/// \li  Tranformation Nm to V at shoulder
/// \li  Tranformation Nm to V at elbow
/// \li  Angle of ATI transducer to arm 
/// \li  Transformation (x-shift) from Local to Global coordinates
/// \li  Transformation (y-shift) from Local to Global coordinates
/// \li  Transformation (rotation 1,1) from Local to Global coordinates
/// \li  Transformation (rotation 1,2) from Local to Global coordinates
/// \li  Transformation (rotation 2,1) from Local to Global coordinates
/// \li  Transformation (rotation 2,2) from Local to Global coordinates
////////////////////////////////////////////////////////////////////////////
void ManipulandumDR::init(string paramfile)
{
	string s; 
	paramfilename=paramfile; 
	ifstream inputFile(paramfilename.c_str(),ios::in);
	if(inputFile.fail()){
		cout<<"ManipulandumDR.init: Robot parameter file could not be opened\n";
		exit(-1);
	} else{
		inputFile>>board_cnt[0]; getline(inputFile,s);		// Board for Shoulder Encoder 
		inputFile>>channel[0]; getline(inputFile,s);		// Channel for Shoulder Encoder 
		inputFile>>dir_cnt[0]; getline(inputFile,s);		// Direction for Shoulder Encoder 
		inputFile>>board_cnt[1]; getline(inputFile,s);		// Board for Elbow Encoder 
		inputFile>>channel[1]; getline(inputFile,s);		// Channel for Elbow Encoder 
		inputFile>>dir_cnt[1]; getline(inputFile,s);		// Direction for Elbow Encoder 
		inputFile>>board_da; getline(inputFile,s);			// Board for DA-piston forces 
		inputFile>>da_offset; getline(inputFile,s);			// Board for DA-piston forces 
		inputFile>>offset[0]; getline(inputFile,s);			// Offset for Shoulder encoder [deg]
		inputFile>>offset[1]; getline(inputFile,s);			// Offset for Elbow Enconder [deg]
		inputFile>>jointLength[0]; getline(inputFile,s);	// Length of upper arm length [m]
		inputFile>>jointLength[1]; getline(inputFile,s);	// Length of lower arm [m]
		inputFile>>torque2volts[0];	getline(inputFile,s);       // Tranformation Nm to V at shoulder
		inputFile>>torque2volts[1];	getline(inputFile,s);       // Tranformation Nm to V at elbow
		inputFile>>ang_ati2arm;getline(inputFile,s);			// Angle of ATI transducer to arm 
		inputFile>>vFromLocal[0];getline(inputFile,s);			// Transformation (x-shift) from Local to Global coordinates
		inputFile>>vFromLocal[1];getline(inputFile,s);			// Transformation (y-shift) from Local to Global coordinates
		inputFile>>AFromLocal[0][0];getline(inputFile,s);		// Transformation (rotation 1,1) from Local to Global coordinates
		inputFile>>AFromLocal[0][1];getline(inputFile,s);		// Transformation (rotation 1,2) from Local to Global coordinates
		inputFile>>AFromLocal[1][0];getline(inputFile,s);		// Transformation (rotation 2,1) from Local to Global coordinates
		inputFile>>AFromLocal[1][1];getline(inputFile,s);		// Transformation (rotation 2,2) from Local to Global coordinates

		if (inputFile.bad() || inputFile.eof()){
			cout<<"ManipulandumDR.init: parameter file in wrong format\n";
			exit(-1);
		} 
	}  
	AToLocal=AFromLocal.getInverse();
	vToLocal=AToLocal*vFromLocal*-1;

	s626.initCounter(channel[0],board_cnt[0]); 
	s626.initCounter(channel[1],board_cnt[1]); 
	force=Vector2D(0,0);
	forceProd=Vector2D(0,0);
	filter.init(Vector2D(0,0));
} 


//////////////////////////////////////////////
/// Set the transformation from local (robot) to World coordinates 
/// also set the inverse transform 
/// \param A 2x2 rotation matrix 
/// \param v 2x1 shift vector 
//////////////////////////////////////////////
void ManipulandumDR::setFromLocal(const Matrix2D &A,const Vector2D &v){ 
	AFromLocal=A;
	vFromLocal=v;
	AToLocal=A.getInverse();
	vToLocal=AToLocal*vFromLocal*-1;
};	  


//////////////////////////////////////////////
/// Set the current position of the robot to be (0,0) 
//////////////////////////////////////////////
void ManipulandumDR::recenter(){ 
	vFromLocal=vFromLocal-positionFilt;
	vToLocal=AToLocal*vFromLocal*-1;
};	  

//////////////////////////////////////////////
/// Gets the raw counter values by reference 
/// \param sh this is where the shoulder counter will be dumped
/// \param el this is where the elbow counter will be dumped
//////////////////////////////////////////////
void ManipulandumDR::getCounter(long &sh,long &el){ 
	sh=sh_cnt;
	el=el_cnt;
} 	  

//////////////////////////////////////////////
/// Main routine that does all the heavy work with update frequency 
/// \param dt time in ms since last update 
//////////////////////////////////////////////
void ManipulandumDR::update(double dt){
		
	// Store the last update 
	lastPosition= position;               //all units are meters
	
	// Get the counters and calculate the angles 
	sh_cnt=s626.getCounter(channel[0],board_cnt[0]); 
	el_cnt=s626.getCounter(channel[1],board_cnt[1]);
	theta[0]=  dir_cnt[0]*((double)(sh_cnt))/CONST_abs*2*Pi-offset[0];      //convert values to radian
	theta[1]=  dir_cnt[1]*((double)(el_cnt))/CONST_abs*2*Pi-offset[1];

	
	// Calculate Jacobians
	Jdx_dtheta[0][0] = jointLength[0]*cos(theta[0]);
	Jdx_dtheta[0][1] = jointLength[1]*sin(theta[1]);
	Jdx_dtheta[1][0] = -jointLength[0]*sin(theta[0]);
	Jdx_dtheta[1][1] = jointLength[1]*cos(theta[1]);
	
	// robot end effector postion, in Cartesian coord centered at robot shoulder joint
	positionLocal[0] =  jointLength[0]*sin(theta[0]) - jointLength[1]*cos(theta[1]);
	positionLocal[1] =  jointLength[0]*cos(theta[0]) + jointLength[1]*sin(theta[1]);
	
	// robot end effector velocity
	position=AFromLocal*positionLocal+vFromLocal; 
	velocity=(position-lastPosition)/dt;
	
	filter.update(position,dt);
	filter.position(positionFilt);
	filter.velocity(velocityFilt);
} 

////////////////////////////////////////////////////////////////////////////////
/// Sets the voltage output to the Motors directly 
/// \param v0 volts to the shoulder motor
/// \param v1 volts to the elbow motor 
/// 
////////////////////////////////////////////////////////////////////////////////
void ManipulandumDR::setVolts(double v0,double v1) { 
	volts[0]=v0;
	volts[1]=v1; 
	s626.outDA(0+da_offset,v0,board_da);
	s626.outDA(1+da_offset,v1,board_da);
} 

////////////////////////////////////////////////////////////////////////////////
/// Sets the force output for the robot in Eucledian coordinates 
/// using the current state of the robot. This needs to be called again as position of the robot changes. 
/// \param force Vector of forces in N 
////////////////////////////////////////////////////////////////////////////////
void ManipulandumDR::setForce(Vector2D force){
	force=AToLocal*force;
	torque=Jdx_dtheta.getTranspose()*force; 
	volts[0]=torque[0]*torque2volts[0]; 
	volts[1]=torque[1]*torque2volts[1]; 
	setVolts(volts[0],volts[1]);
}

////////////////////////////////////////////////////////////////////////////////
/// translates the ATI readings into world coordinates   
/// \param forcereading in ATI coordinate frame
/// \return Force in world coordinate 
/// 
////////////////////////////////////////////////////////////////////////////////
Vector2D ManipulandumDR::ati2World(Vector2D atiforce){ 
	Matrix2D ATI2Arm=rotationMatrixRad(ang_ati2arm); 
    Matrix2D Arm2Local=rotationMatrixRad(theta[1]); 
	Vector2D force=ATI2Arm*atiforce; 
	force=Arm2Local*force; 
	return AFromLocal*force; 
} 
////////////////////////////////////////////////////////////////////////////////
/// Needs a textdisplay to be effective.    
/// \param col column in the text display that will be used. 
/// 
////////////////////////////////////////////////////////////////////////////////

char manipbuffer[60];
void ManipulandumDR::printState(int col){ 
	sprintf(manipbuffer,"Counter: %d %d",sh_cnt,el_cnt);
	tDisp.setText(manipbuffer,1,col);
	sprintf(manipbuffer,"Theta: %3.2f %3.2f",theta[0],theta[1]);
	tDisp.setText(manipbuffer,2,col);
	sprintf(manipbuffer,"Volts: %3.2f %3.2f",volts[0],volts[1]);
	tDisp.setText(manipbuffer,3,col);
	sprintf(manipbuffer,"Torque: %3.2f %3.2f",torque[0],torque[1]);
	tDisp.setText(manipbuffer,4,col);
	sprintf(manipbuffer,"Force: %3.2f %3.2f",force[0],force[1]);
	tDisp.setText(manipbuffer,5,col);
	sprintf(manipbuffer,"Position: %3.2f % 3.2f",position[0],position[1]);
	tDisp.setText(manipbuffer,6,col);
	sprintf(manipbuffer,"VelocityFilt: %3.1f % 3.1f",velocityFilt[0],velocityFilt[1]);
	tDisp.setText(manipbuffer,7,col);

} 