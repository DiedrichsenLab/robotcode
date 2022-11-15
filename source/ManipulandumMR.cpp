////////////////////////////////////////////////////////////////////////////
/// ManipulandumMR.cpp
/// 
/// 
/// written for a 2-link robot with absolute rotatry encoders (MicroESystem) 
/// under a s626 iocard 
/// 
/// 
/// Uses Vector2D and Matrix2D classes for compact code; 
/// 
// Joern Diedrichsen, 2007 
// j.diedrichsen@bangor.ac.uk
////////////////////////////////////////////////////////////////////////////

// include corresponding header file
#include "ManipulandumMR.h"
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


// -----------------------------------------------------------
// Manipulandum::Manipulandum 
// Constructor 
// -----------------------------------------------------------
ManipulandumMR::ManipulandumMR()  {
	errorState = 0;	
	isFirstUpdate=1;
	offsetPiston=0;
	overdrive=0; 
	tauPiston=0.06;										///< Time constant for the pistons 
	setFromLocal(Matrix2D(1,0,0,1),Vector2D(0,0));		// Assume that user wants local (robot) coordinates 
}  

// -----------------------------------------------------------
// ManipulandumMR::ManipulandumMR 
// Destructor 
// -----------------------------------------------------------
ManipulandumMR::~ManipulandumMR()  {
}  



// -----------------------------------------------------------
// ManipulandumMR::init
/// reads calibration from parameter file 
/// and calculates all required variables from that 
// -----------------------------------------------------------
void ManipulandumMR::init(string paramfile)
{
	string s; 
	paramfilename=paramfile; 
	ifstream inputFile(paramfilename.c_str(),ios::in);
	if(inputFile ==0){
		cout<<"ManipulandumMR.init: Robot parameter file could not be opened\n";
		exit(-1);
	} else{
		inputFile>>board_cnt[0]; getline(inputFile,s);		/// Board for Shoulder Encoder 
		inputFile>>channel[0]; getline(inputFile,s);		/// Channel for Shoulder Encoder 
		inputFile>>dir_cnt[0]; getline(inputFile,s);		/// Direction for Shoulder Encoder 
		inputFile>>board_cnt[1]; getline(inputFile,s);		/// Board for Elbow Encoder 
		inputFile>>channel[1]; getline(inputFile,s);		/// Channel for Elbow Encoder 
		inputFile>>dir_cnt[1]; getline(inputFile,s);		/// Direction for Elbow Encoder 
		inputFile>>board_da; getline(inputFile,s);			/// Board for DA-piston forces 
		inputFile>>offset[0]; getline(inputFile,s);			/// Offset for Shoulder encoder [deg]
		inputFile>>offset[1]; getline(inputFile,s);			/// Offset for Elbow Enconder [deg]
		inputFile>>jointLength[0]; getline(inputFile,s);	/// Length of upper arm length [m]
		inputFile>>jointLength[1]; getline(inputFile,s);	/// Length of lower arm [m]
		inputFile>>linkLength[0]; getline(inputFile,s);		/// Link Length for Piston, Shoulder [m]
		inputFile>>linkLength[1];	getline(inputFile,s);	/// Link Length for Piston, Elbow [m]
		inputFile>>force2volts;	getline(inputFile,s);       /// Tranformation N to V at pistons 
		inputFile>>tauPiston;								/// Time constant for the pistons [ms]
		if (inputFile.bad() || inputFile.eof()){
			cout<<"ManipulandumMR.init: parameter file in wrong format\n";
			exit(-1);
		} 
		diagLength[0] = sqrt(linkLength[0]*linkLength[0]+CONST_LEN*CONST_LEN);
		diagLength[1] = sqrt(linkLength[1]*linkLength[1]+CONST_LEN*CONST_LEN);
		alpha[0]=asin(CONST_LEN/diagLength[0]);
		alpha[1]=asin(CONST_LEN/diagLength[1]);

	}  
	s626.initCounter(channel[0],board_cnt[0]); 
	s626.initCounter(channel[1],board_cnt[1]); 
	force=Vector2D(0,0);
	forceProd=Vector2D(0,0);
	forcePiston=Vector2D(0,0);
	forcePistonProd=Vector2D(0,0);
	filter.init(Vector2D(0,0));
} 


// -----------------------------------------------------------
/// Set the transformation from local (robot) to World coordinates 
/// also set the inverse transform 
// -----------------------------------------------------------
void ManipulandumMR::setFromLocal(const Matrix2D &A,const Vector2D &v){ 
	AFromLocal=A;
	vFromLocal=v;
	AToLocal=A.getInverse();
	vToLocal=AToLocal*vFromLocal*-1;
};	  


// -----------------------------------------------------------
/// Set the current position of the robot to be (0,0) 
// -----------------------------------------------------------
void ManipulandumMR::recenter(){ 
	vFromLocal=vFromLocal-positionFilt;
	vToLocal=AToLocal*vFromLocal*-1;
};	  

// -----------------------------------------------------------
/// Gets the raw counter values by reference 
// -----------------------------------------------------------
void ManipulandumMR::getCounter(long &sh,long &el){ 
	sh=sh_cnt;
	el=el_cnt;
} 	  

// -----------------------------------------------------------
/// ManipulandumMR::update
/// Updates the sensory state of the robot 
/// uses kalman filter to estimate velocity and position 
// -----------------------------------------------------------
void ManipulandumMR::update(double dt){
		
	/// Store the last update 
	lastPosition= position;               //all units are meters
	
	/// Get the counters and calculate the angles 
	sh_cnt=s626.getCounter(channel[0],board_cnt[0]); 
	el_cnt=s626.getCounter(channel[1],board_cnt[1]);
	theta[0]=  dir_cnt[0]*((double)(sh_cnt))/CONST_abs*2*Pi-offset[0];      //convert values to radian
	theta[1]=  dir_cnt[1]*((double)(el_cnt))/CONST_abs*2*Pi-offset[1];
	phi[0]=alpha[0]+theta[0];
	phi[1]=alpha[1]-theta[1];

	/// Calculate the piston length 
	pistonLength[0] = sqrt(linkLength[0]*linkLength[0]+diagLength[0]*diagLength[0]-2*linkLength[0]*diagLength[0]*cos(phi[0]));
	pistonLength[1] = sqrt(linkLength[1]*linkLength[1]+diagLength[1]*diagLength[1]-2*linkLength[1]*diagLength[1]*cos(phi[1]));
	
	/// Calculate Jacobians
	Jdx_dtheta[0][0] = jointLength[0]*cos(theta[0]);
	Jdx_dtheta[0][1] = jointLength[1]*sin(theta[1]);
	Jdx_dtheta[1][0] = -jointLength[0]*sin(theta[0]);
	Jdx_dtheta[1][1] = jointLength[1]*cos(theta[1]);
	Jdtheta_dl[0][0] = +pistonLength[0]/(linkLength[0]*diagLength[0]*sin(phi[0]));
	Jdtheta_dl[0][1] = 0;
	Jdtheta_dl[1][0] = 0;
	Jdtheta_dl[1][1] = -pistonLength[1]/(linkLength[1]*diagLength[1]*sin(phi[1]));
	
	/// robot end effector postion, in Cartesian coord centered at robot shoulder joint
	positionLocal[0] =  jointLength[0]*sin(theta[0]) - jointLength[1]*cos(theta[1]);
	positionLocal[1] =  jointLength[0]*cos(theta[0]) + jointLength[1]*sin(theta[1]);
	
	/// robot end effector velocity
	position=AFromLocal*positionLocal+vFromLocal; 
	velocity=(position-lastPosition)/0.005;
	
	///Forces produced at the pistons and endpoint 
	forcePistonProd=forcePistonProd+(forcePiston-forcePistonProd)/tauPiston*dt;
	forceProd=AFromLocal*Jdx_dtheta.getTranspose().getInverse()*Jdtheta_dl.getInverse()*forcePistonProd; 

	filter.update(position,dt);
	filter.position(positionFilt);
	filter.velocity(velocityFilt);
} 


//-------------------------------------------------------------------------------
/// ManipulandumMR :: setValves 
/// Sets the voltage output to the Valves directly 
// ------------------------------------------------------------------------------
void ManipulandumMR::setValves(double v0,double v1,double v2,double v3) { 
	s626.outDA(0,v0,board_da);
	s626.outDA(1,v1,board_da);
	s626.outDA(2,v2,board_da);
	s626.outDA(3,v3,board_da);
} 

//-------------------------------------------------------------------------------
/// ManipulandumMR :: setForce 
/// Sets the force output for the robot in Eucledian coordinates 
/// using the current state of the robot (needs to be called again as 
/// the robot moves 
/// Valve 0: Shoulder push 
/// Valve 1: Shoulder pull 
/// Valve 2: Elbow push 
/// Valve 3: Elbow pull 
// ------------------------------------------------------------------------------
void ManipulandumMR::setForce(Vector2D force){
	int i; 
	force=AToLocal*force+AToLocal*(force-forceProd)*overdrive;
	forcePiston=Jdtheta_dl*(Jdx_dtheta.getTranspose())*force; 
	for (i=0;i<2;i++) {
		if (forcePiston[i]<0) { 
			volts[i*2]=offsetPiston*force2volts;
			volts[i*2+1]=(-forcePiston[i]+offsetPiston)*force2volts; 
		} else { 
			volts[i*2]=(forcePiston[i]+offsetPiston)*force2volts; 
			volts[i*2+1]=offsetPiston*force2volts;
		} 
	} 
	setValves(volts[0],volts[1],volts[2],volts[3]);
}



// -----------------------------------------------
/// prints the state of the robot to the Text display
// --------------------------------------------------------------------------------
char manipbuffer[60];
void ManipulandumMR::printState(){ 
	sprintf(manipbuffer,"Counter: %d %d",	sh_cnt,el_cnt);
	tDisp.setText(manipbuffer,1,0);
	sprintf(manipbuffer,"Theta: %3.2f %3.2f",theta[0]/Pi*180,theta[1]/Pi*180);
	tDisp.setText(manipbuffer,2,0);
	sprintf(manipbuffer,"Phi: %3.2f %3.2f",phi[0],phi[1]);
	tDisp.setText(manipbuffer,3,0);
	sprintf(manipbuffer,"pLength: %3.2f %3.2f",pistonLength[0],pistonLength[1]);
	tDisp.setText(manipbuffer,4,0);
	sprintf(manipbuffer,"pForce: %3.2f %3.2f",forcePiston[0],forcePiston[1]);
	tDisp.setText(manipbuffer,5,0);
	sprintf(manipbuffer,"Force: %3.2f %3.2f",force[0],force[1]);
	tDisp.setText(manipbuffer,6,0);
	sprintf(manipbuffer,"Position: %3.2f % 3.2f",position[0],position[1]);
	tDisp.setText(manipbuffer,7,0);
	sprintf(manipbuffer,"VelocityFilt: %3.1f % 3.1f",velocityFilt[0],velocityFilt[1]);
	tDisp.setText(manipbuffer,8,0);

} 