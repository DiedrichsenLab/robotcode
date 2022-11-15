// -------------------------------------------------------------------------
/// WristManipulandum Class 
/// fMRI compatible 2-DoF wrist movement device
/// under a s626 iocard 
/// For breakoutbox for multiple boxes
// Atsushi Yokoi, 2014
// --------------------------------------------------------------------------

// include corresponding header file
#include "WristManipulandum.h"
#include "S626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 
#include <stdio.h>
#include <math.h>

using namespace std; 
extern S626sManager s626;

#define Pi 3.141592654
#define VPD_LEFT_HORIZONTAL		4.98/330 //(2.74/180)			///< volts per degree, left horizontal encoder 
#define VPD_LEFT_VERTICAL		4.98/330//(2.62/180)
#define VPD_RIGHT_HORIZONTAL	4.98/330//(2.76/180)
#define VPD_RIGHT_VERTICAL		4.98/330//(2.62/180)

// defining constants for WristManipulandum
// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// Constructor 
// -----------------------------------------------------------
WristManipulandum::WristManipulandum()  {
	for (int i=0; i<NUMJOINTS; i++){
		scale[i]= 0.5;//502.3/1000*9.81; // initial value for voltage to angle scale
	}
	length = 0.3; 	// arbitrary initial value for the length of fencing grip
	dt = 1/1000;	//1/1000; // Default update rate
}  

// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// Destructor 
// -----------------------------------------------------------
WristManipulandum::~WristManipulandum()  {
}  

// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// setScale: 
// -----------------------------------------------------------
void WristManipulandum::setScale(double s){ 
	cout<<endl;
	cout<< "NEW setScale: set V2F Box ==>"<<endl;
	for (int i=0; i<NUMJOINTS; i++){
		scale[i]= s;
		cout<<"Joint" << i <<": "<< scale[i] <<endl; 
	}
	cout<<endl;
}

// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// setScale: 
// -----------------------------------------------------------
void WristManipulandum::setScale(Vector2D mScale){ 
	scale = mScale;
	
	cout<<endl;	
	cout<< "NEW setScale: set V2F Box ==>"<<endl;
	for (int i=0; i<NUMJOINTS; i++){	
		cout<<"Joint" << i <<": "<< scale[i] <<endl; 
	}
	cout<<endl;
}

// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// Init:
// LEFT Manipulandum: 0 
// RIGHT Manipulandum: 1 
// CALIBRATION FILE
// 
// -----------------------------------------------------------
bool WristManipulandum::init(int kind, string filename) // TODO: input calib file
{
	string s; 
	paramfilename=filename; 
	ifstream inputFile(paramfilename.c_str(),ios::in);
	if(inputFile ==0){
		cout<<"WristManipulandum.init: Robot parameter file could not be opened\n";
		exit(-1);
	} else{
		
		inputFile>>		outputVolts; 		getline(inputFile,s);				// Output voltage in channel 42
		inputFile>>		TBlockPinLeft[0]; 	getline(inputFile,s);				// Pin in Terminal Block for Left Horizontal Encoder
		inputFile>>		TBlockPinLeft[1]; 	getline(inputFile,s);				// Pin in Terminal Block for Left Vertical Encoder
		inputFile>>		TBlockPinRight[0]; 	getline(inputFile,s);				// Pin in Terminal Block for Left Horizontal Encoder
		inputFile>>		TBlockPinRight[1]; 	getline(inputFile,s);				// Pin in Terminal Block for Left Vertical Encoder
		inputFile>>		board; 				getline(inputFile,s);				// Board is usually 0
		inputFile>>		initialAng[0];		getline(inputFile,s);				// Initial angles for Kalman Filter
		inputFile>>		initialAng[1]; 		getline(inputFile,s);				// Initial angles for Kalman Filter
		inputFile>>		acelAngNoiseVar; 	getline(inputFile,s);				// KalmanFilter: Term for aceleration noise variance
		inputFile>>		angNoiseVar; 		getline(inputFile,s);				// KalmanFilter: Term for position noise variance
		inputFile>>		transf_int_type;	getline(inputFile,s);				// trans type, LIN, SIN or TAN
		inputFile>>		ang_or_angfilt;		getline(inputFile,s);				// angles (0) or angles filtered (1)

		if (inputFile.bad() || inputFile.eof()){
			cout<<"WristManipulandum.init: parameter file in wrong format\n";
			exit(-1);
		} 
	}
	s626.outDA(0,outputVolts,board); cout << "Channel 42 assigned 5 volts" << endl;//18
	filterang.init(initialAng, acelAngNoiseVar ,angNoiseVar, dt);						// initialize filter angles
	int i;
	
	switch (kind) { 
		case BOX_LEFT:
		
			ADchannel[0]=s626.registerAD((TBlockPinLeft[0]/2-3), outputVolts, 0);		// 9 corresponds to pin 24, hence 15
			cout<<"ch: "<<"                     " <<ADchannel[0]<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
			ADchannel[1]=s626.registerAD((TBlockPinLeft[1]/2-3), outputVolts, 0);
			cout<<"ch: "<<"          " <<ADchannel[1]<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
			
			vpd_h = VPD_LEFT_HORIZONTAL;		// assign calibration for each device
			vpd_v = VPD_LEFT_VERTICAL;
			cout<<"Left Wrist Manipulandum Loaded..."<<endl;
			break;
		case BOX_RIGHT: 
		
			ADchannel[0]=s626.registerAD((TBlockPinRight[0]/2-3), outputVolts, 0);
			cout<<"ch: "<<"                     " <<ADchannel[0]<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
			ADchannel[1]=s626.registerAD((TBlockPinRight[1]/2-3), outputVolts, 0);
			cout<<"ch: "<<"                     " <<ADchannel[1]<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
			
			vpd_h = VPD_RIGHT_HORIZONTAL;		// assign calibration for each device
			vpd_v = VPD_RIGHT_VERTICAL;
			cout<<"Right Wrist Manipulandum Loaded..."<<endl;
			break;
		default: 
			cerr<<"Unknown box type"<<endl;
			exit(-1);
	} 

	for (i=0;i<NUMJOINTS;i++) { 
		off(i); 
	}  

	tTransf = static_cast<transformation>(transf_int_type);	
	
	cout<<endl;	
	cout<< "NEW setScale: set V2F Box ==>"<<endl;
	for (i=0; i<NUMJOINTS; i++){	
		cout<<"Joint" << i <<": "<< scale[i] <<endl; 
	}
	cout<<endl;
	
	angfilt[i]=1;
	angvelfilt[i]=1;
	posfilt[i]=1;
	velfilt[i]=1;
	positionFilt[i]=0.0;
	velocityFilt[i]=0.0;
	dV[i] = 0.0;
	
	//cout << "vdp " << vpd_h << "\t" << vpd_v << endl;
	
	return true; 
} 

// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// -----------------------------------------------------------
bool WristManipulandum::init(int kind)
{
	filterang.init(Vector2D(0,0), 100 ,0.01);			// initialize filter for angles
	
	s626.outDA(0,5.0,0); cout << "Channel 42 assigned 5 volts" << endl;//18
	int i,ch=0; 
	switch (kind) { 
		case BOX_LEFT: 
			ADoffset=9;	//0
			board=0; 
			cout<<"BOX_LEFT manip was initialized!"<<endl;
			vpd_h = VPD_LEFT_HORIZONTAL;		// assign calibration for each device
			vpd_v = VPD_LEFT_VERTICAL;
			break;
		case BOX_RIGHT: // need to adjust 
			ADoffset=11;		//8
			cout<<"BOX_RIGHT manip was initialized!"<<endl;
			vpd_h = VPD_RIGHT_HORIZONTAL;		// assign calibration for each device
			vpd_v = VPD_RIGHT_VERTICAL;
			break;
		default: 
			cerr<<"Unknown box type"<<endl;
			exit(-1);
	} 

	for (i=0;i<NUMJOINTS;i++) { 
		ADchannel[i]=s626.registerAD(ADoffset+i,5,0); // may need modification for range
		cout<<ch<<"    " <<ADchannel[i]<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		if (ADchannel[i]<0){ 
			exit(-1); 
		}
		angfilt[i]=0.0;
		angvelfilt[i]=0.0;
		posfilt[i]=0.0;
		velfilt[i]=0.0;
		positionFilt[i]=0.0;
		velocityFilt[i]=0.0;
		dV[i] = 0.0;
	}  
	
	for (i=0;i<NUMJOINTS;i++) { 
		off(i); 
	}  

	tTransf = LIN;
	
	return true; 
}

// -----------------------------------------------------------
// WristManipulandum::update: 
// 
// -----------------------------------------------------------
void WristManipulandum::update(double dt){
	// update the sensor input and calculate unfiltered joint angle
	
	for (int i=0;i<NUMJOINTS;i++) { 
		volts[i]=s626.getAD(ADchannel[i],board);
	}
	ang[0] = (volts[0]-baseline[0])/vpd_h; // joint angle
	ang[1] = (volts[1]-baseline[1])/vpd_v; // joint angle

	// update Kalman filter to get filtered joint angles
	filterang.position(angfilt);
	filterang.velocity(angvelfilt);
//	filterang.update(Vector2D(ang[0],ang[1]),dt);
	filterang.update(ang, dt);
	
	
	// angles to position in world coordinates, unfilterd and filtered
	ang2pos(tTransf, ang, pos);	
	ang2pos(tTransf, angfilt, posfilt);
	ang2pos(tTransf, angvelfilt, velfilt);
	
	// calculate display position from joint angle	
//	Manip2Disp(ang,angvel,posfilt,velfilt);	

	position = Vector2D(pos[0],pos[1]);
	velocity = Vector2D(vel[0],vel[1]);
	positionFilt = Vector2D(posfilt[0],posfilt[1]);
	velocityFilt = Vector2D(velfilt[0],velfilt[1]);	

} 

// -----------------------------------------------------------
// WristManipulandum::transfromManipulandumCoordinate2DisplayCoordinate
// -----------------------------------------------------------
void WristManipulandum::Manip2Disp(double pm[], double vm[], double pd[], double vd[]){
	for (int i=0; i<NUMJOINTS; i++){		
			pd[i] = length*sin(pm[i]*Pi/180);			
			vd[i] = length*cos(pm[i]*Pi/180)*vm[i];		
	}
}


// -----------------------------------------------------------
// WristManipulandum::on/off 
// -----------------------------------------------------------
void WristManipulandum::on(int joint){
	s626.outDIO(DIOoffset+joint,0); 
} 


// -----------------------------------------------------------
// WristManipulandum::on/off 
// -----------------------------------------------------------
void WristManipulandum::off(int joint){
	s626.outDIO(DIOoffset+joint,1); 
} 


// -----------------------------------------------------------
// set baseline
// 
// -----------------------------------------------------------
void WristManipulandum::setZero(double x[NUMJOINTS]){
	for (int i=0;i<NUMJOINTS;i++){ 
		baseline[i]=x[i];
	} 
} 

inline void WristManipulandum::ang2pos(transformation tTrans, double angIn[], double posOut[]){
	// this is a map to world position! not at encoder level
	int joint;
	switch (tTrans){
	case LIN:
		for (joint = 0; joint < NUMJOINTS; joint++){
			posOut[joint] = angIn[joint]*length*scale[joint];//scale[1];//scale[joint];
		} break;
	case SIN:
		for (joint = 0; joint < NUMJOINTS; joint++){
			posOut[joint] = sin(angIn[joint]*Pi/180)*length*scale[joint];//scale[1];//scale[joint];
		} break;
	case TAN:
		for (joint = 0; joint < NUMJOINTS; joint++){
			posOut[joint] = tan(angIn[joint]*Pi/180)*length*scale[joint];//scale[1];//scale[joint];
		} break;
	}
}	


