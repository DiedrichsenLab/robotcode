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

#define Pi 3.141592654

using namespace std; 
extern S626sManager s626;


// defining constants for WristManipulandum
// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// Constructor 
// -----------------------------------------------------------
WristManipulandum::WristManipulandum()  {
	for (int i=0; i<NUMJOINTS; i++){
		scale[i]= 502.3/1000*9.81; // initial value for voltage to angle scale
	}
	length = 0.3; // arbitrary initial value for the length of fencing grip
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
void WristManipulandum::setScale(double s0, double s1){ 
	scale[0]= s0;
	scale[1]= s1;
	
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
bool WristManipulandum::init(int kind, string filename) // ToDo: adjust AD channel numbers
{
	int i,ch=0; 
	switch (kind) { 
		case BOX_LEFT: 
			ADoffset=0; 
			DAchannel=0;
			DIOoffset=0; 
			board=0; 
			cout<<"Left Wrist Manipulandum Loaded..."<<endl;
			break;
		case BOX_RIGHT: 
			ADoffset=8;
			DAchannel=1;
			DIOoffset=5;
			cout<<"Right Wrist Manipulandum Loaded..."<<endl;
			break;
		default: 
			cerr<<"Unknown box type"<<endl;
	} 

	for (i=0;i<NUMJOINTS;i++) { 
		ADchannel[i]=s626.registerAD(ADoffset+i,10.0,board);
		cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		if (ch<0) 
			exit(-1); 
		angfilt[i]=0.0;
		angvelfilt[i]=0.0;
		posfilt[i]=0.0;
		velfilt[i]=0.0;
		positionFilt[i]=0.0;
		velocityFilt[i]=0.0;
		force[i]=0.0;
		forceProd[i]=0.0;
	}  

	/**
	for (i=0;i<NUMJOINTS;i++) { 
		off(i); 
	}  
	**/
	
	//----set the volts2angle factors 
	ifstream inputFile(filename.c_str(),ios::in);
	
	if(inputFile ==0){
		cout<<"Couldn't open file: " <<filename<<endl;
		exit(-1);
	} else{
		// ignore headers in file
		i=0;
		while (!(inputFile.bad() || inputFile.eof())) { 
			inputFile>>scale[i];
			i=i+1; 		
		} 
	}
	
	cout<<endl;	
	cout<< "NEW setScale: set V2F Box ==>"<<endl;
	for (i=0; i<NUMJOINTS; i++){	
		cout<<"Joint" << i <<": "<< scale[i] <<endl; 
	}
	cout<<endl;
	
	return true; 
} 

// -----------------------------------------------------------
// WristManipulandum::WristManipulandum 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// 
// -----------------------------------------------------------
bool WristManipulandum::init(int kind)
{
	int i,ch=0; 
	switch (kind) { 
		case BOX_LEFT: 
			ADoffset=0; 
			DAchannel=0;
			DIOoffset=0; 
			board=0; 
			break;
		case BOX_RIGHT: // need to adjust 
			ADoffset=8;
			DAchannel=1;
			DIOoffset=5;
			break;
		default: 
			cerr<<"Unknown box type"<<endl;
	} 

	for (i=0;i<NUMJOINTS;i++) { 
		ADchannel[i]=s626.registerAD(ADoffset+i,10.0,board); // may need modification for range
		cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		if (ch<0) 
			exit(-1); 
		angfilt[i]=0.0;
		angvelfilt[i]=0.0;
		posfilt[i]=0.0;
		velfilt[i]=0.0;
		positionFilt[i]=0.0;
		velocityFilt[i]=0.0;
		force[i]=0.0;
		forceProd[i]=0.0;
	}  

	/**
	for (i=0;i<NUMJOINTS;i++) { 
		off(i); 
	}  
	**/
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
		ang[i]=(volts[i]-baseline[i])*scale[i]; // joint angle
	} 	
	// update Kalman filter to get filtered joint angles
	filter.update(Vector2D(ang[0],ang[1]),dt);
	filter.position(angfilt);
	filter.velocity(angvelfilt);
	// calculate display position from joint angle	
	Manip2Disp(ang,angvel,posfilt,velfilt);	
	position = Vector2D(pos[0],pos[1]);
	velocity = Vector2D(vel[0],vel[1]);
	positionFilt = Vector2D(posfilt[0],posfilt[1]);
	velocityFilt = Vector2D(velfilt[0],velfilt[1]);	
	// dummy variables (only exist for compatibility with ManipulandumDR class)
	force 		= Vector2D(0,0);
	forceProd 	= Vector2D(0,0);
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

/**
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
**/

// -----------------------------------------------------------
// set baseline
// 
// -----------------------------------------------------------
void WristManipulandum::setZero(double x[NUMJOINTS]){
	for (int i=0;i<NUMJOINTS;i++){ 
		baseline[i]=x[i];
	} 
} 




