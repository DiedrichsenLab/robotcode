// -------------------------------------------------------------------------
// HapticBox Class 
// fMRI compatible button force box with stimulation devices 
// under a s626 iocard 
// For breakoutbox for multiple boxes
// --------------------------------------------------------------------------

// include corresponding header file
#include "HapticBox.h"
#include "S626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 
#include <stdio.h>

using namespace std; 
extern S626sManager s626;


// defining constants for HapticBox
// -----------------------------------------------------------
// HapticBox::HapticBox 
// Constructor 
// -----------------------------------------------------------
HapticBox::HapticBox()  {
	for (int i=0; i<NUMFINGERS; i++){
		scale[i]= 502.3/1000*9.81;				// average g to volts  scale based on calibration of 17/6/2010 JD
	}
}  

// -----------------------------------------------------------
// HapticBox::HapticBox 
// Destructor 
// -----------------------------------------------------------
HapticBox::~HapticBox()  {
}  

// -----------------------------------------------------------
// HapticBox::HapticBox 
// setScale: 
// -----------------------------------------------------------
void HapticBox::setScale(double s){ 
	int i;
	/*cout<<endl;
	cout<< "OLD setScale: set V2F Box ==> T " << scale[0] <<"  I "<< scale[1] <<"  M "<< scale[2] <<"  R "<< scale[3] <<"  L "<< scale[4] <<endl; 
	cout<<endl;*/

	for (i=0; i<NUMFINGERS; i++){
		scale[i]= s;
	}
	cout<<endl;
	cout<< "NEW setScale: set V2F Box ==> T " << scale[0] <<"  I "<< scale[1] <<"  M "<< scale[2] <<"  R "<< scale[3] <<"  L "<< scale[4] <<endl; 
	cout<<endl;
}

// -----------------------------------------------------------
// HapticBox::HapticBox 
// setScale: 
// -----------------------------------------------------------
void HapticBox::setScale(double s0, double s1, double s2, double s3, double s4){ 
	/*cout<<endl;
	cout<< "OLD setScale: set V2F Box ==> T " << scale[0] <<"  I "<< scale[1] <<"  M "<< scale[2] <<"  R "<< scale[3] <<"  L "<< scale[4] <<endl; 
	cout<<endl;*/
	
	scale[0]= s0;
	scale[1]= s1;
	scale[2]= s2;
	scale[3]= s3;
	scale[4]= s4;
	
	cout<<endl;
	cout<< "NEW setScale: set V2F Box ==> T " << scale[0] <<"  I "<< scale[1] <<"  M "<< scale[2] <<"  R "<< scale[3] <<"  L "<< scale[4] <<endl; 
	cout<<endl;
}

// -----------------------------------------------------------
// HapticBox::HapticBox 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// CALIBRATION FILE
// 
// -----------------------------------------------------------
bool HapticBox::init(int kind, string filename)
{
	int i,ch=0; 
	switch (kind) { 
		case BOX_LEFT: 
			ADoffset=0; 
			DIOoffset=0; 
			board=0; 
			break;
		case BOX_RIGHT: 
			ADoffset=8;
			DIOoffset=0;
			break;
		default: 
			cerr<<"Unknown box type"<<endl;
	} 

	for (i=0;i<NUMFINGERS;i++) { 
		ADchannel[i]=s626.registerAD(ADoffset+i,10.0,board);
		cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		if (ch<0) 
			exit(-1); 
		forcefilt[i]=0.0;
	}  

	for (i=0;i<NUMFINGERS;i++) { 
		off(i); 
	}  
	//----set the vots2foce factors 
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
	cout<< "init: set V2F Box "<<kind<<"==> T " << scale[0] <<"  I "<< scale[1] <<"  M "<< scale[2] <<"  R "<< scale[3] <<"  L "<< scale[4] <<endl; 
	cout<<endl;
	return true; 
} 

// -----------------------------------------------------------
// HapticBox::HapticBox 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// 
// -----------------------------------------------------------
bool HapticBox::init(int kind)
{
	int i,ch=0; 
	switch (kind) { 
		case BOX_LEFT: 
			ADoffset=0; 
			DIOoffset=0; 
			board=0; 
			break;
		case BOX_RIGHT: 
			ADoffset=8;
			DIOoffset=0;
			break;
		default: 
			cerr<<"Unknown box type"<<endl;
	} 

	for (i=0;i<NUMFINGERS;i++) { 
		ADchannel[i]=s626.registerAD(ADoffset+i,10.0,board);
		cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		if (ch<0) 
			exit(-1); 
		forcefilt[i]=0.0;
	}  

	
	for (i=0;i<NUMFINGERS;i++) { 
		off(i); 
	}  
	return true; 
}

// -----------------------------------------------------------
// HapticBox::update: 
// 
// -----------------------------------------------------------
void HapticBox::update(){
	// update the force input 
	int i;
	for (i=0;i<NUMFINGERS;i++) { 
		volts[i]=s626.getAD(ADchannel[i],board);
		force[i]=(volts[i]-baseline[i])*scale[i]; 
		forcefilt[i]=filterconst*forcefilt[i]+(1-filterconst)*force[i]; 
	} 
} 


// -----------------------------------------------------------
// HapticBox::on/off 
// -----------------------------------------------------------
void HapticBox::on(int finger){
	// update the force input 
	// cout<<"Finger on: " << DIOoffset+finger << endl;
	s626.outDIO(DIOoffset+finger,0); 
	stimulation[finger]=1; 
} 


// -----------------------------------------------------------
// HapticBox::on/off 
// -----------------------------------------------------------
void HapticBox::off(int finger){
	// update the force input 
	//cout<<"Finger off: " << DIOoffset+finger-1 << endl;
	//s626.outDIO(DIOoffset+finger-1,0); 
	// cout<<"Finger off: " << DIOoffset+finger << endl;
	s626.outDIO(DIOoffset+finger,1); 
	stimulation[finger]=0; 
} 


// -----------------------------------------------------------
// zeroForce 
// 
// -----------------------------------------------------------
void HapticBox::zeroForce(double x[NUMFINGERS]){
	int i; 
	for (i=0;i<NUMFINGERS;i++){ 
		baseline[i]=x[i];
	} 
} 




