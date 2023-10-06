// -------------------------------------------------------------------------
// PneumaticBox Class 
// fMRI compatible button force box with stimulation devices 
// under a s626 iocard 
// For breakoutbox for multiple boxes
// --------------------------------------------------------------------------

// include corresponding header file
#include "PneumaticBox.h"
#include "S626sManager.h"
#include "TextDisplay.h"
#include <string> 
#include <iostream>
#include <fstream> 
#include <stdio.h>
//#include <NIDAQmx.h>
#include <windows.h>


using namespace std; 
extern S626sManager s626;
extern TextDisplay tDisp;
// for the usb6218 device
//TaskHandle	taskHandle=0;

int initComplete = 0; // Don't allow updating of volts until the boxes have been initiated.


// defining constants for PneumaticBox
// -----------------------------------------------------------
// PneumaticBox::PneumaticBox 
// Constructor 
// -----------------------------------------------------------
PneumaticBox::PneumaticBox()  {
	//scale=502.3/1000*9.81;	//low force box			// average g to volts  scale based on calibration of 17/6/2010 JD
	setScale(10.8424); // for high force box
	//setScale(20);
}  

// -----------------------------------------------------------
// PneumaticBox::PneumaticBox 
// Destructor 
// -----------------------------------------------------------
PneumaticBox::~PneumaticBox()  {
}  


// -----------------------------------------------------------
// PneumaticBox::PneumaticBox 
// Init: 
// -----------------------------------------------------------
void PneumaticBox::setScale(double s){ 
	for (int i=0;i<NUMFINGERS;i++) { 
		scale[i] = s; 
	} 
}

// -----------------------------------------------------------
// PneumaticBox::PneumaticBox 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// 
// -----------------------------------------------------------
bool PneumaticBox::init(int type)
{
	int i,ch=0; 
	switch (type) { 
		case BOX_LEFT:
			ADoffset=0;
			cout<<"board="<<board<<endl;
			break;
		case BOX_RIGHT: 
			ADoffset=8; 
			board=1;
			break;
		default:
			cerr<<"Unknown box type"<<endl;
	}
	board=0;
	//this is for the s626 card
	for (i=0;i<NUMFINGERS;i++) { 
		//----define analoge inpu output
		ADchannel[i]=s626.registerAD(ADoffset+i,10.0,board); //10=> voltage range  
		cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		if (ch<0) 
			exit(-1); 
		forcefilt[i]=0.0;
	}  
	////this is for the usb6218 device
	////---------------------------------------------
	//// DAQmx Configure Code --->Tobias: code from the VoltsUpdate.c example
	////---------------------------------------------
	//int32 err;	
	//char errstr[255];
	//char buffer[255];
	//char local_device[255];
	//buffer[0]=0;
	//local_device[0]=0;
	//unsigned int bufflength;

	//// check device	
	//DAQmxGetSystemInfoAttribute (DAQmx_Sys_DevNames, local_device, 255);    // Get the list of devices connected	
	//bufflength=strlen(local_device);
	//cout<<endl<<"NIDAQ Local name of connected device: "<<local_device<<endl;			

	//if (bufflength>0){
	//	DAQmxGetDeviceAttribute (local_device, DAQmx_Dev_ProductType, buffer, 255);
	//	cout<<"NIDAQ Device type:"<<buffer<<endl<<endl;	
	//} else {
	//	cout<<"NIDAQ Found no NIDAQ USB device connected."<<endl;
	//	if (MessageBox(tDisp.windowHnd, "No NIDAQ USB device connected. Continue?", "Warning",
	//		MB_ICONEXCLAMATION | MB_YESNO)==7) { 			
	//		exit(1); 
	//	}
	//}

	//string deviceName	= local_device;
	//deviceName			+= "/ao0"; // still hard-coded here. beware.

	//localDeviceName = deviceName;
	//NIDeviceType	= buffer;

	//// create task handle
	//err=DAQmxCreateTask("",&taskHandle);
	//if (err<0){
	//		DAQmxGetErrorString (err, errstr, 255);
	//		printf(errstr);
	//		if (MessageBox(tDisp.windowHnd, "Failed to create NIDAQ task. Continue?", "Warning",
	//			MB_ICONEXCLAMATION | MB_YESNO)==7) { 			
	//			exit(1); 
	//		}
	//	} else {
	//	cout << "NIDAQ Task created hand: " << type << endl;}
	//	
	//// create voltage chanenl
	//err=DAQmxCreateAOVoltageChan(taskHandle,deviceName.c_str(),"",-10.0,10.0,DAQmx_Val_Volts,"");
	////err=DAQmxCreateAOVoltageChan(taskHandle,"Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,"");
	//if (err<0){
	//		DAQmxGetErrorString (err, errstr, 255);
	//		printf(errstr);
	//		if (MessageBox(tDisp.windowHnd, "Failed to open NIDAQ voltage channel. Continue?", "Warning",
	//			MB_ICONEXCLAMATION | MB_YESNO)==7) { 			
	//			exit(1); 
	//		}
	//	} 
	//else {
	//	cout << "NIDAQ Volts open" << endl;
 //	} 
 	
	/*DAQmxCreateTask("",&taskHandle);
	i=DAQmxCreateAOVoltageChan(taskHandle,"Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,"");
	if (i!=0) { 
		cout<<"DAQ Error. Error code: "<<i<<endl;
		return false;
	} */
	//---------------------------------------------
	// DAQmx Start Code
	//---------------------------------------------
	//DAQmxStartTask(taskHandle);
	initComplete = 1;
	return true; 
} 



// -----------------------------------------------------------
// StimulatorBox::StimulatorBox 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// CALIBRATION FILE
// 
// -----------------------------------------------------------
bool PneumaticBox::init(int type, string filename)
{
	int i,ch=0; 
	switch (type) { 
		case BOX_LEFT:
			ADoffset=0;
			
			break;
		case BOX_RIGHT: 
			ADoffset=8; 
			//DAchannel[0]=0;
			//DAchannel[1]=1;
			//DAchannel[2]=2;
			//DAchannel[3]=3;
			//DAchannel[4]=-1;		/// Do over the USB device
			board=1;
			break;
		default:
			cerr<<"Unknown box type"<<endl;
	}
	board=0;
	//this is for the s626 card
	for (i=0;i<NUMFINGERS;i++) { 
		//----define analoge inpu output
		ADchannel[i]=s626.registerAD(ADoffset+i,10.0,board); //10=> voltage range  
		//cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		cout<<"Registered AD Ch-"<<ADchannel[i]<<endl;
		if (ch<0) 
			exit(-1); 
		forcefilt[i]=0.0;
	}  
	
	////this is for the usb6218 device
	////---------------------------------------------
	//// DAQmx Configure Code --->Tobias: code from the VoltsUpdate.c example
	////---------------------------------------------
	//int32 err;	
	//char errstr[255];
	//char buffer[255];
	//char local_device[255];
	//buffer[0]=0;
	//local_device[0]=0;
	//unsigned int bufflength;

	//// check device	
	//DAQmxGetSystemInfoAttribute (DAQmx_Sys_DevNames, local_device, 255);    // Get the list of devices connected	
	//bufflength=strlen(local_device);
	//cout<<endl<<"NIDAQ Local name of connected device: "<<local_device<<endl;			

	//if (bufflength>0){
	//	DAQmxGetDeviceAttribute (local_device, DAQmx_Dev_ProductType, buffer, 255);
	//	cout<<"NIDAQ Device type:"<<buffer<<endl<<endl;	
	//} else {
	//	cout<<"NIDAQ Found no NIDAQ USB device connected."<<endl;
	//	if (MessageBox(tDisp.windowHnd, "No NIDAQ USB device connected. Continue?", "Warning",
	//		MB_ICONEXCLAMATION | MB_YESNO)==7) { 			
	//		exit(1); 
	//	}
	//}

	//string deviceName	= local_device;
	//deviceName			+= "/ao0"; // still hard-coded here. beware.

	//localDeviceName = deviceName;
	//NIDeviceType	= buffer;

	//// create task handle
	//err=DAQmxCreateTask("",&taskHandle);
	//if (err<0){
	//		DAQmxGetErrorString (err, errstr, 255);
	//		printf(errstr);
	//		if (MessageBox(tDisp.windowHnd, "Failed to create NIDAQ task. Continue?", "Warning",
	//			MB_ICONEXCLAMATION | MB_YESNO)==7) { 			
	//			exit(1); 
	//		}
	//	} else {
	//	cout << "NIDAQ Task created hand: " << type << endl;}
	//	
	//// create voltage chanenl
	//err=DAQmxCreateAOVoltageChan(taskHandle,deviceName.c_str(),"",-10.0,10.0,DAQmx_Val_Volts,"");
	////err=DAQmxCreateAOVoltageChan(taskHandle,"Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,"");
	//if (err<0){
	//		DAQmxGetErrorString (err, errstr, 255);
	//		printf(errstr);
	//		if (MessageBox(tDisp.windowHnd, "Failed to open NIDAQ voltage channel. Continue?", "Warning",
	//			MB_ICONEXCLAMATION | MB_YESNO)==7) { 			
	//			exit(1); 
	//		}
	//	} 
	//else {
	//	cout << "NIDAQ Volts open" << endl;
 //	} 

	//----set the volts2force factors 
	ifstream inputFile(filename.c_str(),ios::in);
	if(inputFile.fail()){
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
	cout<< "init: set V2F Box "<<type<<"==> T " << scale[0] <<"  I "<< scale[1] <<"  M "<< scale[2] <<"  R "<< scale[3] <<"  L "<< scale[4] <<endl; 
	cout<<endl;
	initComplete = 1;
	return true; 
} 

// -----------------------------------------------------------
// StimulatorBox::StimulatorBox 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// 


// -----------------------------------------------------------
// PneumaticBox::update: 
//  Read the force 
// -----------------------------------------------------------
void PneumaticBox::update(){
	// update the force input 
	int i;
	for (i=0;i<NUMFINGERS;i++) { 
		volts[i]=s626.getAD(ADchannel[i],board);
		force[i]=(volts[i]-baseline[i])*scale[i]; 
		forcefilt[i]=filterconst*forcefilt[i]+(1-filterconst)*force[i]; 
	} 
} 


// -----------------------------------------------------------
// PneumaticBox::setVolts 
// 
// -----------------------------------------------------------
void PneumaticBox::setVolts(double v0,double v1,double v2,double v3,double v4){						///< Send voltage command to finger 
	s626.outDA(0, v0, 0); 
	s626.outDA(1, v1, 0); 
	s626.outDA(2, v2, 0); 
	s626.outDA(3, v3, 0); 
	//float64     data[1] = {v4};

	//if (initComplete == 1) { // If we try to run this before we initiate the NIDAQ things go badly
	//	DAQmxStartTask(taskHandle);
	//	DAQmxWriteAnalogF64(taskHandle,1,1,10.0,DAQmx_Val_GroupByChannel,data,NULL,NULL);
	//	DAQmxStopTask(taskHandle);
	}
	//DAchannel[finger];
	/*voltsOut[finger]=volts; 
	/if (DAchannel[finger]>=0){
		s626.outDA(DAchannel[finger], volts, board); 
	} else {
		float64     data[1] = {volts};
		DAQmxWriteAnalogF64(taskHandle,1,1,10.0,DAQmx_Val_GroupByChannel,data,NULL,NULL);		
	//ToDo case for left box 
	} */
//} 

// -----------------------------------------------------------
// zeroForce 
// 
// -----------------------------------------------------------
void PneumaticBox::zeroForce(double x[NUMFINGERS]){
	int i; 
	for (i=0;i<NUMFINGERS;i++){ 
		baseline[i]=x[i];
	} 
} 




