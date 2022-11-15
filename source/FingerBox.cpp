////////////////////////////////////////////////////////////////////////
// Wrapper class for 5 finger keyboard with or without pneumatic action
// 
// a-yokoi (2015 Nov)
////////////////////////////////////////////////////////////////////////

#include "FingerBox.h"
//#include "StimulatorBox.h"
//#include "PneumaticBox.h"
#include "s626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 
#include <stdio.h>

using namespace std; 
//
//extern S626sManager s626;
// for the usb6218 device
//TaskHandle	taskHandle=0;

// Constructor
FingerBox::FingerBox(){
	LRname[0]		="left";
	LRname[1]		= "right";
	Amplifiers[0]	= "lowForce";
	Amplifiers[1]	= "highForce";
	Keyboards[0]	= "flatBox";
	Keyboards[1]	= "pneumatic";
	Keyboards[2]	= "ergoBox";
	
	sBox 		= new StimulatorBox();
	pBox 		= new PneumaticBox();
	vibrVolts 	= 0;
	vibrSeq 	= 0;
	boardOn 	= 0;
	for (int i=0;i<NUMSTIMULATORS;i++){
		stimulation[i] = 0;
	}
	setFilterConst(0.1); // 0.9 is unfiltered signal
}

// Destructor
FingerBox::~FingerBox(){
	
}
void FingerBox::setFilterConst(double filterconst){
	if (!sBox==NULL){
		sBox->filterconst = filterconst;
	}
	if (!pBox==NULL){
		pBox->filterconst = filterconst;
	}
}
// Initialize
bool FingerBox::init(int type){
	delete pBox; // assume using StimulatorBox
	return sBox->init(type);
}
bool FingerBox::init(int type, string filename){
	return init(type,0,filename); // assume using StimulatorBox
}
bool FingerBox::init(int type, int box, string filename){
	switch (box){
	case 0: // using StimulatorBox
		delete pBox; // discard pBox
		return sBox->init(type,filename);
		break;
		
	case 1: // using PneumaticBox
		delete sBox; // discard sBox
		return pBox->init(type,filename);
		break;
	
	case 2: // using StimulatorBox (assuming ergonomic keyboard)
		delete pBox; // discard pBox
		return sBox->init(type,filename);
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
bool FingerBox::init(){
	// interactively initialize stimulation box
	int amp;
	int LorR;
	int box;
	string caldir = "c:/robot/calib/";
	// Ask about amplifier
	cout<<"Identifying the type of amplifier..."<<endl;
	cout<<"   Which amplifier is this? (0:low force,1:high force):";
	cin>>amp;
	if (amp>1){exit(-1);}
	// Ask about hand
	cout<<"Identifying the type of device..."<<endl;
	cout<<"   Which hand is this device for? (0:left,1:right):";
	cin>>LorR;
	if (LorR>1){exit(-1);}
	// Ask about device type
	cout<<"   What is the type of "<<LRname[LorR].c_str()<<" device? (0:flat keyboard,1:pnaumatic keyboard,2:ergonomic keyboard):";
	cin>>box;
	if (box>1){exit(-1);}
	CalibFileName[LorR] = caldir+LRname[LorR]+"_"+Amplifiers[amp]+"_"+Keyboards[box]+".txt";
	// Initialize
	return init(LorR,box,CalibFileName[LorR]);
}
void FingerBox::readSeq(string filename,int seq){
	switch (box){
	case 0: // using StimulatorBox
		sBox->readSeq(filename,seq);
		break;
		
	case 1: // using PneumaticBox
		// undefined function in PneumaticBox
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
void FingerBox::setScale(double s){
	switch (box){
	case 0: // using StimulatorBox
		sBox->setScale(s);
		break;
		
	case 1: // using PneumaticBox
		pBox->setScale(s);
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
void FingerBox::setScale(double s0, double s1 ,double s2 , double s3 , double s4){
	switch (box){
	case 0: // using StimulatorBox
		sBox->setScale(s0,s1,s2,s3,s4);
		break;
		
	case 1: // using PneumaticBox
		pBox->setScale((s0+s1+s2+s3+s4)/5); // undefined for pneumaticBox
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
void FingerBox::update(){
	switch (box){
	case 0: // using StimulatorBox
		sBox->update();
		break;
		
	case 1: // using PneumaticBox
		pBox->update();
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
void FingerBox::zeroForce(double volts[NUMFINGERS]){
	switch (box){
	case 0: // using StimulatorBox
		sBox->zeroForce(volts);
		break;
		
	case 1: // using PneumaticBox
		pBox->zeroForce(volts);
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
void FingerBox::setVolts(double v0,double v1,double v2,double v3,double v4){
	switch (box){
	case 0: // using StimulatorBox
		// undefined function for stimulatorBox
		break;
		
	case 1: // using PneumaticBox
		pBox->setVolts(v0,v1,v2,v3,v4);
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
double FingerBox::getForce(int i) {
	switch (box){
	case 0: // using StimulatorBox
		return sBox->getForce(i);
		break;
		
	case 1: // using PneumaticBox
		return pBox->getForce(i);
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
double FingerBox::getForceFilt(int i) {
	switch (box){
	case 0: // using StimulatorBox
		return sBox->getForceFilt(i);
		break;
		
	case 1: // using PneumaticBox
		return pBox->getForceFilt(i);
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
double FingerBox::getVolts(int i) {
	switch (box){
	case 0: // using StimulatorBox
		return sBox->getVolts(i);
		break;
		
	case 1: // using PneumaticBox
		return pBox->getVolts(i);
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
void FingerBox::on(int finger){
	switch (box){
	case 0: // using StimulatorBox
		sBox->on(finger);
		break;
		
	case 1: // using PneumaticBox
		// undefined function for pneumaticBox
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
void FingerBox::off(int finger){
	switch (box){
	case 0: // using StimulatorBox
		sBox->off(finger);
		break;
		
	case 1: // using PneumaticBox
		// undefined function for pneumaticBox
		break;
		
	default: // otherwise
		exit(-1);
		break;
	}
}
