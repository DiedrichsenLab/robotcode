// -------------------------------------------------------------------------
// StimulatorBox Class 
// fMRI compatible button force box with stimulation devices 
// under a s626 iocard 
// For breakoutbox for multiple boxes
// --------------------------------------------------------------------------

// include corresponding header file
#include "StimulatorBox.h"
#include "S626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 
#include <stdio.h>

using namespace std; 
extern S626sManager s626;


// defining constants for StimulatorBox
// -----------------------------------------------------------
// StimulatorBox::StimulatorBox 
// Constructor 
// -----------------------------------------------------------
StimulatorBox::StimulatorBox()  {
	for (int i=0; i<NUMFINGERS; i++){
		scale[i]= 502.3/1000*9.81;				// average g to volts  scale based on calibration of 17/6/2010 JD
	}
}  

// -----------------------------------------------------------
// StimulatorBox::StimulatorBox 
// Destructor 
// -----------------------------------------------------------
StimulatorBox::~StimulatorBox()  {
}  

// -----------------------------------------------------------
// StimulatorBox::StimulatorBox 
// setScale: 
// -----------------------------------------------------------
void StimulatorBox::setScale(double s){ 
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
// StimulatorBox::StimulatorBox 
// setScale: 
// -----------------------------------------------------------
void StimulatorBox::setScale(double s0, double s1, double s2, double s3, double s4){ 
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
// StimulatorBox::StimulatorBox 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// CALIBRATION FILE
// 
// -----------------------------------------------------------
bool StimulatorBox::init(int kind, string filename)
{
	int i,ch=0; 
	switch (kind) { 
		case BOX_LEFT: 
			ADoffset=0; 
			DAchannel=0;
			DIOoffset=0; 
			board=0; 
			break;
		case BOX_RIGHT: 
			ADoffset=8;
			DAchannel=1;
			DIOoffset=5;
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

	for (i=0;i<MAXNUMSEQ;i++) {			/// Initialize all sequences to 100 Hz
		stimseq[i][0]=1; 
		stimseq[i][1]=0;
		lengthSeq[i]=2; 
	} 
	vib=0; 
	vibrSeq=0;
	vibrVolts=0; 
	for (i=0;i<NUMSTIMULATORS;i++) {  /// Set all vibrators to 0 
		stimulation[i]=0;
	} 
	for (i=0;i<NUMFINGERS;i++) { 
		off(i); 
	}  
	//----set the vots2foce factors 
	ifstream inputFile(filename.c_str(),ios::in);
	
	if(inputFile.fail()){ //Ali: if(inputFile ==0) -> if(inputFile.fail())
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
// StimulatorBox::StimulatorBox 
// Init:
// LEFT BOX: 0 
// RIGHT BOX: 1 
// 
// -----------------------------------------------------------
bool StimulatorBox::init(int kind)
{
	int i,ch=0; 
	switch (kind) { 
		case BOX_LEFT: 
			ADoffset=0; 
			DAchannel=0;
			DIOoffset=0; 
			board=0; 
			break;
		case BOX_RIGHT: 
			ADoffset=8;
			DAchannel=1;
			DIOoffset=5;
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

	for (i=0;i<MAXNUMSEQ;i++) {			/// Initialize all sequences to 100 Hz
		stimseq[i][0]=1; 
		stimseq[i][1]=0;
		lengthSeq[i]=2; 
	} 
	vib=0; 
	vibrSeq=0;
	vibrVolts=0; 
	for (i=0;i<NUMSTIMULATORS;i++) {  /// Set all vibrators to 0 
		stimulation[i]=0;
	} 
	for (i=0;i<NUMFINGERS;i++) { 
		off(i); 
	}  
	return true; 
}

// -----------------------------------------------------------
// StimulatorBox::readSequence for vibrator stimulation 
// 
// -----------------------------------------------------------
void StimulatorBox::readSeq(string filename,int numseq){				/// read stimultion series 
	ifstream inputFile(filename.c_str(),ios::in);
	int i=0; 
	if (numseq<0 || numseq>=MAXNUMSEQ) { 
		cout<<"StimulatorBox::readSeq: number seqeuence not in range"<<endl;
		exit(-1); 
	} 
	if (inputFile.fail()) { //Ali: if(inputFile ==0) -> if(inputFile.fail())
		cout<<"Couldn't open file: " <<filename<<endl;
		exit(-1);
	} else{
		// ignore headers in file
		while (!(inputFile.bad() || inputFile.eof() || i>=NUMSAMPLESEQ)) { 
			inputFile>>stimseq[numseq][i];
			cout <<stimseq[numseq][i]<<endl;
			i=i+1; 		
		} 
		lengthSeq[numseq]=i-1; 
	}
	cout<< lengthSeq[numseq] << " sample read"<<endl; 
} 


// -----------------------------------------------------------
// StimulatorBox::update: 
// 
// -----------------------------------------------------------
void StimulatorBox::update(){
	// update the force input 
	int i;
	for (i=0;i<NUMFINGERS;i++) { 
		volts[i]=s626.getAD(ADchannel[i],board);
		force[i]=(volts[i]-baseline[i])*scale[i]; 
		forcefilt[i]=filterconst*forcefilt[i]+(1-filterconst)*force[i]; 
	} 

	// update vibratory output 
	if (boardOn) { 
		if (vibrSeq<0 || vibrSeq>=MAXNUMSEQ) { 
			cout<<"StimulatorBox::update: stim sequence "  <<vibrSeq <<"not in range"<<endl;
			return; 
		} 
		vib++;
		if (vib>=lengthSeq[vibrSeq]) { 
			vib=0;
		} 
		s626.outDA(DAchannel,vibrVolts*stimseq[vibrSeq][vib],0); 
	} 

} 


// -----------------------------------------------------------
// StimulatorBox::on/off 
// -----------------------------------------------------------
void StimulatorBox::on(int finger){
	// update the force input 
	// cout<<"Finger on: " << DIOoffset+finger << endl;
	s626.outDIO(DIOoffset+finger,0); 
	stimulation[finger]=1; 
} 


// -----------------------------------------------------------
// StimulatorBox::on/off 
// -----------------------------------------------------------
void StimulatorBox::off(int finger){
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
void StimulatorBox::zeroForce(double x[NUMFINGERS]){
	int i; 
	for (i=0;i<NUMFINGERS;i++){ 
		baseline[i]=x[i];
	} 
} 




