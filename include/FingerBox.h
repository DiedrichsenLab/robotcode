////////////////////////////////////////////////////////////////////////
// Wrapper class for 5 finger keyboard with or without pneumatic action
// 
// a-yokoi (2015 Nov)
////////////////////////////////////////////////////////////////////////
#ifndef FINGERBOX_H
#define FINGERBOX_H

#define NUMFINGERS 5
#define NUMSTIMULATORS 5  
#define NUMSAMPLESEQ 50
#define MAXNUMSEQ 3  
#define BOX_LEFT 0 
#define BOX_RIGHT 1 
#include <stdio.h>
#include <string>
#include "StimulatorBox.h"
#include "PneumaticBox.h"
using namespace std;

class FingerBox {
public:
	FingerBox(); 												///< Default constructor
	~FingerBox(); 												///< Destructor
	bool init(); 												///< Initialize with specification of box type
	bool init(int type);										///< Initialize on the channel 
	bool init(int type, string filename);						///< Initialize on the channel with calibration file 
	bool init(int type, int box, string filename); 				///< Initialize with specification of box type
	void readSeq(string filename,int seq);						///< read stimulation sequence 
	void setScale(double s);									///< Set all to same scale and baseline 
	void setScale(double s0, double s1 ,
				double s2 , double s3 , double s4);				///< Set individual scale and baseline
	void setFilterConst(double filterconst); 					///< Set filterconst
	void update();												///< Read forces from Button box 
	void zeroForce(double volts[NUMFINGERS]);					///< Set baseline to the last 100 readings 
	void setVolts(double v0,double v1,
				double v2,double v3,double v4); 				///< Sets voltages of valves directly, send voltage command to finger 
	double getForce(int i);			///< 
	double getForceFilt(int i); ///< 
	double getVolts(int i) ;
	inline string getCalibFileName(int i){return CalibFileName[i];} ///< 
	double vibrVolts;											///< Vibration volts to 
	int vibrSeq;												///<
	int boardOn;									
	///< is board active? 
	int stimulation[NUMSTIMULATORS];							///< Stimulation on or off? 
	void on(int finger);										///< set finger on 
	void off(int finger);										///< set finger of 

private:
	string LRname[2];
	string Amplifiers[2];
	string Keyboards[3];
	string CalibFileName[2];
	int box; 													///< Type of box (0: stimulatorBox, 1: pneumaticBox)
	StimulatorBox *sBox; 										///< 
	PneumaticBox *pBox;	 										///< 
};
#endif