// -------------------------------------------------------------------------
// StimulatorBox Class 
// fMRI compatible button force box with stimulation devices 
// under a s626 iocard 
// 
// --------------------------------------------------------------------------

#ifndef STIMULATORBOX_H
#define STIMULATORBOX_H 

#define NUMFINGERS 5
#define NUMSTIMULATORS 5  
#define NUMSAMPLESEQ 50
#define MAXNUMSEQ 3  
#define BOX_LEFT 0 
#define BOX_RIGHT 1 
#include <stdio.h>
#include <string> 
using namespace std;

class StimulatorBox { 
public: 
	StimulatorBox();	///< Default based on average g to volts  scale based on calibration of 17/6/2010 JD
	~StimulatorBox(); 
	bool init(int type);							///< Initialize on the channel 
	bool init(int type, string filename);			///< Initialize on the channel with calibration file 
	void readSeq(string filename,int seq);			///< read stimulation sequence 
	void setScale(double s);					    ///< Set all to same scale and baseline 
	void setScale(double s0, double s1 , double s2 , double s3 , double s4);	///< Set individual scale and baseline   
	void update();									///< Read forces from Button box 
	void zeroForce(double volts[NUMFINGERS]);				///< set baseline to the last 100 readings 
	inline double getForce(int i) {return force[i];}///< 

	// Handle the error or return a default value

	inline double getForceFilt(int i) {return forcefilt[i];}///< 
	inline double getVolts(int i) {return volts[i];}///< 
	double vibrVolts;								///< Vibration volts to 
	int vibrSeq;									///<
	int boardOn;									///< is board active? 
	int stimulation[NUMSTIMULATORS];				///< Stimulation on or off? 
	void on(int finger);							///< set finger on 
	void off(int finger);							///< set finger of 
	double filterconst;

private: 
	double stimseq[MAXNUMSEQ][NUMSAMPLESEQ]; 
	double baseline[NUMFINGERS]; 
	double scale[NUMFINGERS]; 
	double volts[NUMFINGERS]; 
	double force[NUMFINGERS]; 
	double forcefilt[NUMFINGERS];   ///<  Filtered force 
	int lengthSeq[MAXNUMSEQ]; 
	int vib;						///< vibration on of state 
	int board;						///< Board number
	int ADoffset;					///< Channel number for first AD transducers 
	int ADchannel[NUMFINGERS];		///< Channel number for first AD transducers 
	int DAchannel;					///< channel number for DA transducers 
	int DIOoffset;					///< Channel number for DIO channel 
}; 

#endif 