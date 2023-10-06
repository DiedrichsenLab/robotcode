// -------------------------------------------------------------------------
// PneumaticBox Class 
// fMRI compatible button force box with stimulation devices 
// under a s626 iocard 
// 
// --------------------------------------------------------------------------

#ifndef PNEUMATICBOX_H
#define PNEUMATICBOX_H 

#define NUMFINGERS 5
#define NUMSTIMULATORS 5  
#define NUMSAMPLESEQ 50
#define MAXNUMSEQ 3 
#define BOX_LEFT 0 
#define BOX_RIGHT 1 
#include <stdio.h>
#include <string> 
using namespace std;

class PneumaticBox { 
public: 
	PneumaticBox();	///< Default based on average g to volts  scale based on calibration of 17/6/2010 JD
	~PneumaticBox(); 
	bool init(int type);							///< Initialize on the channel 
	bool init(int type,string calib);							///< Initialize on the channel 
	void setScale(double scale);					///< Set scale and baseline  
	void update();									///< Read forces from Button box 
	void zeroForce(double volts[NUMFINGERS]);				///< set baseline to the last 100 readings 
	inline double getForce(int i) {return force[i];}///< 
	inline double getForceFilt(int i) {return forcefilt[i];}///< 
	inline double getVolts(int i) {return volts[i];}///< 
	void setVolts(double v0,double v1,double v2,double v3,double v4); ///< sets voltages of valves directly );						///< Send voltage command to finger 
	double filterconst;
	string NIDeviceType;

private: 
	double baseline[NUMFINGERS]; 
	double scale[NUMFINGERS]; 
	double volts[NUMFINGERS]; 
	double force[NUMFINGERS]; 
	double forcefilt[NUMFINGERS];   ///<  Filtered force 
	//double voltsOut[NUMFINGERS]; 
	int board;						///< Board number
	int ADoffset;					///< Channel number for first AD transducers 
	int ADchannel[NUMFINGERS];		///< Channel number for first AD transducers 
	//double DAchannel[NUMFINGERS];		///< channel number for DA transducers (negative numbers for USB device) 
	string localDeviceName;
	
}; 

#endif 