//////////////////////////////////////////////////////////////////////////
// GripBox.h
//
// written for a 5-finger grip device using s626 io board,
// usb servos and force trasducers.
// Carlos Hernandez, 2016
//
//////////////////////////////////////////////////////////////////////////

#include <windows.h>
#define NUMMOTORS 5
#define NUMSENSORS 10

/////////////////////////////////////////////////////////////////////////////////////
/// \brief Class for control the grip device
/// Function init is called with the name of USB port to connect and parameter file  
/// that contains the data to convert raw positions into mm. Then you only have to call   
/// update() in regular intervals in the updateHaptics() loop, and you have access to 
/// estimates of position of the five motors and force sensors. Use set target to 
/// define a new position for a specific motor.
///
/////////////////////////////////////////////////////////////////////////////////////
class GripBox {
public:
	GripBox(void);									///< Constructor
	~GripBox();										///< Destructor - Close communication
	bool  init(char *portName,char *calibfile);		///< Initializa the device
	int   readRawPos(int channel);					///< Read the raw position of specific channel			
	bool  setRawTarget(int channel,int target);		///< Set new raw position
	inline double readPosition(int channel)			///< Read position in mm of specific channel
		{return position[channel];}
	bool  update(void);								///< Reads out all motors to position  
	bool  setTarget(int channel,double position);	///< Set calibrated target in mm 
	bool  setTarget(double position[]);				///< Set calibrated vector of targets in mm 
	bool  isConnected();							///< Check if the device is connected
	double getVolts(int channel); 
	//{return volts[channel];}
	double getForce(int channel); 
	//{return force[channel];}						///< Get force of each transducer 
	void zeroForce(double volts[NUMSENSORS]);		///< Set baseline to the last 100 readings

private:
	HANDLE port;									///< USB Comm handler
	int	   rawPosition[NUMMOTORS];					///< Position in counter ticks
	double position[NUMMOTORS];						///< Calibrated position in mm 
	int	   targetPos[NUMMOTORS];					///< Target position in mm
	bool   commReady;								///< Communication flag USB
	bool   connected;								///< Connection status
	double channelIdx[NUMSENSORS];					///< Index for sensor to channel mapping
	double baseline[NUMSENSORS]; 					///< Force baseline
	double scale[NUMSENSORS]; 						///< G to Volts scaling factor (based on calibration of 17/6/2010 JD)
	double force[NUMSENSORS];						///< Force values
	double volts[NUMSENSORS]; 						///< Voltages from sensors
	int    ADchannel[NUMSENSORS];					///< Channel number for AD transducers 
};

