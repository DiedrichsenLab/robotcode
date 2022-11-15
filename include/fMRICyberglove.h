/// Class fMRICyberglove:
/// 
/// There are 14 channels that can be read from the glove
/// The sensors are arranged as follows:
/// 0 - Thumb - MCP
/// 1 - Thumb - PIP
/// 2 - Thumb/Index - Abd 
/// 3 - Index - MCP
/// 4 - Index - PIP
/// 5 - Index/Middle - Abd
/// 6 - Middle - MCP							
/// 7 - Middle - PIP 
/// 8 - Middle/Ring - Abd
/// 9 - Ring - MCP
/// 10 - Ring - PIP	
/// 11 - Ring/Pinky - Abd
/// 12 - Pinky - MCP 
/// 13 - Pinky - PIP

// ************************************************************
#if !defined DEF_CYBERGLOVE
#define DEF_CYBERGLOVE

#include <math.h>
#include <assert.h>
#include <string> 
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

// NY added following 19/01/2010
#include <vtidm/client.h>
#include <vtidm/master.h>
#include "Vector2d.h"
#include "Vector3d.h"
#include "fglove.h"

using std::cout;
using std::cerr;
using std::cin;

#include "S626sManager.h"
#include "Timer626.h"

#define NSEN 14		// number of sensors

///////////////////////////////////////////////////////////
/// fMRI Cyberglove Class
///////////////////////////////////////////////////////////
class fMRICyberglove{
public:
	fMRICyberglove();									///< Constructor 
	fMRICyberglove(int sGain);							///< Constructor
	 ~fMRICyberglove();									///< Destructor 
	void init();										///< Initialization, connect to USB
	void update(void);									///< Read new values from USB and update state of CG
	
	// Console display functions
	void printStateRaw(void);							///< Prints state of CG on the Text Display 
	void printStateScaled(void);						///< Prints state of CG on the Text Display 

	// Text display functions
	void printAng(int col);								///< prints Angle to the text display
	void printRaw(int col);								///< Prints raw to the text display
	void printScaled(int col);							///< Prints scaled to the text display
	
	// Calibration functions
	bool readCalibration(string filename);				///< Read calibration file and set 
	bool writeCalibration(string filename);				///< Write calibration file to disk 
	void calibrate();									///< Run Calibration routine 
	void getValues(void);								///< Gets calibrated values from the glove

	// Drawing functions
	void calculateKinematics();							///< Calculate the kinematic model of hand 
	void drawFinger(int finger);						///< Render a Finger in 3D 
	void drawHand();									///< Rensder the whole hand in RD 

public: 
	bool updateRaw;
	bool updateCal;
	bool updateScaled;
	
	// sensor measurements from the glove
	double raw[NSEN];										///< Measure raw angles 
	double scaled[NSEN];									///< Measure raw angles 
	double angle[NSEN];										///< These are the measured angles 
	static int SIndex[NSEN];
	
	static double digitL[5][3];		///< Length of digit parts 
	static double WfingerAbd[5][4];		///< Weights for finger adbuction angle 
	
	// Angles for the actual display of the 
	double fingerAngle[5][3]; 
	double fingerAbd[5]; 
	Vector3D position; 
	Vector3D orientation; 

	int gain;						///< Gain to be applied to the sensor outputs (for display only)
	int baseline[5];					///< base line hand flexion/extension calculated for the glove
	int grip[5];						///< the peak raw values for maximal flexion of all fingers

	fdGlove *hGlove;					///< handle for the fMRI glove
};



#endif 
