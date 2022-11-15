// class Cyberglove:
//
//
/// 
/// There are 22 channels that can be read from the glove
/// These are arranged for the Cyberglove class into 
/// 6 "Fingers" and 4 joints 
/// Finger 0:Thumb 0:MCP, 1:PIP 2:DIP 3:abduct   [0 1 2 3]
/// Finger 1:Index 0:MCP, 1:PIP 2:DIP            [4 5 6]
/// Finger 2:Middle 0:MCP, 1:PIP 2:DIP 3:abduct  [7 8 9 10]
/// Finger 3:Ring 0:MCP, 1:PIP 2:DIP 3:abduct    [11 12 13 14] 
/// Finger 4:Pinkie 0:MCP, 1:PIP 2:DIP 3:abduct  [15 16 17 18]
/// Finger 5:Hand 0: Arch 1: Flex 2:Abduct       [19 20 21]
/// int Cyberglove::IFinger[22]={0,0,0,0,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5}; 
/// int  Cyberglove::IJoint[22]={0,1,2,3,0,1,2,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2}; 


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
#include <vhandtk/vhtBase.h>
#include <vhandtk/vhtHumanHand.h>
#include <vtidm/client.h>
#include <vtidm/master.h>
#include "Vector2d.h"
#include "Vector3d.h"

using std::cout;
using std::cerr;
using std::cin;


// #include "ScreenMonitor.h"
#include "S626sManager.h"
#include "Timer626.h"

///////////////////////////////////////////////////////////
/// Trial: Abstract Class with functions that have to be implemented
///////////////////////////////////////////////////////////
class Cyberglove{
public:
	Cyberglove(const char* port_cons, const char* serial_cons);									///< Constructor 
	 ~Cyberglove();									///< Destructor 
	void init();								///< Initialization, conenct to COM
	void update(void);								///< Read new values from Serial port and update state of CG
	void printState(void);							///< Prints state of CG on the Text Display 
	void getValues(void);							
	bool readCalibration(string filename);			///< Read calibration file and set 
	bool writeCalibration(string filename);			///< WRite calibration file to disk 
	void calibrate();								///< Run Calibration routine 
	void calculateKinematics();						///< Calculate the kinematic model of hand 
	void drawFinger(int finger);					///< Render a Finger in 3D 
	void drawHand();								///< Rensder the whole hand in RD 
	void printAng(int col);							///< prints Angle to the text display
	void printRaw(int col);							///< Prints raw to the text display
public: 
	bool updateRaw;
	bool updateCal;
	double angle[22];								///< These are the measured angles 
	double raw[22];									///< Measure raw angles 
	static int IFinger[22]; 
	static int IJoint[22];
	static double sign[22];
	double gain[6][4]; 
	double offset[6][4]; 
	static double digitL[5][3];		///< Length of digit parts 
	static double WfingerAbd[5][4];		///< Weights for finger adbuction angle 
	// Angles for the actual display of the 
	double fingerAngle[5][3]; 
	double fingerAbd[5]; 
	Vector3D position; 
	Vector3D orientation; 

	const char * port;
	const char * serial;


public:		// should be private 

	vhtIOConn *gloveDict;
	vhtCyberGlove *glove; 
	vhtHumanHand *humanHand; 

	
};



#endif 
