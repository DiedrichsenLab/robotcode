//////////////////////////////////////////////////////////////
// EyeTracker.h
// Simple class that takes care of the connection with the 
// eye tracker (SR Research EyeLink1000)
// 
//   2011, Alex Reichenbach 
//   a.reichenbach@ucl.ac.uk
//
//   2014, Peter Zatka-Haas: Added pupil size measurement
//	 p.zatka@ucl.ac.uk
//
//	 2016, Atsushi Yokoi: Added isEyeMissing flag and overloaded constructor
//	 at.yokoi.work@gmail.com
//////////////////////////////////////////////////////////////

#ifndef EyeTracker_H_
#define EyeTracker_H_

#include <math.h>
#include <time.h>
#include "Screen.h" 
#include "Vector2D.h"
#include <GL/gl.h>
#include <GL/glut.h>
#include <iostream>
#include <fstream>
#include <string> 
#include <sdl_expt.h>

extern Screen gScreen;
using namespace std; 

#ifndef PI
#define PI 3.14159265358979323846264338327950288419716939937510
#endif

/////////////////////////////////////////////////////////////////////////////
/// \brief Class for handling eye tracker data (SR Research EyeLink 1000). 
/// Very simple class, just gets the raw eye position data + timestamp.
/// Uses the EyeLink SDK from SR Research.
/// Can be extended for calibration of the eye tracker etc (see programmers guide on the server).
///
/// The constructor opens the connection to the eye tracker and sets offline mode 
/// (= data is tracked on the EyeLink server and can be fetched from this program).
/// The destructor closes the connection.
/// startTracking and stopTracking start/stop data tracking on the EyeLink PC.
/// getData gets a new data sample and extracts x&y position and timestamp.
/// This data can then be obtained with getXPos, getYPos, and getTimestamp.
/// The EyeLink directory must be linked.
/// \author Alex Reichenbach
/// 
/////////////////////////////////////////////////////////////////////////////

class EyeTracker{
public: 
	EyeTracker(int connection);			///< constructor, opens the connection to the eye tracker 
							///< param: connection=0 -> real connection; connection=1 -> simulation mode for debugging 
	EyeTracker(int connection,char *trackerIP, int Eye_used);			///< constructor, opens the connection to the eye tracker 
							///< param: connection=0 -> real connection; connection=1 -> simulation mode for debugging 
	~EyeTracker();			///< destructor, closes the connection to the eye tracker
	bool connectionOk();	///< checks whether we have a valid object=connection
	void startTracking();	///< start tracking on the Eyelink PC
	void stopTracking();	///< stops tracking on the Eyelink PC
	bool currTracking();	///< checks whether data is currently tracked
	bool getData();			///< fetches new data from the EyeLink PC, returns whether successful
	int getTimestamp();		///< gets the timestamp of the last data set
	float getXPos();		///< gets the raw x position of the last data set
	float getYPos();		///< gets the raw y position of the last data set
	float getXPosH();		///< gets the raw x position of the last data set
	float getYPosH();		///< gets the raw y position of the last data set
	float getXPosP();		///< gets the raw x position of the last data set
	float getYPosP();		///< gets the raw y position of the last data set		
	float getPupilDiam();   ///< gets the pupil size/area of the last data set
	bool  isEyeMissing();	///< gets flag whether eye is missing
public:
	char *trackerip;
	bool isValid;
	bool isTracking;
	bool isMissing;
	int timeStamp;
	float xPos, yPos, xPosH, yPosH, xPosP, yPosP, PupilDiam;

private:
	int eye_used;
}; 

#endif