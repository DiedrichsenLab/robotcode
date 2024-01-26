#include <windows.h>
#include <conio.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "TextDisplay.h"
#include "Screen.h"
#include "Vector2d.h"
#include "Matrix2d.h"
#include "DataManager.h" 
#include "S626sManager.h"
#include "Experiment.h" 
#include "Timer626.h"
#include "TRCounter626.h" 
#include "Win626.h"
#include <gl/glut.h>
#include "Target.h"    // Neda added--> Simple class that implements rectangles that can change color

using namespace std;

#define Pi 3.141592654
//#define FIX_SIZE 0.004
#define NUMFINGERS 5
#define UPDATERATE 2    // sampling rate at every UPDATERATE ms 1=1K, 2=500Hz ...
#define RECORDRATE 2    // recording update rate
#define UPDATE_TEXTDISP 60
#define SCR_SCALE 1.84/72 //3/72 //2.54/72 // cm/pixel 
#define MAX_PRESS 14
///////////////////////////////////////////////////////////////
// Enumeration of Trial State 
///////////////////////////////////////////////////////////////
enum TrialState {
	WAIT_TRIAL,			// 1
	START_TRIAL,		// 2
	//____________________Neda
	START_FIX,	    	// 3 wait for eye to fixate at the begining of the seq
	//____________________end
	WAIT_ALLRELEASE,	// 4
	WAIT_ANNOUNCE,		// 5
	WAIT_PRESS,			// 6
	WAIT_END_RELEASE,		// 7
	WAIT_FEEDBACK,		// 8
	END_FIX,            // 9      
	WAIT_ITI,			// 10
	//____________________Neda	
	END_TRIAL,			// 11 wait for eye to fixate at the begining of the seq
	CALIB        		// 12 only used for calibration
	//____________________end
};


///////////////////////////////////////////////////////////////
// Define haptic state: collection of variables that define the haptic scene 
// This collection of variables is copied when making thread-safe copy for control or display
///////////////////////////////////////////////////////////////
class HapticState {
public:
	HapticState() {}
	long counter[2];	// Holds the raw values of the 2 counters 
};

#define NUMDISPLAYLINES 30
///////////////////////////////////////////////////////////////
// Define graphics state: collection of variables that define the graphic scene 
///////////////////////////////////////////////////////////////
class GraphicState {
public:
	GraphicState();
	void reset(void);							///< Reset all targets to invisble 
	string line[NUMDISPLAYLINES + 1];				///< Number of lines in Feedback display 
	double lineXpos[NUMDISPLAYLINES + 1];
	double lineYpos[NUMDISPLAYLINES + 1];
	void clearCues(void);						///< Clear all the cues on the screen 
	//Vector2D FixPlusPos[1];
	double FixPlusX;
	double FixPlusY;
	int lineColor[NUMDISPLAYLINES + 1];
	int boxColor[2];
	GLfloat size[NUMDISPLAYLINES + 1];
	bool boxOn;
	int showLines;

	char Points[2];
	char cuePress[14];
};


///////////////////////////////////////////////////////////////
/// Data Record: holds a data frame for DataManager. determines what 
/// Data is recorded at 500 Hz and written to mov file  
///////////////////////////////////////////////////////////////
class DataRecord {
public:
	DataRecord() {}
	DataRecord(int s);
	void write(ostream& out);
public:
	int state;
	double timeReal;
	double time;
	double force_left[5];
	double force_right[5];
	///_______________________Neda add

	int eyeTime;						///< eye tracker data - timestamp
	float eyeX;							///< eye tracker data - x position(easy calibration)
	float eyeY;							///< eye tracker data - y position(easy calibration)
	float eyeX_org;						///< eye tracker data - x position(original coordinate of Eyelink)
	float eyeY_org;						///< eye tracker data - y position(original coordinate of Eyelink)
	float pupil_size;
	float PPDx;
	float PPDy;
	float eyestatus;
	float xStart;
	float xEnd;
	float eventType;
	float eventTime;
	///_________________________Neda end
};


///////////////////////////////////////////////////////////////
// MyBlock
///////////////////////////////////////////////////////////////
class MyBlock :public Block {
public:
	MyBlock();
	virtual Trial* getTrial();		// create a new Trial 
	virtual void giveFeedback();
	virtual void start();
};

///////////////////////////////////////////////////////////////
/// MyTrial: contains all in information for a single trial 
/// is read in from a target file 
/// updateGraphics, updateHaptics and control determine 
/// the main behavior of a trial 
///////////////////////////////////////////////////////////////
class MyTrial :public Trial {
public:
	MyTrial();
	virtual void writeHeader(ostream& out);		///< Write HEader for data file 
	virtual void read(istream& in);				///< Trial input from Target file 
	virtual void updateGraphics(int i);			///< Update Graphics window (called ~60hz) 
	virtual void updateHaptics();				///< Update Haptics (called with 1000 hz) 
	virtual void updateTextDisplay();			///< 
	virtual void copyHaptics();
	virtual void control();						// main implementation of trial
	virtual void start();						// Start the Trial
	virtual void end();
	virtual bool isFinished();					// Trial Finished ? 
	virtual void writeDat(ostream& out);		// has to be implemented 
	virtual void writeMov(ostream& out);		// Trial output to data file 
	friend  void MyBlock::giveFeedback();
private:
	TrialState state;						///< State of the Trial 
	int cTrial; 							///< Trial number
	int Horizon;                            ///< How mnay digits ahead can you see
	int StimTimeLim;							///< For how long is the seq/chunk displayed
	int seqType;							///< Which sequence of finger movments has to be done? 
	int press[MAX_PRESS];					///< Which digit to press 
	int fGiven[MAX_PRESS];
	int feedback;							///< Give Feedback or not? 
	int complete;							///< How much time do you have to complete the seq.?
	int iti;								///< Timedelay before the next trail starts in[ms]
	int sounds;								///< Do we play the sounds for finger presses
	int hand;								///< Which board are we using left= 0 right= 1
	int seqCounter;							///< Which position in the seq are we?
	int numNewpress;
	int released;
	int tempCounter;							///< Which position in the seq are we?
	int DigPressed;							///< For Horizon-wize digit revealing
	int isError;							///< Was there an error in the finger presses?
	int isComplete;							///< Are all presses made  
	int response[MAX_PRESS];				///< Which key is pressed 
	int points;								///< How many points did you get in a trial 0/1/-1?
	int seqLength;							///< How long is the sequence (arbitrary)?
	int chunkLength;						///< How many chunks in the sequence?
	double pressTime[MAX_PRESS];			///< Time when each finger was pressed 
	double releaseTime[MAX_PRESS];			///< Time when each finger was release

	double MT;								///< Overall MT 
	string cueS, cueC, cueP; 					///< Visual cues for sequence, chunk, and press
	DataManager<DataRecord, 30000 / 2> dataman;	///< For data recording for MOV file 
};

///////////////////////////////////////////////////////////////
/// MyExperiment: Inherited from Experiment
/// added functionality to the default behavior 
///////////////////////////////////////////////////////////////
class MyExperiment :public Experiment {
public:
	MyExperiment(string tit, string code, string dDir);
	void init(); // Initializes the Experiment 
	virtual void control();							// Main control loop 
	virtual void onExit(void);						// Called upon exit 
	virtual bool parseCommand(string args[], int numArgs);

};

// computing standard deviation
double mystd(double array[], int sizeOfArray);

// computing percentile
double myPrctile(double array[], int num_val, double percent);
///______________________________Neda end#pragma once
