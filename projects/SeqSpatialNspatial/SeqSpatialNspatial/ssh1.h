#pragma once
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
#define MAX_PRESS 2 // edited by Armin
///////////////////////////////////////////////////////////////
// Enumeration of Trial State 
///////////////////////////////////////////////////////////////
enum TrialState {
	WAIT_TRIAL,			// 0
	START_TRIAL,		// 1

	//____________________Neda
	START_FIX,	    	// 2 wait for eye to fixate at the begining of the seq
	//____________________end
	//WAIT_ALLRELEASE,	// -
	WAIT_GOCUE,		// 3
	WAIT_PRESS,			// 4
	WAIT_END_RELEASE,		// 5
	//WAIT_FEEDBACK,		// -
	//END_FIX,            // -      
	WAIT_ITI,			// 6
	//____________________Neda	
	END_TRIAL,			// 7 
	//CALIB        		// -
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
	char cuePress[14]; // I'm gonna need max 3 finger presses, change this based on your task //AP added
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
	//int subNum;
	int PrepTime;
	int seqType;							///< Which sequence of finger movments has to be done? 
	int MovTimeLim;							// Added by SKim for fMRI
	int TrialTime;							// Added by SKim for fMRI, TrialTime = PrepTime + MovTime + iti
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
	int DigPressed;							///< For Horizon-wize digit revealing
	int isError;							///< Was there an error in the finger presses?
	int nFingerErrors;						// Number of error tappings, SKim
	int isComplete;							///< Are all presses made  
	int response[MAX_PRESS];				///< Which key is pressed 
	int points;								///< How many points did you get in a trial 0/1/-1?
	int seqLength;							///< How long is the sequence (arbitrary)?
	double pressTime[MAX_PRESS];			///< Time when each finger was pressed 
	double releaseTime[MAX_PRESS];			///< Time when each finger was release

	double MT;								///< Overall MT 
	double RT;								// Reaction Time, added by SKim
	string cueP;							// edited by SKim, using only press cue	
	//	string cueS, cueC, cueP; 					///< Visual cues for sequence, chunk, and press

	double nPress;					///< Time of the start of the trial 
	double startTimeReal;				///< Time of the start of the trial 

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


//////////////////////////////////////////////////////////////
class FixCross : public Target {
public:
	void draw();
};
void FixCross::draw() {
	gScreen.setColor(color);
	gScreen.drawBox(Vector2D(size[0], 0.3), Vector2D(position[0], position[1]));
	gScreen.drawBox(Vector2D(0.3, size[1]), Vector2D(position[0], position[1]));
}


// computing standard deviation
double mystd(double array[], int sizeOfArray);

// computing percentile
double myPrctile(double array[], int num_val, double percent);
///______________________________Neda end#pragma once