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

using namespace std;

#define Pi 3.141592654
//#define FIX_SIZE 0.004
#define NUMFINGERS 5 // maximal number of fingers in the sequence 
#define UPDATERATE 5 // in ms 
#define RECORDRATE 5 // in ms 
#define UPDATE_TEXTDISP 10
#define SCR_SCALE 1.84/72 //3/72 //2.54/72 // cm/pixel 
///////////////////////////////////////////////////////////////
// Enumeration of Trial State 
///////////////////////////////////////////////////////////////
enum TrialState {
	WAIT_TRIAL,
	START_TRIAL,
	WAIT_TR,
	WAIT_PLAN, //3 
	WAIT_RESPONSE, //4 
	WAIT_FEEDBACK,//5 
	WAIT_ITI,
	END_TRIAL
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

#define NUMDISPLAYLINES 15
///////////////////////////////////////////////////////////////
// Define graphics state: collection of variables that define the graphic scene 
///////////////////////////////////////////////////////////////
class GraphicState {
public:
	GraphicState();
	void reset(void);							///< Reset all targets to invisble 
	string line[NUMDISPLAYLINES];				///< Number of lines in Feedback display 
	double lineXpos[NUMDISPLAYLINES];
	double lineYpos[NUMDISPLAYLINES];
	int lineColor[NUMDISPLAYLINES];
	GLfloat lineSize[NUMDISPLAYLINES];
	//bool boxOn;
	bool showLines;
	int showBoxes;
	int boxColor;
	bool showFeedback;
	bool showDiagnostics;
	bool showSequence;
	int digit_color[NUMFINGERS];	/// Color of each of the digits in the sequence
};


///////////////////////////////////////////////////////////////
/// Data Record: holds a data frame for DataManager. determines what 
/// Data is recorded at 200 Hz and written to mov file  
///////////////////////////////////////////////////////////////
class DataRecord {
public:
	DataRecord() {}
	DataRecord(int s, int t);
	void write(ostream& out);
public:
	int trialNum;
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
	int startTR;							///< Which TR should the trial Start? 
	int startTRReal;
	int startTimeReal;
	int feedbackTime;
	int BN;
	int QuartetType;
	double rewThresh1;						/// Recorded value of the applied reward threshold (from global)
	double rewThresh2;						/// Recorded value of the applied reward threshold (from global)
	int startTime;							///< When should the next trail start? is independent of TR time!
	int planTime;
	int execTime;
	string sequence;
	int iti;								///< Timedelay before the next trail starts in[ms]
	int response[NUMFINGERS];					///< Which key is pressed 
	bool releaseState = TRUE; //[NUMFINGERS];				///< Was the finger released already or is it still pressed?
	int unpressedFinger = 0;
	int numCorrect = 0;								///< Number of corrrect presses 
	int isError;								///< Is a error made 
	int numPoints;								///< Number of points awarded 
	//int inactiveFinger;						///< How many fingers are inactive?
	//int allPressed;							///< Counts how many fingers are already placed on the board
	double RT[NUMFINGERS] = { 0, 0, 0, 0, 0 };					///< When was the finger moved (time-count starts with trial)
	double pressed[NUMFINGERS] = { 0, 0, 0, 0, 0 };				/// pressed digit
	double MT = 0;								///< Movement time, time till finishing a finger-sequence
	//double Force;							///< sum of max Froces in a sequence

	DataManager<DataRecord, 30000 / 5> dataman;	///< For data recording for MOV file 
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
	virtual void printHeader(ostream& out) {}		// Prints the dat_header to files
	virtual void onExit(void);						// Called upon exit 
	virtual bool parseCommand(string args[], int numArgs);

};

#pragma once
