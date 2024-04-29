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
//#include "SDL.h" // SDL library
//#include "SDL_mixer.h" // Necessary for playing multiple sounds (added by a-yokoi)

using namespace std;

#define Pi 3.141592654
#define NUMFINGERS 5
#define UPDATERATE 5
#define RECORDRATE 5
#define UPDATE_TEXTDISP 60
#define SCR_SCALE 1.84/72 //3/72 //2.54/72 // cm/pixel 
#define MAX_PRESS 11 // max number of finger presses in a sequence
///////////////////////////////////////////////////////////////
// Enumeration of Trial State 
///////////////////////////////////////////////////////////////
enum TrialState {
	WAIT_TRIAL,			// 0
	START_TRIAL,		// 1
	WAIT_TR,			// 2
	WAIT_PREP,			// 3
	WAIT_PRESS,			// 4
	WAIT_RELEASE,		// 5
	WAIT_FEEDBACK,		// 6 
	WAIT_ITI,			// 7
	END_TRIAL			// 8
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

#define NUMDISPLAYLINES 100 //20
///////////////////////////////////////////////////////////////
// Define graphics state: collection of variables that define the graphic scene 
///////////////////////////////////////////////////////////////
class GraphicState {
public:
	GraphicState();
	void reset(void);							///< Reset all display lines 
	void clearCues(void);						///< Clear all the cues on the screen 
	string line[NUMDISPLAYLINES];				///< Number of lines in Feedback display 
	double lineXpos[NUMDISPLAYLINES];
	double lineYpos[NUMDISPLAYLINES];
	int lineColor[NUMDISPLAYLINES];
	int boxColor;
	GLfloat size[NUMDISPLAYLINES];
	bool boxOn;
	bool showlines;
	char seq[MAX_PRESS];
	char seqMask[MAX_PRESS];
	char cross;
};

///////////////////////////////////////////////////////////////
/// Data Record: holds a data frame for DataManager. determines what 
/// Data is recorded at 200 Hz and written to mov file  
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
	int subNum;									///< Which subject number 
	bool isTrain;
	int planTime;
	int execTime;
	int iti;
	int precueTime;
	string windowSize;
	int digitChangePos;
	char digitChangeValue;
	int press[MAX_PRESS];					///< Which digit to press (in intrinsic coordinates 1:thumb 5: pinkie) 
	int load;                               ///< How many keypresses to plan? (from 1 to 5)
	int show;                               ///< Do you show hand information at ecoding (1) or at go cue (2)?
	int hand;								///< Which board are we using left= 1 right= 2
	int group;								/// Which group the subject blongs to
	int seqCounter;							///< Which position in the seq are we?
	int maskCounter;
	int chunkIndex;
	int fixed_dur;							///< Is the trial duration fixed or not?

	int newPress;							///< Is this a new press?
	int newRelease;							///todo: Should I add this? Ask Jorn
	int pressedFinger;						///< Which finger was pressed
	int pressedHand;                        ///< Which hand was pressed
	int released;							///< Are all fingers released?

	int numNewThresCross;					///< Has the pre-movement threshold been crossed?
	int crossedFinger;						///< Which finger crossed the threshold?
	int belowThres;							///< Are all fingers below pre-mov threshold?


	int isExtrinsic;
	int isIntrinsic;
	int isRepetition;
	int handTrans;

	int complete;
	int isError;							///< Was there an error in the trial?
	int coord;                              ///< 1: Extrinsic coordinate -- 2: intrinsic coordinate
	int cueType;                            ///< 1: numerical - 2: alphabetical
	int isCross;							///< Was there a thres cross in the trial?
	int numCrosses;							///< How many crosses in the trial?
	int timeStamp;							///< When was the pre-movement threshold crossed?

	int timingError;						///< Was there an error in the timing of finger presses? (e.g. anticipation)
	int response[MAX_PRESS];				///< Which digit is pressed (in intrinsic coordinates 1:thumb 5: pinkie) 
	int handPressed[MAX_PRESS];             ///< Which hand is pressed
	int points;							///< How many points did you get in a trial -1/0/+1/+3?
	int seqLength;							///< How long is the sequence (arbitrary)?
	int startTime;							///< Requested start time for trial in ms
	int startTimeReal;						///< Actual start time for trial (as recorded) in ms
	int startTR;							///< Requested start time for trial in TRs
	int startTRtime;						///< Actual start time for trial (as recorded) in TRs	
	int useMetronome;						///< Use timer metronome? 1=yes; 0=no
	int trialDur;							///< How long was this trial? (in ms)
	double stimOnsetTime;					///< Is it the last trial of the run? 1=yes; 0=no
	double waitTime;						///< How long to wait before the first trial? (for behavioral version only)	
	double pressTime[MAX_PRESS];			///< Time when each finger was pressed
	double releaseTime[MAX_PRESS];			///< Time when each finger was released
	double RT;								///< Reaction time (from go cue)
	double ET;								///< Execution time (from first press)
	double norm_MT;							///< Movement time (RT + MT)
	string seq; 						///< Visual cues for sequences
	string seqMask;							///< Mask for visual cues
	DataManager<DataRecord, 30000 / 2> dataman;///< For data recording for MOV file 
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

