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
#define NUMFINGERS 5
#define UPDATERATE 5
#define UPDATE_TEXTDISP 10
#define SCR_SCALE 3/72 //2.54/72 // cm/pixel 
///////////////////////////////////////////////////////////////
// Enumeration of Trial State 
///////////////////////////////////////////////////////////////
enum TrialState{
		WAIT_TRIAL,
		START_TRIAL,
		WAIT_TR,
		WAIT_ANNOUNCE, 
		WAIT_RESPONSE, 
		WAIT_FEEDBACK,
		WAIT_ITI,
		END_TRIAL
};


///////////////////////////////////////////////////////////////
// Define haptic state: collection of variables that define the haptic scene 
// This collection of variables is copied when making thread-safe copy for control or display
///////////////////////////////////////////////////////////////
class HapticState {
public:
	HapticState(){}
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
	GLfloat size[NUMDISPLAYLINES]; 
	bool boxOn;
};


///////////////////////////////////////////////////////////////
/// Data Record: holds a data frame for DataManager. determines what 
/// Data is recorded at 200 Hz and written to mov file  
///////////////////////////////////////////////////////////////
class DataRecord { 
public:
	DataRecord(){}
	DataRecord(int s);
	void write(ostream &out);
public:
	int state;
	int TR;
	int currentSlice;
	double timeReal;
	double time; 
	double volts_left[5]; 
	double volts_right[5]; 
}; 


///////////////////////////////////////////////////////////////
// MyBlock
///////////////////////////////////////////////////////////////
class MyBlock:public Block { 
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
class MyTrial:public Trial { 
public:
	MyTrial();
	virtual void writeHeader(ostream &out);		///< Write HEader for data file 
	virtual void read(istream &in);				///< Trial input from Target file 
	virtual void updateGraphics(int i);			///< Update Graphics window (called ~60hz) 
	virtual void updateHaptics();				///< Update Haptics (called with 1000 hz) 
	virtual void updateTextDisplay();			///< 
	virtual void copyHaptics();
	virtual void control();						// main implementation of trial
	virtual void start();						// Start the Trial
	virtual void end();
	virtual bool isFinished();					// Trial Finished ? 
	virtual void writeDat(ostream &out);		// has to be implemented 
	virtual void writeMov(ostream &out);		// Trial output to data file 
	friend  void MyBlock::giveFeedback();
private:
	TrialState state;						///< State of the Trial 
	int startTR;							///< Which TR should the trial Start? 
	int lastTrial;							///< Is that the last Trial? (important for scanning to the the last TRs)
	int startTime;							///< When should the next trail start? is independent of TR time! 
	int seqType;							///< Which sequence of finger movments has to be done? 
	int announce; 
	int feedback;							///< Give Feedback or not? 
	int complete;							///< How much time do you have to complete the seq.?
	int iti;								///< Timedelay before the next trail starts in[ms]
	int trialType;							///< Are we doing a explain, test/left-right, train, trainscan, scan Trial? No effect on programm
	int sounds;								///< Do we play the sounds for finger presses
	int board;								///< Which board are we using left= 0 right= 1
	int day;								///< Which day of training is it 1:8 ? No effect on programm
	int seqCounter;							///< Which position in the seq are we?
	int errorFlag;							///< Was there an error in the finger presses?
	int lateFlag;							///< Was seq  complete and correct but time was already up?
	int incomplete;							///< Was sequence incomplete due to timethreshold but correct so fare?
	int hardpressKnown[NUMFINGERS];			///< Do we already meassure a hard press on that fingure?
	int hardPress;							///< Was the force to high for the finger presses?
	int response[NUMFINGERS];				///< Which key is pressed 
	int releaseState[NUMFINGERS];			///< Was the finger released already or is it still pressed?
	int inactiveFinger;						///< How many fingers are inactive?
	int superFast;							///< Was the RT super fast? if yes set to 1 otherwise 0
	double RT[NUMFINGERS];					///< When was the finger moved (time-count starts with trial)
	double pressT[NUMFINGERS];				///< For how long was the finger pressed	
	double MT;								///< Movement time, time till finishing a finger-sequence
	double Force;							///< sum of max Froces in a sequence
	int pointState;							///< How many points did you get in a trial 0/1/-1?

	DataManager<DataRecord,15000/5> dataman;	///< For data recording for MOV file 
}; 

///////////////////////////////////////////////////////////////
/// MyExperiment: Inherited from Experiment
/// added functionality to the default behavior 
///////////////////////////////////////////////////////////////
class MyExperiment:public Experiment{ 
public: 
	MyExperiment(string tit,string code); 
	void init(); // Initializes the Experiment 
	virtual void control();							// Main control loop 
	virtual void printHeader(ostream &out){}		// Prints the dat_header to files
	virtual void onExit(void);						// Called upon exit 
	virtual bool parseCommand(string args[],int numArgs);

}; 

