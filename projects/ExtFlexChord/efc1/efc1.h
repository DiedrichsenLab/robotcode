#include <windows.h>
#include <conio.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "TextDisplay.h"
#include "Screen.h"
//#include "Vector2d.h"

//#include "Matrix2d.h"
#include "Target.h"
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
#define NUMFINGERS 10
#define UPDATERATE 2
#define RECORDRATE 2
#define UPDATE_TEXTDISP 10
#define SCR_SCALE 1.84/72 //3/72 //2.54/72 // cm/pixel 
#define MAX_PRESS 6 // max number of finger presses in a sequence
/////////////////////////////////////////k//////////////////////
// Enumeration of Trial State 
///////////////////////////////////////////////////////////////
enum TrialState {
	WAIT_TRIAL,			// 0
	START_TRIAL,		// 1
	WAIT_PLAN,			// 2
	WAIT_EXEC,			// 3
	GIVE_FEEDBACK,		// 4
	WAIT_ITI,			// 5
	END_TRIAL			// 6
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
	string line[NUMDISPLAYLINES];				///< Number of lines in Feedback display 
	double lineXpos[NUMDISPLAYLINES];
	double lineYpos[NUMDISPLAYLINES];
	int lineColor[NUMDISPLAYLINES];
	GLfloat size[NUMDISPLAYLINES];
	bool showBoxes;
	bool showLines;
	bool showTarget;
	bool showForces;
	bool planCue;
	bool planError;
	bool chordError;
	bool showForceBars;
	bool showFeedback;
	bool fingerCorrectGraphic[5];
	bool showTimer5;
	int boxColor;
	int rewardTrial;
	bool showDiagnostics;
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
	int trialNum;
	double timeReal;
	double time;
	double fforce[2][5];
	double diffForceMov[5];
	double visualizedForce[5];
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

	int subNum;								///< Subjet number
	int planTime;							///< Duration between cue onset (visual stimulus) and go onset
	int execMaxTime;						///< Maximum duration for execution of the chord
	int feedbackTime;						///< The duration between giveFeedback and giveScore
	int iti;								///< inter trial interval
	double stimTrigPlan;
	double stimTrigBaseline;
	bool TrigPlan;
	bool TrigBaseline;
	string session;
	int week;
	int day;
	string chordID;							///< Chord identifier. 0: neutral , 1: flexion , 2: extension
	bool trialCorr;							///< 1: trial is correct , 2: trial is not correct
	int trialErrorType;						///< 0: no error , 1: movement during planning , 2: could not execute
	double RT;								///< Reaction time: time from go cue to full execution of chord
	int trialPoint;							///< point received in each trial

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

void SetDacVoltage(WORD channel, DOUBLE volts);
void SetDIOState(WORD group, WORD states);


class ForceCursor : public Target {
public:
	void draw();
};
void ForceCursor::draw() {
	//setColor(1);
	gScreen.setColor(color);
	gScreen.drawBox(Vector2D(size[0], 0.3), Vector2D(position[0], position[1]));
}
