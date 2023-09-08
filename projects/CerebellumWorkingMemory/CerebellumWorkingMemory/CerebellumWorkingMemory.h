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
#include <string>
#include <sstream>
#include <math.h>
//#include "SDL.h" // SDL library
//#include "SDL_mixer.h" // Necessary for playing multiple sounds (added by a-yokoi)

using namespace std;

#define Pi 3.141592654
//#define FIX_SIZE 0.004
#define NUMFINGERS 5
#define UPDATERATE 2
#define RECORDRATE 2
#define UPDATE_TEXTDISP 60
#define SCR_SCALE 1.84/72 //3/72 //2.54/72 // cm/pixel 
#define MAX_PRESS 30 // set to 20. not optimal, but seems to be the only way to handle dat file for both tasks
/////////////////////////////////////////k//////////////////////
// Enumeration of Trial State 
///////////////////////////////////////////////////////////////
/*enum TrialState{
		WAIT_TRIAL,			// 0
		START_TRIAL,		// 1
		WAIT_TR,			// 2
		WAIT_ANNOUNCE,		// 3
		WAIT_PRESS,			// 4
		WAIT_RELEASE,		// 5
		WAIT_FEEDBACK,		// 6
		WAIT_ITI,			// 7
		PRESS_EARLY,        // 8
		END_TRIAL			// 9
};*/

enum TrialState {
	// having different states for fs and wm
	WAIT_TRIAL,			    // 0  waits for the trial to start
	START_TRIAL,		    // 1  starts the trial
	WAIT_TR,			    // 2  checks and waits for the TR (scanning
	FS_WAIT_ANNOUNCE,		// 3  announces the force speed task: initial information given to the subject
	FS_WAIT_PRESS,			// 4  wait for presses: the number of presses is not important. They can press any number of digits in the period
	FS_WAIT_RELEASE,		// 5  wait for finger release
	FS_WAIT_FEEDBACK,		// 6  shows feedback

	WAIT_ITI,				// 7  The iti

	WM_WAIT_ANNOUNCE,		// 8  anounces the WM task: initial information about recall direction and wm load are given
	WM_WAIT_DIGIT,		    // 9  shows digits sequentially
	WM_WAIT_PRESS,          // 10 waits for presses during execution phase of the wm task
	WM_WAIT_RELEASE,        // 11 waits for releases during execution phase of the wm task
	WM_WAIT_FEEDBACK,       // 12 shows feedback: does not come here after memory trials
	END_TRIAL			    // 13 Trial ends
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
#define NUMDISPLAYLINES_ROWS 30
#define NUMDISPLAYLINES_COLS 10

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
	int boxColor[2];
	GLfloat size[NUMDISPLAYLINES];

	int lineColor_table[7];
	GLfloat size_table[7];


	bool tableOn;
	bool boxOn;
	bool crossOn;
	bool showLines;
	char cueSeq[1];
	char cuePress[9];
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
	int cTrial; 							///< Trial number
	int trialType;							///< trial type: 1:FS, 2:WM:memory encoding, 3:WM:execution
	int seqType;							///< Which sequence of finger movements has to be done? 
	int seqNumb;							///< Which sequence number it is (indicator)
	int press[MAX_PRESS];					///< Which digit to press (in intrinsic coordinates 1:thumb 5: pinkie) 
	int feedback;							///< Give Feedback or not? 
	int complete;							///< How much time do you have to complete the seq.?
	int iti;								///< Timedelay before the next trail starts in[ms]
	int announceTime;
	int feedbackTime;
	int feedbackType;
	int sounds;								///< Do we play the sounds for finger presses
	int hand;								///< Which board are we using left= 0 right= 1
	int isScan;								///< Is scanning session or not 0-no, 1-yes
	int isMetronome;						///< Timer - 1 - yes, 0 - no
	int lastTrial;							///< Last trial = 1 for extended screen present in scan
	int FoSEx;								///< First or second execution of same seq in a row
	int blockType;							///< Block type - SL, CL, PSM, Scan etc.
	int ScanSess;							///< Which scanning session it is (0-4)
	int seqCounter;							///< Which position in the seq are we?
	int seqCounter2;
	int seqCounterR;
	int seqCounterRi;
	int numNewpress;						///< How many new presses made?
	int released;							///< Are all fingers released?
	int gNumErrors;
	int gNumErrors2;
	int isError;							///< Was there an error in the finger presses?
	int isComplete;							///< Are all presses made  
	int response[MAX_PRESS];				///< Which digit is pressed (in intrinsic coordinates 1:thumb 5: pinkie) 
	int points;								///< How many points did you get in a trial 0/1/-1?
	int seqLength;							///< How long is the sequence (arbitrary)?
	int chunkLength;						///< How many chunks in the sequence?
	int endTime;
	int startTime;
	int startTimeReal;
	int startTR;
	int startTRTime;
	int wmType;
	int wmLoad;
	int exec;
	//int TnumPress;	                        ///< Target number of presses: mostly used in FS experiment
	int AnumPress;                          ///< actual number of presses: mostly used in FS experiment
	double ForceL;							///< force level threshold
	double SpeedL;                          ///< trial time that determines speed 
	double SlowTime;						///< time considered slow
	double timeTrial;
	double pressTime[MAX_PRESS];			///< Time when each finger was pressed 
	//double vpressTime[MAX_PRESS];           ///< for fs Experiment: the time when the rectangle around the number changes color 
	double respForce[MAX_PRESS];			///< force excerted by the pressed finger
	double respForceTmp;
	double releaseTime[MAX_PRESS];			///< Time when each finger was release
	double MT;								///< Overall MT 
	double RT;
	double gavg;
	int Force;
	int Speed;
	double peakForcef[5]; // peak force????????????????????????
	double peakForce[MAX_PRESS]; // peak force????????????????????????
	//vector<int> resp;
	//vector<double>pressTime2;
	vector<int> PP; // this vector will contain the digits from the sequence (cueM). cueM will be passed on as an integer
	string respStr;
	string cueDm, cueD; 					///< Visual cues for sequence, chunk, and press
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

