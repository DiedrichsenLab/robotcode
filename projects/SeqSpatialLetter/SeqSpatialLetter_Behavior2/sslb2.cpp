
///////////////////////////////////////////////////////////////
///
/// Sequence Spatial Letter Behavioural experiment
///////////////////////////////////////////////////////////////

#include "sslb2.h"
#include "StimulatorBox.h"
#include "Target.h"
#include <algorithm>
#include <ctime>
///////////////////////////////////////////////////////////////
/// Global variables
///////////////////////////////////////////////////////////////
S626sManager s626;		///< Hardware Manager
TextDisplay tDisp;		///< Text Display (for commands)
Screen gScreen;			///< Experiment Screen
StimulatorBox gBox[2];	///< Stimulator Box

Target gTarget(SHAPE_DISC, 1);	// Draw a white box for visual target, SKim

Timer gTimer(UPDATERATE);		///< Timer from S626 board experiments
/// Usually the timers are used in the following fashion
/// \li 0: Timer that gives you the time from the start of the last block
/// \li 1: Timer that gives you the time from the start of the current trial
/// \li 2: Flexible event timer that you can use in MyTrial::control() to stop the time since the last event.
/// \li 3: Used by screen to keep track of the screen refresh rate
/// \li 4: Time elapsed since the last control loop call
/// \li 5: Time elapsed since the last data record was done.
HapticState hs; ///< This is the haptic State as d by the interrupt set up in SeqEye_Horizon.h
///< For Thread safety this SHOULD NOT be assessed While
///< the interrupt is running. Use Thread-safe copy to
///< Get the current haptic state for graphical display
GraphicState gs;

///______________________________________________________  Neda added
///////////////////////////////////////////////////////////////
/// Experimental constants for sizes of Stimuli, etc
///////////////////////////////////////////////////////////////

#define FIX_SIZE 0.5

#define TARGET_Y 3
#define START_Y  -7
#define RT_POS START_Y+1
#define JUMP_POS  RT_POS+1

Color_t defaultCol = { 255,255,255 }; ///< set default color to grey
Color_t detectCol = { 32,32,32 };
int flashCtr = 0;
Color_t CUECOLOR = { 80,20,20 };
Color_t COLORCUECOLOR = { 0,90,40 };

double TargetDistance = TARGET_Y - START_Y;

Vector2D fixationPos;	///< position of the fixation cross
#define CALIB_NUM 9		/// present 5 stimuli for eye calibration
# define EYE_MISSING 30000
bool calib_mode = false;
int curr_calib;
int refPointNumX = 0;	// Basically, this is 0(central fixation)
int jumpPointNumX = 1;	// Basically, this is fixation jump to rightward
int refPointNumY = 0;	// Basically, this is 0(central fixation)
int jumpPointNumY = 2;	// Basically, this is fixation jump to upward
FixCross 	fixationCross;

char buffer[300];		///< String buffer
HINSTANCE gThisInst;	///< defined in Experiment.cpp Instance of Windows application
Experiment* gExp;		///< defined in Experiment.cpp Pointer to myExperiment
Trial* currentTrial;	///< defined in Experiment.cpp Pointer to current Trial
bool gKeyPressed;		///< Key pressed?
char gKey;				///< Which key?
int gNumErrors = 0;		///< How many erros did you make during a block
int gNumFingerErrors = 0;	// How many finger errors did you make during a block, SKim
int finger[5];			///< State of each finger
int gPointsBlock = 0;	/// a global variable, PointBlock
int gPointsTotal = 0;	/// a global variable, PointTotal

//float timeThresPercent = 110;	///< 110% of current median MT (previous block)
//float superThresPercent = 95;	///< 95% of current median MT (previous block)
double timeThreshold = 2000;	// unit: ms 
double superThreshold = 1000;	// unit: ms

float ERthreshold = 20;		///< Trheshold of 20% of error rate in order to lower MT thresholds

#define FEEDBACKTIME 200	// time for which the points of the trial is displayed at the end of a trial
// Neda increased feedback time so that the subject has time to blink
string FINGERSOUND[6] = { "A.wav", "C.wav", "D.wav", "E.wav", "G.wav" };
//string TASKSOUNDS[5] = { "../../util/wav/smb_kick.wav",
//"../../util/wav/smb_bump.wav",
//"../../util/wav/smb_1-up.wav",
//"../../util/wav/smb_coin.wav",
//"../../util/wav/perc2.wav" };

string TASKSOUNDS[2] = { "wav/chord.wav", "wav/smb_coin.wav"};

char TEXT[5] = { '1','2','3','4','5' };
#define CUE_SEQ 6
#define CUE_PRESS 2.3 // the Y position of the presses on the screen
#define SIZE_CUE 9    // the font size of presses
#define FIXCROSS_SIZE 1

// Force Thresholdsf
#define STARTTH 0.4	// Threshold for start
#define preTH 1		// Press threshold
#define relTH 0.5	// Release threshold
#define maxTH 4		// max threshold  
double THRESHOLD[3][5] = {
	{preTH, preTH, preTH, preTH, preTH},
	{relTH, relTH, relTH, relTH, relTH},
	{maxTH, maxTH, maxTH, maxTH, maxTH}
};
double fGain[5] = { 1.0,1.0,1.0,1.0,1.0 };  // Increased gains for index and little fingers, SKim

///////////////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the robot and run it
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	gThisInst = hThisInst;
	gExp = new MyExperiment("SeqSpatialLetter_Behavior2","sslb2","C:/data/SeqSpatialLetter_Behavior2/");
	gExp->redirectIOToConsole();

	// the white interactive window
	tDisp.init(gThisInst, 100, 0, 400, 20, 5, 2, &(::parseCommand));
	tDisp.setText("Subj:", 0, 0);

	// win1: 1920x1200, win2: 1680x1050
	gScreen.init(gThisInst, 1920, 0, 1680, 1050, &(::updateGraphics));

	gScreen.setCenter(Vector2D(0, 0)); // In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE)); // cm/pixel

	// initalize s626cards
	s626.init("c:/robotcode/calib/s626_single.txt");
	if (s626.getErrorState() == 0) {
		cout << "Hello" << endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); //1 5 // initialize at 200 Hz update rate
	}

	// initialize stimulation box
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/flatbox2_lowforce_LEFT_03-Mar-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");
	gBox[1].init(BOX_RIGHT, "c:/robotcode/calib/Flatbox1_highforce_RIGHT_31-July-2017.txt");
	gBox[0].filterconst = 0.8;
	gBox[1].filterconst = 0.8;

	gTimer.init();

	gExp->control();
	return 0;
}

//////////////////////////////////////////////////////////////////
/// MyExperiment Class: contains all the additional information on 
/// how that specific Experiment is run. Most of it is standard
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////////
MyExperiment::MyExperiment(string name, string code, string dDir) : Experiment(name, code, dDir) {
	theBlock = new MyBlock();
	theTrial = new MyTrial();
	currentTrial = theTrial;
}

////////////////////////////////////////////////////////////////////////
// MyExperiment: control
////////////////////////////////////////////////////////////////////////
void MyExperiment::control(void) {
	MSG msg;
	do {
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		theBlock->control();
		currentTrial->copyHaptics();		// Thread save copy
		if (gTimer[4] > UPDATE_TEXTDISP) {	// currently every 60ms
			currentTrial->updateTextDisplay();
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			gTimer.reset(4);
		};
		InvalidateRect(gScreen.windowHnd, NULL, TRUE);
		UpdateWindow(gScreen.windowHnd);
	} while (msg.message != WM_QUIT);
}

///////////////////////////////////////////////////////////////
// Parse additional commands
///////////////////////////////////////////////////////////////
bool MyExperiment::parseCommand(string arguments[], int numArgs) {
	int n, j, x;
	float arg[4];
	MSG msg;

	/// Print continusly state of the encodeers
	if (arguments[0] == "state") {
		tDisp.keyPressed = 0;
		tDisp.lock();

		while (!tDisp.keyPressed) {
			if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
				TranslateMessage(&msg);
				DispatchMessage(&msg);

			}
			s626.updateAD(0);
			for (x = 0; x < 2; x++) {
				gBox[x].update();

				sprintf(buffer, "Force : %2.2f %2.2f %2.2f %2.2f %2.2f", gBox[x].getForce(0),
					gBox[x].getForce(1), gBox[x].getForce(2), gBox[x].getForce(3), gBox[x].getForce(4));
			}
			tDisp.setText(buffer, 5, 0);
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10);
		}
		tDisp.unlock();
	}

	//else if (arguments[0] == "diagnostics") {
	//	if (numArgs != 2) {
	//		tDisp.print("USAGE: diagnostics on->1 off->0");
	//	}
	//	else {
	//		sscanf(arguments[1].c_str(), "%f", &arg[0]);
	//		gs.showDiagnostics = arg[0];
	//	}
	//}
	// fMRI experiment, SKim
	
	/// Print continusly state of the encodeers
	else if (arguments[0] == "zeroF") {
		tDisp.keyPressed = 0;
		tDisp.lock();
		double volts[2][5] = { {0,0,0,0,0},{0,0,0,0,0} };
		for (n = 0; n < 100; n++) {
			for (x = 0; x < 2; x++) {
				for (j = 0; j < 5; j++) {
					volts[x][j] += gBox[x].getVolts(j);
				}
			}
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10);
		}
		cout << endl;
		for (x = 0; x < 2; x++) {
			for (j = 0; j < 5; j++) {
				volts[x][j] /= 100;
				cout << volts[x][j] << "  " << endl;
			}
			gBox[x].zeroForce(volts[x]);
		}
		tDisp.unlock();
	}

	/// Flip display left-right or up-down
	else if (arguments[0] == "flip") {
		if (numArgs != 3) {
			tDisp.print("USAGE: flip horz_sign vert_sign");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			gScreen.setScale(Vector2D(SCR_SCALE * arg[0], SCR_SCALE * arg[1]));
			gScreen.setCenter(Vector2D(2 * arg[0], 2 * arg[1]));

		}
	}

	/// Show the force lines
	else if (arguments[0] == "showlines") {
		if (numArgs != 2) {
			tDisp.print("USAGE: showlines 0/1");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			gs.showLines = arg[0];
		}
	}

	/// Gain for each finger
	else if (arguments[0] == "gains") {
		if (numArgs != 6) {
			tDisp.print("USAGE: Gain fingers");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			sscanf(arguments[3].c_str(), "%f", &arg[2]);
			sscanf(arguments[4].c_str(), "%f", &arg[3]);
			sscanf(arguments[5].c_str(), "%f", &arg[4]);
			fGain[0] = arg[0]; ///< Gain Finger 1
			fGain[1] = arg[1]; ///< Gain Finger 2
			fGain[2] = arg[2]; ///< Gain Finger 3
			fGain[3] = arg[3]; ///< Gain Finger 4
			fGain[4] = arg[4]; ///< Gain Finger 5
		}
	}

	else if (arguments[0] == "thresh") {
		if (numArgs != 3) {
			tDisp.print("USAGE: thresh superThreshold Threshold");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			superThreshold = arg[0];

			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			timeThreshold = arg[1];

		}
	}

	else if (arguments[0] == "resize") {
		if (numArgs != 2) {
			tDisp.print("USAGE: resize 0|1");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			gScreen.setCenter(Vector2D(0, 0));    // In cm //0,2
			gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
		}
	}

	else {
		return false; /// Command not recognized
	}
	return true;
}

///////////////////////////////////////////////////////////////
/// onExit  
///////////////////////////////////////////////////////////////
void MyExperiment::onExit() {
	s626.stopInterrupt();
	tDisp.close();
	gScreen.close();
}

///////////////////////////////////////////////////////////////
/// Constructor
///////////////////////////////////////////////////////////////
MyBlock::MyBlock() {
	state = WAIT_BLOCK; // WAIT_BLOCK == 0
}

///////////////////////////////////////////////////////////////
/// getTrial
///////////////////////////////////////////////////////////////
Trial* MyBlock::getTrial() {
	return new MyTrial();
}

///////////////////////////////////////////////////////////////
/// Called at the start of the block: resets TR Counter
///////////////////////////////////////////////////////////////
void MyBlock::start() {
	for (int i = 0; i < NUMDISPLAYLINES; i++) { gs.line[i] = ""; }
	gs.boxOn = true;
	gNumErrors = 0;
	gNumFingerErrors = 0;
	gPointsBlock = 0;
	sprintf(buffer, "%d", gPointsBlock);
	gs.line[2] = buffer;
}

void get_q1_q3(int array[], int num_val, int& q1, int& q3) {
	// Sort the array
	sort(array, array + num_val);

	int idx_q1 = (int) num_val*0.25;
	int idx_q3 = (int) num_val*0.75;
	q1 = array[idx_q1];
	q3 = array[idx_q3];
}

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback() {
	int bn = gExp->theBlock->blockNumber;	// current block number
	int totTrials = gExp->theBlock->numTrials;
	int* validTrialIdx = new int [totTrials] {-1};
	int currentMT, currentER;
	int CountValidTrial = 0;
	MyTrial* tpnr;
	for (int i = 0; i < trialNum; i++) { //check each trial
		tpnr = (MyTrial*)trialVec.at(i);
		currentMT = (int)(tpnr->MT);
		currentER = (int)(tpnr->isError);
		if ((currentMT>0) and (currentER==0)) {
			validTrialIdx[CountValidTrial] = i;
			CountValidTrial++;
		}
		//Ptarray[i] = (int)(tpnr->point);
		//numPointsTot = numPointsTot + tpnr->numPoints;
	}
	int idx;
	int* MTarray = new int [CountValidTrial];
	for (int i = 0; i < CountValidTrial; i++) {
		idx = validTrialIdx[i];
		tpnr = (MyTrial*)trialVec.at(idx);
		MTarray[i] = (int)(tpnr->MT);
	}
	int q1 = 0, q3 = 0;
	get_q1_q3(MTarray, CountValidTrial, q1, q3);

	sprintf(buffer, "Block %d / %d complete !",bn,8);
	gs.line[0] = buffer;
	gs.lineColor[0] = 1;
	sprintf(buffer, "Points: %d / %d", gPointsBlock, 3*totTrials);
	gs.line[1] = buffer;
	gs.lineColor[1] = 1;
	double ErrorRate = (double)(totTrials-CountValidTrial)*100 / totTrials;
	sprintf(buffer, "Error Rate: %2.1f%%", ErrorRate);
	gs.line[2] = buffer;
	gs.lineColor[2] = 1;

	sprintf(buffer, "block %d (Error Rate=%2.1f%%): MT_q1=%2.0ds    MT_q3=%2.0ds", bn, ErrorRate, q1, q3);
	cout << buffer << endl;

	delete[] validTrialIdx;
	delete[] MTarray;
}

///////////////////////////////////////////////////////////////
/// My Trial class contains the main info of how a trial in this experiment is run
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////////

MyTrial::MyTrial() {
	state = WAIT_TRIAL;

	//INIT TRIAL VARIABLE
	MTLimit = 10000;
	hand = 2;			// Read right box
	isError = 0;		// init error flag
	nFingerErrors = 0;	// Number of tapping errors, SKim  
	seqCounter = 0;		// init the sequence index variable
	MT = 0;				// init total movement time, SKim edited
	RT = 0;				// Added by SKim, reaction time
	onsettime = 0;

	point = 0;

	for (int i = 0; i < MAX_PRESS; i++) { // MAX_PRESS = 5 defined in header
		response[i] = -1;	// finger response
		pressTime[i] = -1;	// initialize button press time
		releaseTime[i] = -1;	// initialize button release time
		fGiven[i] = 0;
	}
}

///////////////////////////////////////////////////////////////
// Read   // This is where the Target files are read
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file, [startTime, cueType, press 1-5, iti, PrepTime]
	(in) >> startTime >> PrepTime >> cueType;

	for (int i = 0; i < MAX_PRESS; i++) { // MAX_PRESS = 14--> read presses
		(in) >> press[i];
		cueP += to_string(press[i]);
	}
	(in) >> iti ;

	// do other job
	string zero("0");
	seqLength = cueP.find(zero);	// get seqLength
	// chunkLength = cueC.length(); // get chunkLength
	if (seqLength < 0) { seqLength = cueP.length(); }
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	out << cueType << "\t"
		<< PrepTime << "\t"
		<< startTime << "\t"
		<< startTimeReal << "\t";
	for (int i = 0; i < MAX_PRESS; i++) {
		out << press[i] << "\t";
	}

	out << iti << "\t"
		// << sounds << "\t"
		<< MT << "\t"
		<< RT << "\t"
		<< point << "\t"
		<< isError << "\t";
	for (int i = 0; i < MAX_PRESS; i++) {
		out << response[i] << "\t";
	}
	for (int i = 0; i < MAX_PRESS; i++) {
		out << pressTime[i] << "\t";
	}

	out << timeThreshold << "\t"
		<< superThreshold << "\t" << endl;
		/*<< fGain[0] << "\t"
		<< fGain[1] << "\t"
		<< fGain[2] << "\t"
		<< fGain[3] << "\t"
		<< fGain[4] << "\t" << endl;*/
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) { // save the header only when BN==1
	char header[200];
	out << "cueType" << "\t"
		//<< "Horizon" << "\t"
		<< "PrepTime" << "\t"
		<< "startTime" << "\t" //repeat of target file: TIME BEGINNING FOR EACH TRIAL SINCE T=0 (1st TTL)
		<< "startTimeReal" << "\t"; //actual time of the beginning of each trial since T=0

	for (int i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "press%d", i);
		out << header << "\t";
	}
	out << "iti" << "\t"
		// << "sounds" << "\t"
		<< "MT" << "\t"
		<< "RT" << "\t"   // added by SKim
		<< "point" << "\t"
		<< "isError" << "\t";
	for (int i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "response%d", i);
		out << header << "\t";
	}
	for (int i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "pressTime%d", i);
		out << header << "\t";
	}

	out << "timeThreshold" << "\t"
		<< "timeThresholdSuper" << "\t" << endl;
		/*<< "Gain1" << "\t"
		<< "Gain2" << "\t"
		<< "Gain3" << "\t"
		<< "Gain4" << "\t"
		<< "Gain5" << "\t" << endl;*/
}

///////////////////////////////////////////////////////////////
// Save: Save movement data
///////////////////////////////////////////////////////////////
void MyTrial::writeMov(ostream& out) {
	dataman.save(out);
}

///////////////////////////////////////////////////////////////
// Start Trial
///////////////////////////////////////////////////////////////
void MyTrial::start() {
	dataman.clear();
	state = START_TRIAL;
}

///////////////////////////////////////////////////////////////
// End the trial
///////////////////////////////////////////////////////////////
void MyTrial::end() {
	state = END_TRIAL;
	dataman.stopRecording();
	gs.reset();
}

///////////////////////////////////////////////////////////////
// isFinished
///////////////////////////////////////////////////////////////
bool MyTrial::isFinished() {
	return(state == END_TRIAL ? TRUE : FALSE);
}


///////////////////////////////////////////////////////////////
// copyHaptics: makes a thread safe copy of haptic state
///////////////////////////////////////////////////////////////
void MyTrial::copyHaptics() {
	S626_InterruptEnable(0, false);
	S626_InterruptEnable(0, true);
}

///////////////////////////////////////////////////////////////
/// updateTextDisp: called from TextDisplay
///////////////////////////////////////////////////////////////
void MyTrial::updateTextDisplay() {
	//sprintf(buffer,"Force:    %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[hand-1].getForce(0),gBox[hand-1].getForce(1),gBox[hand-1].getForce(2),gBox[hand-1].getForce(3),gBox[hand-1].getForce(4));
	//tDisp.setText(buffer,2,0);

	sprintf(buffer, "Block: %d    Trial: %d/%d    Error: %d    State : %d", gExp->theBlock->blockNumber, gExp->theBlock->trialNum + 1, gExp->theBlock->numTrials, gNumErrors, state);
	tDisp.setText(buffer, 2, 0);

	sprintf(buffer, "SuperThresh: %2.0f Thresh : %2.0f ", superThreshold, timeThreshold);
	tDisp.setText(buffer, 3, 0);

	sprintf(buffer, "Press:  %d %d %d %d %d", finger[0], finger[1], finger[2], finger[3], finger[4]);
	tDisp.setText(buffer, 4, 0);

	sprintf(buffer, "sequence Counter: %d ", seqCounter);
	tDisp.setText(buffer, 5, 0);

	sprintf(buffer, "Total Points: %d", gPointsTotal);
	tDisp.setText(buffer, 8, 0);

//	sprintf(buffer, "trial : %d cueType : %d state : %d", cTrial, cueType, state);
	//tDisp.setText(buffer, 9, 0);
}

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD
///////////////////////////////////////////////////////////////
#define FINGWIDTH 1.6 // 2 -> 1.6
#define RECWIDTH 1.4
#define FORCESCALE 2
#define BASELINE -8  // SKim

void MyTrial::updateGraphics(int what) {
	int i;
	double height;
	// Finger forces
//	gScreen.printChar('+', 0, -3, SIZE_CUE);
//	fixationCross.position = gScreen.getCenter();
	fixationCross.position = Vector2D(0, -3);
	fixationCross.size = Vector2D(FIXCROSS_SIZE, FIXCROSS_SIZE);
	fixationCross.setShape(SHAPE_PLUS);

	fixationCross.setColor(gs.fixationColor);
	fixationCross.draw();

	if (gs.showLines == 1) {
		gScreen.setColor(Screen::yellow); // defines the color of force lines
		for (i = 0; i < 5; i++) {
			//reads the forces and determins how high the small bars should jump up
			height = gBox[hand - 1].getForce(i) * FORCESCALE * fGain[i] + BASELINE;	// force gauge
			height = max(BASELINE, min(height, preTH * FORCESCALE + BASELINE));		// lower limit < force < Upper limit
			//draws the smaller line for individual finger forces
			gScreen.drawLine(i * FINGWIDTH - 4.0, height, i * FINGWIDTH - 2.4, height);
		}
		gScreen.setColor(Screen::white); // defines the color of base lines
		gScreen.drawLine(-4, BASELINE, 4, BASELINE); // the lower line
		gScreen.drawLine(-4, preTH * FORCESCALE + BASELINE, 4, preTH * FORCESCALE + BASELINE);
		gScreen.drawLine(-4, relTH * FORCESCALE + BASELINE, 4, relTH * FORCESCALE + BASELINE); // changed by SKim
		// The upper line set at force threshold
	}
	// Other letters
	gScreen.setColor(Screen::white);
	for (i = 0; i < NUMDISPLAYLINES; i++) {
		if (!gs.line[i].empty()) {
			gScreen.setColor(gs.lineColor[i]);
			gScreen.print(gs.line[i].c_str(), gs.lineXpos[i], gs.lineYpos[i], gs.size[i] * 1);
		}
	}
	
	if (state == WAIT_GOCUE || state == WAIT_PRESS) {
			// Draw horizon SKim
			gScreen.setColor(1);

			if (cueType == 1) {  // Visual, vertical
				//gHorizon.position = Vector2D(0, 3.5 + Horizon);
				//gHorizon.size = Vector2D(10, 16 - 2 * Horizon);
				//gHorizon.draw();
				gScreen.drawLine(-4, -1, -4, 8);
				gScreen.drawLine(-2.4, -1, -2.4, 8);
				gScreen.drawLine(-0.8, -1, -0.8, 8);
				gScreen.drawLine(0.8, -1, 0.8, 8);
				gScreen.drawLine(2.4, -1, 2.4, 8);
				gScreen.drawLine(4, -1, 4, 8);

				for (i = 0; i < seqLength - seqCounter; i++) {
					if (gs.cuePress[i] > 0) {
						double xPos = gs.cuePress[i + seqCounter] - '1';
						gTarget.position = Vector2D(-3.2 + 1.6 * xPos, -0.0 + i * 1.6);
						gTarget.size = Vector2D(1.2, 1.2);
						gTarget.draw();
					}
				}
			}

			else if (cueType == 0) { // Vertical, Numbers
				//gHorizon.position = Vector2D(0, 3.5 + Horizon);
				//gHorizon.size = Vector2D(10, 16 - 2 * Horizon);
				//gHorizon.draw();
				for (i = 0; i < seqLength - seqCounter; i++) {  // Edited by SKim
					if (gs.cuePress[i] > 0) {
						gScreen.printChar(gs.cuePress[i + seqCounter], 0, -0.7 + i * 1.6, SIZE_CUE); // -4.7 is matched to -4.0 for visual target type
						// the number 6.5 is usually the seqLength/2 so that the sequence in centered
					}
				}
			}
	}
}

//////////////////////////////////////////////////////////////////////
/// updateHaptics: called from Hardware interrupt to allow for regular
/// update intervals cc
/// This routine is called from the  
/// Interrupt loop with UPDATERATE (usually 1000Hz or 200Hz).
/// Here you update all your devices and later make sure that
/// control and record are called with the right frequency.
/// IMPORTANT: This function is Time-critical. So never try to save data,
/// draw something on the screen, etc here. If you do the UPDATERATE will drop
/// immediately.
//////////////////////////////////////////////////////////////////////
void MyTrial::updateHaptics() {
	/// Update clocks and manipulandum
	gTimer.countup();
	gTimer.countupReal();
	s626.updateAD(0);
	gBox[0].update();
	gBox[1].update();
	//gBox[1].update();
	/// Call the Trial for control
	currentTrial->control();

	/// record the data at record frequency
	if (dataman.isRecording() && gTimer[3] >= RECORDRATE) {
		gTimer.reset(3);
		bool x = dataman.record(DataRecord(state));
		if (!x) {
			dataman.stopRecording();
		}
	}
}

//////////////////////////////////////////////////////////////////////
// Control Trial: A state-driven routine to guide through the process of a trial
// This loop is updated every 5 ms --> short latency loop
//////////////////////////////////////////////////////////////////////
void MyTrial::control() {
	int i, f, channel, j;
	Vector2D recSize;
	Vector2D recPos;

	// check fingers
	double force;
	int numNewpress = 0;	// is there a new press?
	int pressedFinger = 0;
	int released = 0;		// Number of buttons released from pressure

	for (f = 0; f < 5; f++) {
		force = gBox[hand - 1].getForce(f) * fGain[f];
		if (finger[f] == 0 && force > THRESHOLD[0][f]) { // Press threshold comparison
			numNewpress++;
			pressedFinger = f + 1;
			finger[f] = 1;
		}
	}

	for (f = 0; f < 5; f++) {
		force = gBox[hand - 1].getForce(f) * fGain[f];
		if (force < THRESHOLD[1][f]) { // Release threshold comparison
			finger[f] = 0;
			released++;
		}
	}
	// All the colors are pre-defined in the Screen class
	//0Black, 1white, 2red, 3green, 4blue, ....

	// state: 
	// WAIT_TRIAL (0) -> START_TRIAL (1) -> START_FIX (2) -> WAIT_GOCUE (3) -> WAIT_PRESS (4) ->
	// WAIT_END_RELEASE (5) -> WAIT_FEEDBACK (6) -> WAIT_ITI (7) -> END_TRIAL (8)
	switch (state) {
	case WAIT_TRIAL: // this state is before you enter the "run X *.tgt" command
		gs.clearCues();
		gTimer.reset(0);	// A timer for whole block

		break;

	case START_TRIAL: // This state is right after you've entered
		for (i = 0; i < NUMDISPLAYLINES; i++) {
			gs.line[i] = "";
		} // clear screen

		//dataman.startRecording(); // see around line #660
		gTimer.reset(1);	// A timer for whole trial
		gTimer.reset(2);	// A timer for each event in the trial			
		//onsettime = gTimer[0]; // time of the beginning of each trial since T=0

		if (gTimer[0] >= 5000) { // ready to run the task
			state = START_FIX;
		}

		break;

	case START_FIX:
		// check timeout
		if (released == 5) {
			startTimeReal = gTimer[0];

			dataman.startRecording();
			gs.clearCues();
			for (i = 0; i < seqLength; i++) {
				gs.cuePress[i] = cueP.at(i);
			}
			gTimer.reset(2);
			state = WAIT_GOCUE;
		}

		break;

	case WAIT_GOCUE:
		if (gTimer[2] > PrepTime) { // Wait for PrepTime, preplanning
			gs.fixationColor = 3;
			gTimer.reset(2);
			state = WAIT_PRESS;
		}
		break;

	case WAIT_PRESS: //Targets are shown here for preplanning
		if (gTimer[2] <= MTLimit) {
			if (numNewpress > 0 && seqCounter < seqLength) {
				response[seqCounter] = pressedFinger;
				pressTime[seqCounter] = gTimer[1];	// initially, pressTime[i] = -1. However if pressed lately, error could occur like a negative MT.
				//pressTime[seqCounter] = gTimer[0] - onsettime;	// time since the trial onset
				if (seqCounter == 0) {
					RT = gTimer[2];  // Reaction time for the first press
				}
				if (response[seqCounter] == press[seqCounter]) { // if press is correct
					// PLAY SOUND
					// channel = Mix_PlayChannel(-1, wavTask[0], 0); // SDL
					//PlaySound("wav/chimes.wav", NULL, SND_ASYNC | SND_FILENAME);
				}
				else if (response[seqCounter] != press[seqCounter]) { // press is wrong
					isError = 1;
					// PLAY SOUND
					// PlaySound("wav/chord.wav", NULL, SND_ASYNC | SND_FILENAME);
					nFingerErrors++;
					// channel = Mix_PlayChannel(-1, wavTask[1], 0); // SDL
				}
				seqCounter++;
			}
			if ((seqCounter == seqLength) and (released==5)) {
				MT = gTimer[2] - RT;
				gTimer.reset(2);
				gs.fixationColor = 1;
				state = WAIT_END_RELEASE;
			}
		}
		else {
			MT = gTimer[2] - RT;
			gTimer.reset(2);
			gs.fixationColor = 1;
			state = WAIT_END_RELEASE;
		}

		break;

	case WAIT_END_RELEASE:
		if (isError > 0) {
			point = -1;
			gNumErrors++;
			gNumFingerErrors += nFingerErrors;
		}
		else if (MT > 0) {	// with isError==0
			if (MT <= superThreshold) { // 0 < MT <= timeThreshSuper
				point = 3;
				// PLAY SOUNDS
				// channel = Mix_PlayChannel(-1, wavTask[2], 0); // SDL
			}
			else if (MT <= timeThreshold) { // timeThreshSuper < MT <= timeThresh
				point = 1;
			}
			//if (RT >= 500) { // Do not extend the PrepTime!
			//	point = max(0, point-2);
			//}
		}
		gPointsBlock += point;
		gPointsTotal += point;
			
		gs.clearCues();
		gTimer.reset(2);

		state = WAIT_FEEDBACK;

		break;

	case WAIT_FEEDBACK: 
		if (gTimer[2] > FEEDBACKTIME) {
			gs.clearCues();
			if (point > 0) {
				sprintf(buffer, "+%d", point);
				gs.lineColor[2] = 3;	// Green
			}
			else if (point == 0) {
				sprintf(buffer, "%d", point);
				gs.lineColor[2] = 1;	// White
			}
			else {
				sprintf(buffer, "%d", point);
				gs.lineColor[2] = 2;	// Red
			}
			gs.line[2] = buffer;	// displays the reward

			state = WAIT_ITI;
		}

	case WAIT_ITI:  //
		if (gTimer[2] > iti) {
			dataman.stopRecording();
			state = END_TRIAL;
		}

		break;
	case END_TRIAL: //10 as apears in mov

		break;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/// Data Record: creator records the current data from the device
/////////////////////////////////////////////////////////////////////////////////////
DataRecord::DataRecord(int s) {
	int i;
	state = s;							//culumn 1 of the .mov file
	time = gTimer[1];					//culumn 2 of the .mov file
	timeReal = gTimer.getRealtime();	//culumn 3 of the .mov file

	for (i = 0; i < 5; i++) {
		//force_left[i] = gBox[0].getForce(i);	//culumn 4-8 of the .mov file
		force_right[i] = gBox[1].getForce(i);
	}

}


/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream& out) {
	out << state << "\t" << timeReal << "\t" << time << "\t" 
		<< force_right[0] << " \t" << force_right[1] << "\t" << force_right[2] << " \t" << force_right[3] << "\t" << force_right[4] << " \t"
		<< endl;
}

/////////////////////////////////////////////////////////////////////////////////////
/// Graphic State
/// Collection of current variables relating to what's on the screen
/// contains 4 lines for display
///
/////////////////////////////////////////////////////////////////////////////////////

GraphicState::GraphicState() {
	int i;

	// points in block
	lineXpos[0] = 0;
	lineYpos[0] = 6; // feedback
	lineColor[0] = 1; // white
	size[0] = 5;

	// MT
	lineXpos[1] = 0;
	lineYpos[1] = 5; // feedback
	lineColor[1] = 1; // white
	size[1] = 5;

	// total points
	lineXpos[2] = 0;
	lineYpos[2] = 4; // block points
	lineColor[2] = 1; // white
	size[2] = 5;

	// Sequence Cue
	lineXpos[3] = 0;
	lineYpos[3] = CUE_SEQ; // 6 block points
	lineColor[3] = 1; // white
	size[3] = 9;
	// Commented by SKim
	//// Chunk Cue
	//for (i = 0; i < 5; i++) {//(i=0;i<3;i++){
	// lineXpos[i + 4] = 0;//i*1.4-1.4;
	// lineYpos[i + 4] = CUE_CHUNK;//4.5;
	// lineColor[i + 4] = 1; // white
	// size[i + 4] = 7;  // font size
	//}

	// Press Cue
	for (i = 0; i < 9; i++) {//(i=0;i<3;i++){ // edited by SK
		lineXpos[i + 8] = 0;//i*1.4-1.4;
		lineYpos[i + 8] = CUE_PRESS;//2.3;
		lineColor[i + 8] = 1; // white
		size[i + 8] = 7;  // font size
	}

	fixationColor = 1; 

	clearCues();


	boxOn = false;
	showLines = true;
}

void GraphicState::clearCues(void) {
	int i;

	for (i = 0; i < 5; i++) { // seqLength
		cuePress[i] = 0;
	}
}

void GraphicState::reset(void) {
	for (int i = 0; i < NUMDISPLAYLINES; i++) {
		line[i] = "";
	}
}

