///////////////////////////////////////////////////////////////
/// SensoriMotorPrediction - Marco Emanuele , Sept 2023
///////////////////////////////////////////////////////////////
#include "smp0.h" 
#include "Vector2d.h"
#include <algorithm> 
#include <iostream>
#include <cmath>

///////////////////////////////////////////////////////////////
/// Global variables 
///////////////////////////////////////////////////////////////
S626sManager s626;				///< Hardware Manager 
TextDisplay tDisp;				///< Text Display
Screen gScreen;					///< Screen 
PneumaticBox gBox;				///< Stimulator Box
TRCounter gCounter;				///< TR Counter 
Timer gTimer(UPDATERATE);		///< Timer from S626 board experiments 
HapticState hs;					///< This is the haptic State as d by the interrupt 
GraphicState gs;				///< Graphic state
char buffer[300];				///< String buffer 
HINSTANCE gThisInst;			///< Instance of Windows application
Experiment* gExp;				///< Pointer to myExperiment 
Trial* currentTrial;			///< Pointer to current Trial 
#define DAC_VSCALAR 819.1       ///< Binary-to-volts scalar for DAC.
bool gTimerFlagFirst = 0;
bool startTriggerEMG = 0;       // Ali added this: experimental - under construction
float emgTrigVolt = 2;	        // Ali added this: experimental - under construction


///< Basic imaging parameters
#define TRTIME 1000				///< timer for simulating timer
//#define HOLDTIME 1000			///< timer for holding key press
#define MOVETHRESHOLD 0.8		///< above this force, finger detected as moving
#define RELEASETHRESHOLD 0.6	///< below this force, finger detected as released
#define FEEDBACKTIME 1000;		///< duration of n points feedback on screen
#define NOGOTIME 3000;			///< Duration if the cue is no-go
char counterSignal = '5';		///< What char is used to count the TR
//int sliceNumber = 32;			///< How many slices do we have

///< Screen graphics definitions
#define baseTHhi  1.2 //0.8//1.0			// Baseline higher threshold (to check for premature movements during sequence planning phase)
double fGain[5] = { 1.0 ,1.0,1.0,1.0,1.0 };	// finger specific force gains -> applied on each finger
double forceGain = 1;						// universal force gain -> applied on all the fingers
double mahdiyar = 123;
bool blockFeedbackFlag = 0;
bool wait_baseline_zone = 1;				// if 1, waits until the subject's fingers are all in the baseline zone. 

#define FINGWIDTH 1.3
#define N_FINGERS 5
#define FINGER_SPACING 0.2
#define BASELINE_X1 -(FINGWIDTH*N_FINGERS/2)
#define BASELINE_X2 +(FINGWIDTH*N_FINGERS/2)

#define FLX_ZONE_WIDTH 6
#define FLX_BOT_Y1 9
#define FLX_TOP_Y1 FLX_BOT_Y1+FLX_ZONE_WIDTH
#define FLX_BOT_Y2 FLX_BOT_Y1
#define FLX_TOP_Y2 FLX_TOP_Y1

#define VERT_SHIFT -5	// vertical shift of the screen graphics

///< Visualization colors
Color_t myColor[9] = {
{0,0,0},			// Black 1 - 0%
{255,255,255},		// White 2 - 100%
{0,200,0},			// Green 3
{150,0,0},			// Red 4
{127,127,127}, 		// mid gray 5 - 50%
{64, 64, 64},		// dark grey 6 - 75%
{191, 191, 191},	// light grey 7 - 25%
{100,100,100},		// baseline gray 8
{249,215,28} };		// yellow 9

///< Task specific parameters
bool gSound = true;
string TASKSOUNDS[8] = {
		"C:/robotcode/util/wav/ding.wav",		// 0
		"C:/robotcode/util/wav/smb_coin.wav",	// 1
		"C:/robotcode/util/wav/chimes.wav",		// 2
		"C:/robotcode/util/wav/smb_kick.wav",	// 3	
		"C:/robotcode/util/wav/bump.wav",		// 4
		"C:/robotcode/util/wav/chord.wav",		// 5
		"C:/robotcode/util/wav/smb_pipe.wav",	// 6
		"C:/robotcode/util/wav/error.wav"		// 7
};
int gNumCorr = 0;
int gNumWrong = 0;
char gKey;
bool gKeyPressed;
double gTargetWidth = 0.25;
double baselineCorrection = -3.0;
double gErrors[5] = { 0,0,0,0,0 };
double execAccTime = 600;
double sF[5] = { 0,0,0,0,0 };
double gVolts[5] = { 0,0,0,0,0 };
double currentForce = 0;
double maxForce[5] = { 0, 0, 0 ,0, 0 };

///////////
////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the fingerBox and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	// 1. initialization window, text display and screen
	gThisInst = hThisInst;
	gExp = new MyExperiment("smp0", "smp0", "C:/data/SensoriMotorPrediction/smp0/");
	//gExp->redirectIOToConsole();

	// gExp->redirectIOToConsole();		// I uncommented this!!!
	tDisp.init(gThisInst, 0, 0, 600, 20, 9, 2, &(::parseCommand));		// Default setting for the Windows 10 PC
	tDisp.setText("Subj", 0, 0);
	gScreen.init(gThisInst, 1920, 0, 1920, 1080, &(::updateGraphics));	// Default setting for the Windows 10 PC
	gScreen.setCenter(Vector2D(0, 0)); // This set the center of the screen where forces are calibrated with zero force // In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));					// cm/pixel

	// 2. initalize s626cards 
	s626.init("c:/robotcode/calib/s626_single.txt");
	if (s626.getErrorState() == 0) {
		cout << "Initializing S626 Card" << endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); // initialize at 200 Hz update rate 
	}
	gTimer.init(); // Ali Changed Here!!!!

	// 3. stimulation box initialization and calibration
	// high force 1
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/Flatbox1_highforce_LEFT_07-Jun-2017.txt");
	//gBox.init(BOX_RIGHT,"c:/robot/calib/Flatbox1_highforce_RIGHT_31-July-2017.txt");

	// STARK
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/Flatbox1_highforce2_LEFT_12-Feb-2022.txt");
	//gBox.init(BOX_RIGHT,"c:/robot/calib/Flatbox1_highforce2_RIGHT_03-Dec-2021.txt");

	// CHOMSKY
	//gBox[0].init(BOX_LEFT, "c:/robot/calib/LEFT_lowForce_FlatBox2_24-Jan-2018.txt");
	//gBox[1].init(BOX_RIGHT, "c:/robot/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");

	//gBox[0].init(BOX_LEFT, "c:/robotcode/calib/Flatbox1_highforce2_LEFT_12-Feb-2022.txt"); 
	//gBox.init(BOX_RIGHT, "c:/robotcode/calib/Flatbox1_highforce2_RIGHT_03-Dec-2021.txt");
	gBox.init(BOX_RIGHT, "c:/robotcode/calib/right_lowforce_pneumatic1.txt");

	// low force
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/flatbox2_lowforce_LEFT_03-Mar-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");

	//gBox[0].filterconst = 0.8;
	//gBox[1].filterconst = 0.8;

	gExp->control(); //from A goes to B

	return 0;
}

///////////////////////////////////////////////////////////////
///	MyExperiment Class: contains all the additional information on how that specific 
/// Experiment is run. Most of it is standard
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Constructor 
///////////////////////////////////////////////////////////////
MyExperiment::MyExperiment(string name, string code, string dDir) : Experiment(name, code, dDir) {
	theBlock = new MyBlock();
	//theForce = new MyForce();
	theTrial = new MyTrial();
	currentTrial = theTrial;
}

////////////////////////////////////////////////////////////////////////
// MyExperiment: control 
////////////////////////////////////////////////////////////////////////
void MyExperiment::control(void) { // here is B
	MSG msg;
	float emgTrigVolt = 0;	// Ali EMG

	//if (!gTimerFlagFirst) { // Ali Changed Here!!!!
	//	gTimer.init();
	//	gTimerFlagFirst = TRUE;
	//}

	do {
		//This function checks if there are any messages in the application's message queue. 
		//If a message is available, it's retrieved and stored in the msg variable.
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		theBlock->control();
		currentTrial->copyHaptics();		// Thread save copy 
		//cout << gTimer[4] << " ";
		if (gTimer[4] > UPDATE_TEXTDISP) { // Update text display every UPDATE_TEXTDISP ms
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
	int b, i;
	float arg[4];
	MSG msg;

	/// Zero the force of the two fingerBox
	if (arguments[0] == "zeroF") {
		tDisp.keyPressed = 0;
		tDisp.lock();
		double volts[5] = { 0,0,0,0,0 };
		int n, j;
		for (n = 0; n < 100; n++) {
			for (j = 0; j < 5; j++) {
				// Here it reads baseline values, I added baseline_correction 
				// so the calibrated values are outside the baseline area
				volts[j] += gBox.getVolts(j);
			}
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10);
		}
		cout << endl;
		for (j = 0; j < 5; j++) {
			volts[j] /= 100;
			cout << volts[j] << "  " << endl;
		}
		gBox.zeroForce(volts);
		tDisp.unlock();
	}

	// Compute Max Force of each finger
	else if (arguments[0] == "maxForce") {
		if (numArgs != 2) {
			tDisp.print("USAGE: maxForce finger->1 to 5, exit->-1)");
		}

		else {
			int finger = std::stoi(arguments[1]);
			theTrial->force(finger);
		}
	}

	//  Valves Command: set voltage channels directly 
	else if (arguments[0] == "push") {
		if (numArgs != 6) {
			tDisp.print("USAGE: push v1 v2 v3 v4 v5");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			sscanf(arguments[3].c_str(), "%f", &arg[2]);
			sscanf(arguments[4].c_str(), "%f", &arg[3]);
			//sscanf(arguments[5].c_str(), "%f", &arg[4]);
			gVolts[0] = arg[0];
			gVolts[1] = arg[1];
			gVolts[2] = arg[2];
			gVolts[3] = arg[3];
			//gVolts[4] = arg[4];
			gBox.setVolts(gVolts[0], gVolts[1], gVolts[2], gVolts[3], gVolts[4]);
		}
	}

	/// Set TR Counter to simulated or non-simulated (default is given by TRTIME in ms)
	else if (arguments[0] == "TR") {
		if (numArgs != 2) {
			tDisp.print("USAGE: TR duration [in ms]");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			if (arg[0] > 0) {
				gCounter.simulate(arg[0]);  // TR>0  -> simulate trigger (practice sessions) and define TR time (custom duration defined by input TR)
			}
			else {
				gCounter.simulate(0);       // TR<=0 -> wait for trigger from scanner (scanning sessions)
			}
		}
	}

	/// Show the force lines 
	else if (arguments[0] == "showlines") {
		if (numArgs != 2) {
			tDisp.print("USAGE: showlines 0/1");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			if (arg[0] > 0) {
				gs.showLines = true;
			}
			else {
				gs.showLines = false;
			}
		}
	}

	/// play sound
	//add to target whether sound on or off?
	else if (arguments[0] == "playsound") {
		if (numArgs != 2) {
			tDisp.print("USAGE: sound on->1 off->0");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			gSound = arg[0];
		}
	}

	/// diagnostics on or off
	// set whether diagnostics are on or off - refer to gs.showDiagnostis in MyTrial::updateGraphics
	else if (arguments[0] == "diagnostics") {
		if (numArgs != 2) {
			tDisp.print("USAGE: diagnostics on->1 off->0");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			gs.showDiagnostics = arg[0];
		}
	}

	/// set force gain. You can set any arbitrary force gain for every participant if they cant do the chord.
	else if (arguments[0] == "setGlobalGain") {
		if (numArgs != 2) {
			tDisp.print("USAGE: setGlobalGain <gain>");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			forceGain = arg[0];
		}
	}

	/// set individual finger force gain. You can set any arbitrary force gain for every participant if they cant do the chord.
	else if (arguments[0] == "setFingerGain") {
		if (numArgs != 6) {
			tDisp.print("USAGE: setFingerGain <gain1> <gain2> ... <gain5>");
		}
		else {
			for (i = 0; i < 5; i++) {
				sscanf(arguments[i + 1].c_str(), "%f", &arg[0]);
				fGain[i] = arg[0];
			}
		}
	}

	/// reset the centers
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

	else if (arguments[0] == "wait_baseline_hold") {
		if (numArgs != 2) {
			tDisp.print("USAGE: wait_baseline_hold 0|1");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			wait_baseline_zone = arg[0];
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
	state = WAIT_BLOCK;
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
	int i;
	for (i = 0; i < NUMDISPLAYLINES; i++) { gs.line[i] = ""; }
	gCounter.reset();
	gCounter.start();
	gNumCorr = 0;
	gNumWrong = 0;
	blockFeedbackFlag = 0;
}

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state 
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback() {
	gs.showLines = 0;
	int i, j, n = 0;
	MyTrial* tpnr;
	double medianRT;
	double vecRT[2000];
	blockFeedbackFlag = 1;

	// putting RT values in an array
	for (i = 0; i < 2000; i++) {
		vecRT[i] = 0;
	}
	for (i = 0; i < trialNum; i++) { //check each trial
		tpnr = (MyTrial*)trialVec.at(i);
		if (tpnr->trialCorr == 1) { //if trial was correct
			vecRT[i] = tpnr->RT - 600;
			n++;	//count correct trials
		}
	}

	// calculating the median RT
	if (n > 2) {
		double dummy;
		for (i = 0; i < n - 1; i++) {
			for (j = i + 1; j < n; j++) {
				if (vecRT[i] > vecRT[j]) {
					dummy = vecRT[i];
					vecRT[i] = vecRT[j];
					vecRT[j] = dummy;
				}
			}
		}
		if (n % 2 == 0) {
			i = n / 2;
			medianRT = ((vecRT[i - 1] + vecRT[i]) / 2);
		}
		else {
			i = (n - 1) / 2;
			medianRT = (vecRT[i]);
		}
	}

	// number of correct and wrong trials
	gNumCorr = n;
	gNumWrong = trialNum - n;

	//gScreen.setColor(Screen::white);
	sprintf(buffer, "End of Block");
	gs.line[0] = buffer;
	gs.lineColor[0] = 1;

	sprintf(buffer, "Perc Correct = %d/%d", gNumCorr, gNumCorr + gNumWrong);
	gs.line[1] = buffer;
	gs.lineColor[1] = 1;

	if (n > 2) {
		sprintf(buffer, "Med RT = %.0f ms", medianRT);
		gs.line[2] = buffer;
		gs.lineColor[2] = 1;
	}
}

///////////////////////////////////////////////////////////////
///	My Trial class contains the main info of how a trial in this experiment is run
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////////
MyTrial::MyTrial() {
	int i, j;
	state = WAIT_TRIAL;
	///< INIT TRIAL VARIABLE
	trialCorr = 0;		// flag for tiral being correct or incorrect -> 0: trial error , 1: trial correct
	trialErrorType = 0;	// flag for the type of trial error -> 0: no error , 1: planning error , 2: execution error
	RT = 0;
}

void MyTrial::force(int finger) {
	if (finger == -1) {
		state = WAIT_TRIAL;
	}
	else {
		state = MAX_FORCE;
		forceFinger = finger;
	}
}

///////////////////////////////////////////////////////////////
// Read - Done
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	in >> subNum
		>> chordID
		>> stimFinger
		>> planTime
		>> execMaxTime
		>> feedbackTime
		>> iti
		>> trialLabel;
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	// write to .dat file
	out << subNum << "\t"
		<< chordID << "\t"
		<< planTime << "\t"
		<< execMaxTime << "\t"
		<< feedbackTime << "\t"
		<< iti << "\t"
		<< fGain[0] << "\t"						// finger specific gains
		<< fGain[1] << "\t"
		<< fGain[2] << "\t"
		<< fGain[3] << "\t"
		<< fGain[4] << "\t"
		<< forceGain << "\t"					// Global force gain for all fingers
		<< VERT_SHIFT << "\t"					// vertical shift applied to the screen
		<< VERT_SHIFT + baseTHhi << "\t"		// baseline top thresh
		<< VERT_SHIFT + FLX_TOP_Y1 << "\t"		// ext top threshold
		<< VERT_SHIFT + FLX_BOT_Y1 << "\t"		// ext bottom threshold
		<< VERT_SHIFT - (FLX_TOP_Y1) << "\t"	// flex top threshold
		<< VERT_SHIFT - FLX_BOT_Y1 << "\t"		// flex bot threshold
		<< trialCorr << "\t"					// trial is correct or not
		<< trialErrorType << "\t"				// trial error type
		<< RT << "\t"							// reaction time of each trial. 
		<< trialPoint << "\t"					// points received in each trial
		<< endl;
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) {
	char header[200];
	out << "subNum" << "\t"
		<< "chordID" << "\t"
		<< "planTime" << "\t"
		<< "execMaxTime" << "\t"
		<< "feedbackTime" << "\t"
		<< "iti" << "\t"
		<< "fGain1" << "\t"
		<< "fGain2" << "\t"
		<< "fGain3" << "\t"
		<< "fGain4" << "\t"
		<< "fGain5" << "\t"
		<< "forceGain" << "\t"
		<< "verticalShift" << '\t'
		<< "baselineTopThresh" << '\t'
		<< "extTopThresh" << '\t'
		<< "extBotThresh" << '\t'
		<< "flexTopThresh" << '\t'
		<< "flexBotThresh" << '\t'
		<< "trialCorr" << "\t"
		<< "trialErrorType" << "\t"
		<< "RT" << "\t"
		<< "trialPoint" << "\t"
		<< endl;
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
	int i;
	double diffForce[5] = { 0,0,0,0,0 };
	sprintf(buffer, "TR : %d time: %2.2f slice:%d", gCounter.readTR(), gCounter.readTime(), gCounter.readSlice());
	tDisp.setText(buffer, 2, 0);
	sprintf(buffer, "Time : %2.2f", gTimer[1]);
	tDisp.setText(buffer, 3, 0);

	sprintf(buffer, "State : %d   Trial: %d", state, gExp->theBlock->trialNum);
	tDisp.setText(buffer, 4, 0);

	tDisp.setText("Condition: " + trialLabel, 5, 0);

	// display forces
	tDisp.setText("Forces", 7, 0);
	sprintf(buffer, "F1: %2.2f   F2: %2.2f   F3: %2.2f   F4: %2.2f   F5: %2.2f", gBox.getForce(0), gBox.getForce(1), gBox.getForce(2),
		gBox.getForce(3), gBox.getForce(4));
	tDisp.setText(buffer, 8, 0);

	// display max forces
	tDisp.setText("Max forces", 10, 0);
	sprintf(buffer, "maxF1: %2.2f   maxF2: %2.2f   maxF3: %2.2f   maxF4: %2.2f   maxF5: %2.2f", maxForce[0], maxForce[1], maxForce[2],
		maxForce[3], maxForce[4]);
	tDisp.setText(buffer, 11, 0);

	// display forces
	tDisp.setText("Volts", 13, 0);
	sprintf(buffer, "F1: %2.2f   F2: %2.2f   F3: %2.2f   F4: %2.2f   F5: %2.2f", gBox.getVolts(0), gBox.getVolts(1), gBox.getVolts(2),
		gBox.getVolts(3), gBox.getVolts(4));
	tDisp.setText(buffer, 14, 0);

	//// differential forces
	//for (i = 0; i < 5; i++) {
	//	diffForce[i] = gBox.getForce(i) - gBox.getForce(i);
	//}
	//sprintf(buffer, "D1: %2.2f   D2: %2.2f   D3: %2.2f   D4: %2.2f   D5: %2.2f", diffForce[0], diffForce[1], diffForce[2],
	//	diffForce[3], diffForce[4]);
	//tDisp.setText(buffer, 9, 0);

	// force gains
	sprintf(buffer, "GlobalGain = %1.1f     forceGain = %1.1f %1.1f %1.1f %1.1f %1.1f", forceGain, fGain[0], fGain[1], fGain[2], fGain[3], fGain[4]);
	tDisp.setText(buffer, 16, 0);
}

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD 
///////////////////////////////////////////////////////////////


void MyTrial::updateGraphics(int what) {
	int i;
	char tmpChord;
	double x1, x2, xPos, yPos, xSize, ySize;
	double diffForce[5] = { 0,0,0,0,0 };

	if (blockFeedbackFlag) {
		gScreen.setCenter(Vector2D(0, 0));    // In cm //0,2
		gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
	}

	if (gs.showTarget == 1) { // show target squares
		for (i = 0; i < 5; i++) {
			tmpChord = chordID[i];
			x1 = ((i * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) + FINGER_SPACING;
			x2 = (((i + 1) * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) - FINGER_SPACING;
			xPos = (x1 + x2) * 0.5;
			yPos = (FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
			xSize = x2 - x1;
			ySize = FLX_TOP_Y1 - FLX_BOT_Y1;

			// 0% probability
			if (tmpChord == '0') {

			}
			// 25% probability
			else if (tmpChord == '1') {
				//yPos = (FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
				if (gs.fingerCorrectGraphic[i]) {
					gScreen.setColor(Screen::green);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
				else {
					gScreen.setColor(myColor[7]);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
			}
			// 75% probability
			else if (tmpChord == '2') {
				yPos = (FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
				if (gs.fingerCorrectGraphic[i]) {
					gScreen.setColor(Screen::green);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
				else {
					gScreen.setColor(myColor[6]);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
			}
			// 100% probability
			else if (tmpChord == '3') {
				yPos = (FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
				if (gs.fingerCorrectGraphic[i]) {
					gScreen.setColor(Screen::green);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
				else {
					gScreen.setColor(Screen::white);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
			}

			// 50% probability
			else if (tmpChord == '4') {
				yPos = (FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
				if (gs.fingerCorrectGraphic[i]) {
					gScreen.setColor(Screen::green);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
				else {
					gScreen.setColor(myColor[5]);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
			}
		}
	}

	if (gs.showLines == 1) { // show lines
		// Baseline box
		gScreen.setColor(myColor[gs.boxColor]);
		gScreen.drawBox(FINGWIDTH * N_FINGERS, (baseTHhi) * 2, 0, VERT_SHIFT);

		// Baseline lines
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT + baseTHhi, BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT + baseTHhi);
		gScreen.drawLine(BASELINE_X1 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT - baseTHhi, BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT - baseTHhi);

		// Ext Bottom threshold
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_BOT_Y1, BASELINE_X2, VERT_SHIFT + FLX_BOT_Y2);
		// Ext Top threshold
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_TOP_Y1, BASELINE_X2, VERT_SHIFT + FLX_TOP_Y2);
		// Ext Box
		//gScreen.setColor(Screen::green);
		//gScreen.drawBox(FINGWIDTH * N_FINGERS, FLX_ZONE_WIDTH, 0, VERT_SHIFT + 0.5 + FLX_BOT_Y1 + FLX_ZONE_WIDTH/2);

		// Flx Top threshold	
		//gScreen.setColor(Screen::grey);
		//gScreen.drawLine(1. * BASELINE_X1, VERT_SHIFT - FLX_BOT_Y1, 1. * BASELINE_X2, VERT_SHIFT - (FLX_BOT_Y2));
		// Flx Bottom threshold
		//gScreen.setColor(Screen::grey);
		//gScreen.drawLine(1. * BASELINE_X1, VERT_SHIFT - (FLX_TOP_Y1), 1. * BASELINE_X2, VERT_SHIFT - (FLX_TOP_Y2));
		// Flx Box
		//gScreen.setColor(Screen::green);
		//gScreen.drawBox(FINGWIDTH * N_FINGERS, FLX_ZONE_WIDTH, 0, VERT_SHIFT -0.5 + EXT_TOP_Y1 - FLX_ZONE_WIDTH / 2);

		/*
		// Finger forces (flexion)
		for (i = 0; i < 5; i++) {
			gScreen.setColor(Screen::red);
			gScreen.drawLine(((i * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) + FINGER_SPACING, VERT_SHIFT + forceGain*gBox[1].getForce(i), (((i + 1) * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) - FINGER_SPACING, VERT_SHIFT + forceGain*gBox[1].getForce(i));
		}
		// Finger forces (extension)
		for (i = 0; i < 5; i++) {
			gScreen.setColor(Screen::blue);
			gScreen.drawLine(((i * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) + FINGER_SPACING, VERT_SHIFT - forceGain*gBox[0].getForce(i), (((i + 1) * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) - FINGER_SPACING, VERT_SHIFT - forceGain*gBox[0].getForce(i));
		}
		*/
	}

	if (gs.showForces == 1) {
		for (i = 0; i < 5; i++) {
			diffForce[i] = fGain[i] * gBox.getForce(i) + baselineCorrection;
		}
		// Finger forces (difference -> force = f_ext - f_flex)
		for (i = 0; i < 5; i++) {
			gScreen.setColor(Screen::red);
			gScreen.drawLine(((i * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) + FINGER_SPACING, VERT_SHIFT + forceGain * diffForce[i], (((i + 1) * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) - FINGER_SPACING, VERT_SHIFT + forceGain * diffForce[i]);
		}

		//if (gs.showTimer5) {
		//	gScreen.setColor(Screen::white);
		//  gScreen.print(to_string(gBox.getVolts(0)), 6, 3, 4);
		//	gScreen.print(to_string(gTimer[5]), 10, 3, 4);
		//}
	}

	if (gs.showMaxForces == 1) {
		currentForce = gBox.getForce(forceFinger);
		if (currentForce > maxForce[forceFinger]) {
			maxForce[forceFinger] = currentForce;
		}
		gScreen.print(to_string(maxForce[forceFinger]), 6, 3, 4);
	}

	// Other letters
	gScreen.setColor(Screen::white);
	for (i = 0; i < NUMDISPLAYLINES; i++) {
		if (!gs.line[i].empty()) {
			gScreen.setColor(gs.lineColor[i]);
			gScreen.print(gs.line[i].c_str(), gs.lineXpos[i], gs.lineYpos[i], gs.size[i] * 1.5);
		}
	}

	if (gs.showFeedback) { // show feedback
		gScreen.setColor(Screen::white);
		if (gs.rewardTrial == 1)
			gScreen.print("Point: +1", 0, 7, 4);
		else if (gs.rewardTrial == 0)
			gScreen.print("Point: 0", 0, 7, 4);
		else if (gs.rewardTrial == -1)
			gScreen.print("Point: -1", 0, 7, 4);
		//if (gs.planError)
			//gScreen.print("-Moved during planning-", 0, 3, 7);
		//if (gs.chordError)
			//gScreen.print("-Chord too short-", 0, 3, 7);
	}

	if (gs.showDiagnostics) {
		string stateString;
		switch (state)
		{
		case WAIT_TRIAL:
			stateString = "Wait Trial";
			break;
		case START_TRIAL:
			stateString = "Start Trial";
			break;
		case WAIT_PLAN:
			stateString = "Wait Plan";
			break;
		case WAIT_EXEC:
			stateString = "Wait Exec";
			break;
		case GIVE_FEEDBACK:
			stateString = "Give Feedback";
			break;
		case WAIT_ITI:
			stateString = "Wait ITI";
			break;
		case END_TRIAL:
			stateString = "End Trial";
			break;
		case MAX_FORCE:
			stateString = "Max Force";
			break;
		}
		gScreen.setColor(Screen::white);
		gScreen.print(stateString, -21, 12, 5);
	}

}

//////////////////////////////////////////////////////////////////////
/// updateHaptics: called from Hardware interrupt to allow for regular update intervals 
//////////////////////////////////////////////////////////////////////
void MyTrial::updateHaptics() {
	/// Update clocks and manipulandrum
	gTimer.countup();
	gTimer.countupReal();
	s626.updateAD(0);
	// scan
	gCounter.update();
	gBox.update();
	//gBox[1].update();
	/// Call the Trial for control 
	currentTrial->control();

	/// record the data at record frequency 
	if (dataman.isRecording()) {
		bool x = dataman.record(DataRecord(state));
		if (!x) {
			dataman.stopRecording();
		}
	}
}

//////////////////////////////////////////////////////////////////////
// control Trial: A state-driven routine to guide through the process of a trial
//////////////////////////////////////////////////////////////////////
bool planErrorFlag = 0;		// flag for checking if error happens during planning.
bool chordErrorFlag = 0;	// flag for checking if the chord was correct or not.
bool fingerCorrect[5] = { 0,0,0,0,0 };
bool chordCorrect = 0;
void MyTrial::control() {
	int i;
	double fingerForceTmp;
	char tmpChord;
	bool check_baseline_hold = 0;

	gs.showDiagnostics = 0;

	switch (state) {
	case WAIT_TRIAL: //0
		gs.showLines = 1;	// set screen lines/force bars to show
		gs.showForces = 1;
		gs.showFeedback = 0;
		gs.showTarget = 0;
		gs.showMaxForces = 0;
		gs.showTimer5 = 0;
		// gs.showForceBars = 1;
		//gBox.setVolts(-5.0, -5.0, -5.0, -5.0, -5.0);
		gs.rewardTrial = 0;
		trialPoint = 0;
		gs.planError = 0;
		gs.boxColor = 5;	// grey baseline box color
		planErrorFlag = 0;
		//SetDacVoltage(0, 0);	// Ali EMG - gets ~200us to change digital to analog. Does it interrupt the ADC?
		SetDIOState(0, 0xFFFF); // Ali EMG

		for (i = 0; i < NUMDISPLAYLINES; i++) {
			if (!gs.line[i].empty()) {
				gs.lineColor[i] = 0;
				gs.line[i] = "";
			}
		}
		break;

	case MAX_FORCE:
		gs.showLines = 0;
		gs.showForces = 0;
		gs.showMaxForces = 1;

		fGain[forceFinger] = abs(baselineCorrection) / (0.1 * maxForce[forceFinger]);

		break;
	case START_TRIAL: //1	e
		gs.showLines = 1;	// set screen lines/force bars to show
		gs.showFeedback = 0;
		gs.showForces = 1;
		gs.showTimer5 = 0;
		// gs.showForceBars = 1;
		gs.boxColor = 5;	// grey baseline box color
		gs.planError = 0;
		gs.chordError = 0;
		planErrorFlag = 0;	// initialize planErrorFlag variable in the begining of each trial
		chordErrorFlag = 1;	// initialize chordErrorFlag variable in the begining of each trial
		startTriggerEMG = 1;	// Ali EMG: starts EMG trigger in the beginning of each trial

		//SetDacVoltage(0, emgTrigVolt);	// Ali EMG - gets ~200us to change digital to analog. Does it interrupt the ADC?
		SetDIOState(0, 0x0000);

		for (i = 0; i < 5; i++) {
			gs.fingerCorrectGraphic[i] = 0;
		}

		// start recording , reset timers
		dataman.clear();
		dataman.startRecording();
		gTimer.reset(0);
		gTimer.reset(1);	// timer for each trial
		gTimer.reset(2);
		gTimer.reset(3);
		gTimer.reset(5);
		gTimer.reset(6);

		state = WAIT_PLAN;
		break;

	case WAIT_PLAN: //2
		//gs.planCue = 1;
		gs.showTimer5 = 0;
		gs.showForces = 1;

		//// if wait baseline zone is off, we have limited planning time and subjects might make planning error:
		//if (wait_baseline_zone == 0) {
		//	if (gTimer[3] >= 300) {	// turn on visual target after 300ms
		//		gs.showTarget = 1;	// show visual target	
		//	}
		//	else {
		//		gs.showTarget = 0;	// dont show visual target
		//	}

		//	for (i = 0; i < 5; i++) {	// check fingers' states -> fingers should stay in the baseline during planing
		//		fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * gBox.getForce(i);
		//		if (fingerForceTmp >= (VERT_SHIFT + baseTHhi) || fingerForceTmp <= (VERT_SHIFT - (baseTHhi))) {
		//			planErrorFlag = 1;
		//			// gs.showForceBars = 0;
		//			break;
		//		}
		//	}
		//	if (planErrorFlag) {
		//		gs.boxColor = 3;	// baseline box becomes red
		//	}

		//	if (gTimer[1] >= planTime) {
		//		if (planErrorFlag == 1) {
		//			state = GIVE_FEEDBACK;
		//		}
		//		else {
		//			state = WAIT_EXEC;
		//		}
		//		gTimer.reset(2);	// resetting timer 2 to use in next state
		//		gTimer.reset(3);	// resetting timer 3 to use in next state
		//		gTimer.reset(5);	// resetting timer 4 to use in next state

		//		// give finger perturbation
		//		//gBox.setVolts(0, 1, 0, 0, 0);
		//	}
		//}
		// if wait baseline zone was on, the code waits until the subject holds the baseline zone for 500ms and then gives the go cue. 
		// So in this case, planning error is impossible to happen:
		//else {
		for (i = 0; i < 5; i++) {	// check fingers' states -> fingers should stay in the baseline during planning
			fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * gBox.getForce(i) + baselineCorrection;
			check_baseline_hold = 1;
			if (fingerForceTmp >= (VERT_SHIFT + baseTHhi) || fingerForceTmp <= (VERT_SHIFT - (baseTHhi))) {
				// if even one finger was out of baseline zone, reset timer(1):
				gTimer.reset(3);
				check_baseline_hold = 0;
				break;
			}
		}

		if (gTimer[3] >= 500) {	// turn on visual target after 300ms of holding the baseline
			gs.showTarget = 1;	// show visual target	
		}
		else {
			gs.showTarget = 0;
		}

		if (check_baseline_hold == 0) {
			gs.boxColor = 3;	// baseline zone color becomes red
		}
		else {
			gs.boxColor = 5;	// baseline zone color becomes grey
		}

		// if subjects holds the baseline zone for plan time after visual cue was shown go to execution state:
		if (gTimer[3] >= 500 + planTime) {
			state = WAIT_EXEC;
			gTimer.reset(2);	// resetting timer 2 to use in next state
			gTimer.reset(3);	// resetting timer 3 to use in next state
			gTimer.reset(5);	// resetting timer 4 to use in next state

			// give finger perturbation
			//gBox.setVolts(0, 1, 0, 0, 0);
		}

		//tDisp.print(std::to_string(gTimer[3]));

		//////// if subject takes too long to go in the planning zone:
		//if (gTimer[1] >= 6000) {
		////	planErrorFlag = 1;
		////	state = GIVE_FEEDBACK;
		////	gTimer.reset(2);	// resetting timer 2 to use in next state
		////	gTimer.reset(3);	// resetting timer 3 to use in next state
		////	gTimer.reset(5);	// resetting timer 4 to use in next state

		//}
		//}
		break;

	case WAIT_EXEC:

		// deliver finger perturbation
		for (i = 0; i < 5; i++) {
			if (stimFinger[i] == '1')
			{
				sF[i] = 5;
			}

		}

		gBox.setVolts(0,
			sF[1],
			0,
			sF[3],
			0);

		gs.showLines = 1;		// show force bars and thresholds
		gs.showForces = 0;
		gs.showTarget = 0;		// show the targets on the screen (grey bars)
		gs.showTimer5 = 0;		// show timer 4 value on screen (duration of holding a chord)
		gs.boxColor = 5;		// grey baseline box color


		//// checking state of each finger
		//for (i = 0; i < 5; i++) {
		//	tmpChord = chordID[i];	// required state of finger i -> 0:relaxed , 1:extended , 2:flexed -- chordID comes from the target file
		//	fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * gBox.getForce(i);
		//	switch (tmpChord) {
		//	case '9':	// finger i stimulated with 0% probability
		//		fingerCorrect[i] = ((fingerForceTmp <= VERT_SHIFT + baseTHhi) && (fingerForceTmp >= VERT_SHIFT - baseTHhi));
		//		break;
		//	case '1':	// finger i stimulated with 25% probability
		//		fingerCorrect[i] = ((fingerForceTmp <= VERT_SHIFT + FLX_TOP_Y1) && (fingerForceTmp >= VERT_SHIFT + FLX_BOT_Y1));
		//		break;
		//	case '2':	// finger i stimulated with 75% probability
		//		fingerCorrect[i] = ((fingerForceTmp <= VERT_SHIFT + FLX_BOT_Y1) && (fingerForceTmp >= VERT_SHIFT - (FLX_TOP_Y1)));
		//		break;
		//	case '3':	// finger i stimulated with 100% probability
		//		fingerCorrect[i] = ((fingerForceTmp <= VERT_SHIFT + FLX_BOT_Y1) && (fingerForceTmp >= VERT_SHIFT - (FLX_TOP_Y1)));
		//		break;
		//	}
		//	gs.fingerCorrectGraphic[i] = 1;
		//}

		//// Checking if the whole chord is correct
		//chordCorrect = fingerCorrect[0];
		//for (i = 1; i < 5; i++) {
		//	chordCorrect = chordCorrect && fingerCorrect[i];
		//}

		//// resetting timer 5 every time the whole chord is wrong
		//if (chordCorrect == 0) {
		//	gTimer.reset(5);
		//}

		//// if subject held the chord for execAccTime (accepting hold time), trial is correct -> go to feedback state:
		//if (gTimer[5] >= execAccTime) {
		//	// chord was executed successfully so error = 0:
		//	//chordErrorFlag = 0;

		//	// RT equals the time it took the subject to execute the chord successfully after the go cue. 
		//	// Also, remember that chord is held for 600ms so RT = 600ms + actual_RT:
		//	//RT = gTimer[2];

		//	// go to the give_feedback state:
		//	state = GIVE_FEEDBACK;

		//	// resetting timers:
		//	gTimer.reset(2);
		//	gTimer.reset(3);

		//	// give finger perturbation
		//	//gBox.setVolts(0, 0, 0, 0, 0);

		//}

		// If subject runs out of time:
		if (gTimer[3] >= execMaxTime) {
			//chordErrorFlag = 1;
			//RT = 10000;
			state = GIVE_FEEDBACK;
			gTimer.reset(2);
			gTimer.reset(3);

		}
		break;

	case GIVE_FEEDBACK:
		//SetDacVoltage(0, 0); // Ali EMG
		SetDIOState(0, 0xFFFF);

		// end finger perturbation
		sF[1] = 0;
		sF[3] = 0;
		gBox.setVolts(0, 0, 0, 0, 0);

		gs.showLines = 1;			// no force lines/thresholds
		gs.showForces = 1;
		gs.showTarget = 0;			// no visual targets
		gs.showTimer5 = 0;
		gs.showFeedback = 0;		// showing feedback (refer to MyTrial::updateGraphics() for details)
		//if (planErrorFlag == 1) {	// if error occurred during planning
		//	gs.rewardTrial = -1;	// set reward to -1
		//	trialPoint = -1;		// reward variable to save in .dat file
		//	gs.planError = 1;		// set planError to 1 -> this enables verbal feedback to the participant
		//	trialCorr = 0;
		//	trialErrorType = 1;		// trial error type saved in the .dat file to know this was a planning error
		//}
		//else {
		//	if (chordErrorFlag) {	// if exeution error happens; i.e. subject could not execute the chord before max trial time.
		//		gs.rewardTrial = 0;	// set reward to zero
		//		trialPoint = 0;		// reward variable to save in .dat file
		//		gs.chordError = 1;	// give feedback for execution error 
		//		trialCorr = 0;
		//		trialErrorType = 2; // trial error type saved in the .dat file to know this was an execution error
		//	}
		//	else {
		//		gs.rewardTrial = 1;	// If no error -> sets reward to 1
		//		trialPoint = 1;		// reward variable to save in .dat file
		//		trialCorr = 1;
		//		trialErrorType = 0;
		//	}
		//}

		if (gTimer[2] >= feedbackTime) {
			state = WAIT_ITI;
			gTimer.reset(2);
		}
		break;

	case WAIT_ITI:
		gs.showLines = 1;
		gs.showForces = 0;
		gs.showTarget = 0;
		gs.showFeedback = 0;
		if (gTimer[2] >= iti) {
			state = END_TRIAL;
			dataman.stopRecording();
			gTimer.reset(2);
		}
		break;

	case END_TRIAL:
		break;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/// Data Record: creator records the current data from the device 
/////////////////////////////////////////////////////////////////////////////////////
DataRecord::DataRecord(int s) {
	int i, j;
	state = s;
	time = gTimer[1];
	timeReal = gTimer.getRealtime();


	//for (j = 0; j < 5; j++) {
		//fforce[j] = gBox.getForce(j);
	//}

	for (i = 0; i < 5; i++) {
		//diffForceMov[i] = (gBox[0].getForce(i) - gBox[1].getForce(i));	// diffForceMov = f_ext - f_flex
		visualizedForce[i] = VERT_SHIFT + forceGain * gBox.getForce(i);	// The position of the force bars that are shown on the screen
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream& out) {
	int i, j;
	out << state << "\t"
		<< timeReal << "\t"
		<< time << "\t";
	for (i = 0; i < 2; i++) {	// Flexion and extension force -> fforce[0][:] is extension forces and fforce[1][:] is flexion forces
		for (j = 0; j < 5; j++) {
			out << fforce[i][j] << "\t";
		}
	}
	for (i = 0; i < 5; i++) {	// Differential forces -> diffForceMov = extension force - flexion force
		out << diffForceMov[i] << "\t";
	}
	for (i = 0; i < 5; i++) {	// Position of visualized force bars
		out << visualizedForce[i] << "\t";
	}
	out << endl;
}

/////////////////////////////////////////////////////////////////////////////////////
///	Graphic State
/// Collection of current variables relating to what's on the screen 
/// contains 4 lines for display 
/// 
/////////////////////////////////////////////////////////////////////////////////////
GraphicState::GraphicState() {

	// points in block 
	lineXpos[0] = 0;
	lineYpos[0] = 6;			// feedback 	
	lineColor[0] = 1;			// white 
	size[0] = 5;

	// RT 
	lineXpos[1] = 0;
	lineYpos[1] = 5;			// feedback 	
	lineColor[1] = 1;			// white 
	size[1] = 5;

	// total points 
	lineXpos[2] = 0;
	lineYpos[2] = 4;			// block points	
	lineColor[2] = 1;			// white 
	size[2] = 5;

	showLines = true;
	showBoxes = 0;
	boxColor = 5;
}

void GraphicState::reset(void) {
	for (int i = 0; i < NUMDISPLAYLINES; i++) {
		line[i] = "";
	}
}

void SetDacVoltage(WORD channel, DOUBLE volts)
{
	// Make adjustments to prevent conversion errors.
	if (volts > 10.0) volts = 10.0;
	else if (volts < -10.0) volts = -10.0;
	// Program new DAC setpoint.
	S626_WriteDAC(0, channel, (LONG)(volts * DAC_VSCALAR));
}

void SetDIOState(WORD group, WORD states)
{
	// Program new DAC setpoint.
	//S626_WriteDAC(0, channel, (LONG)(volts * DAC_VSCALAR));
	S626_DIOWriteBankSet(0, group, states);
}