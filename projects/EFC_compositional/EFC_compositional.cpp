///////////////////////////////////////////////////////////////
/// ExtensionFlextionChord - Ali Ghavampour , Nov 2022
///////////////////////////////////////////////////////////////
#include "EFC_compositional.h" 
#include "StimulatorBox.h"
#include "Vector2d.h"
#include <vector>
#include <iostream>
#include <numeric>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

///////////////////////////////////////////////////////////////
/// Global variables 
///////////////////////////////////////////////////////////////
S626sManager s626;				///< Hardware Manager 
TextDisplay tDisp;				///< Text Display
Screen gScreen;					///< Screen 
StimulatorBox gBox[2];			///< Stimulator Box
TRCounter gCounter;				///< TR Counter 
Timer gTimer(UPDATERATE);		///< Timer from S626 board experiments 
HapticState hs;					///< This is the haptic State as d by the interrupt 
GraphicState gs;				///< Graphic state
char buffer[300];				///< String buffer 
HINSTANCE gThisInst;			///< Instance of Windows application
Experiment* gExp;				///< Pointer to myExperiment 
Trial* currentTrial;			///< Pointer to current Trial 
#define DAC_VSCALAR 819.1 // Binary-to-volts scalar for DAC.
bool gTimerFlagFirst = 0;
Matrix2D 	TransforMatrix(1, 0, 0, 1);	///< adjusts for the fact that subject screen is flipped. used in angles (0,1,1,0)

bool chordStarted = 0;

double t1;

FixCross fixationCross;

ForceCursor forceCursor[5];
int holdTime = 0;

///< Screen graphics defenitions
#define baseTHhi  1.2 //0.8//1.0			// Baseline higher threshold (to check for premature movements during sequence planning phase)
double fGain[5] = { 1.0,1.0,1.0,1.5,1.5 };	// finger specific force gains -> applied on each finger
double forceGain = 1;						// universal force gain -> applied on all the fingers
bool blockFeedbackFlag = 0;
bool wait_baseline_zone = 1;				// if 1, waits until the subject's fingers are all in the baseline zone.

#define FINGWIDTH 1.3
#define N_FINGERS 5
#define FINGER_SPACING 0.2
#define BASELINE_X1 -(FINGWIDTH*N_FINGERS/2)
#define BASELINE_X2 +(FINGWIDTH*N_FINGERS/2)

#define FIXCROSS_SIZE 3
#define FIXCROSS_THICK 0.3 

#define FLX_ZONE_WIDTH 3
#define FLX_BOT_Y1 2
#define FLX_TOP_Y1 FLX_BOT_Y1+FLX_ZONE_WIDTH
#define FLX_BOT_Y2 FLX_BOT_Y1
#define FLX_TOP_Y2 FLX_TOP_Y1
#define VERT_SHIFT 0	// vertical shift of the screen graphics

///< Visualization colors
Color_t myColor[7] = {
{0,0,0},			// Black
{255,255,255},		// White 
{0,200,0},			// Green 
{150,0,0},			// Red 
{130,130,130}, 		// gray
{100,100,100},		// baseline gray
{249,215,28} };		// yellow

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

std::string get_current_time() {
	char buffer[20];
	std::time_t now = std::time(nullptr);
	std::strftime(buffer, sizeof(buffer), "%d-%m-%Y,%H:%M:%S", std::localtime(&now));
	return std::string(buffer);
}

///////////
////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the fingerBox and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	// 1. initialization window, text display and screen
	gThisInst = hThisInst;
	gExp = new MyExperiment("EFC_compositional", "EFC_compositional", "C:/data/EFC_compositional/");
	//gExp->redirectIOToConsole();

	gExp->redirectIOToConsole();
	tDisp.init(gThisInst, 0, 0, 400, 20, 9, 2, &(::parseCommand));
	tDisp.setText("Subj", 0, 0);
	gScreen.init(gThisInst, 1920, 0, 1440, 900, &(::updateGraphics));	// Default setting for the Windows 10 PC (Behav training/testing)
	gScreen.setCenter(Vector2D(0, 0));									// In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));					// cm/pixel

	// 2. initalize s626cards 
	s626.init("c:/robotcode/calib/s626_single.txt");
	if (s626.getErrorState() == 0) {
		cout << "Initializing S626 Card" << endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); // initialize at 200 Hz update rate 
	}

	fixationCross.position = gScreen.getCenter();
	fixationCross.size = Vector2D(FIXCROSS_SIZE, FIXCROSS_SIZE);
	fixationCross.setShape(SHAPE_PLUS);

	for (size_t i = 0; i < 5; ++i) {
		forceCursor[i].size = Vector2D(FINGWIDTH - FINGER_SPACING * 2, FINGWIDTH - FINGER_SPACING * 2);
		forceCursor[i].setColor(SCR_RED);
	}

	gTimer.init(); // Ali Changed Here!!!!

	gBox[0].init(BOX_LEFT, "c:/robotcode/calib/Flatbox1_highforce2_LEFT_12-Feb-2022.txt");
	gBox[1].init(BOX_RIGHT, "c:/robotcode/calib/Flatbox1_highforce2_RIGHT_03-Dec-2021.txt");

	gCounter.init3(3, 0, 32); // TTL pulse for counting TR

	gExp->control();

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
	theTrial = new MyTrial();
	currentTrial = theTrial;
}

////////////////////////////////////////////////////////////////////////
// MyExperiment: control 
////////////////////////////////////////////////////////////////////////
void MyExperiment::control(void) {
	MSG msg;

	//if (!gTimerFlagFirst) { // Ali Changed Here!!!!
	//	gTimer.init();
	//	gTimerFlagFirst = TRUE;
	//}

	do {
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
		double volts[2][5] = { {0,0,0,0,0},{0,0,0,0,0} };
		int n, j;
		for (n = 0; n < 100; n++) {
			for (b = 0; b < 2; b++) {
				for (j = 0; j < 5; j++) {
					volts[b][j] += gBox[b].getVolts(j);
				}
			}
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10);
		}
		cout << endl;
		for (b = 0; b < 2; b++) {
			for (j = 0; j < 5; j++) {
				volts[b][j] /= 100;
				cout << volts[b][j] << "  " << endl;
			}
			gBox[b].zeroForce(volts[b]);
		}
		tDisp.unlock();
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

	else if (arguments[0] == "flipscreen" || arguments[0] == "FLIPSCREEN") {

		gs.flipscreen = !gs.flipscreen;

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

	//else if (arguments[0] == "execAccTime") {
	//	if (numArgs != 2) {
	//		tDisp.print("USAGE: execAccTime <time in milliseconds>");
	//	}
	//	else {
	//		sscanf(arguments[1].c_str(), "%f", &arg[0]);
	//		execAccTime = arg[0];

	//	}
	//}

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
	gs.showForces = 0;
	int max_holdTime;
	int i, j, n = 0;
	MyTrial* tpnr;
	double medianET = 0;
	//double medianMD;
	double vecET[2000];
	//double vecMD[2000];
	blockFeedbackFlag = 1;

	// Median time from go cue to initial correct chord acquisition,
	// calculated only from successful trials.
	for (i = 0; i < trialNum; i++) {
		tpnr = (MyTrial*)trialVec.at(i);
		if (tpnr->trialPoint == 1) {
			vecET[n] = tpnr->RT + tpnr->ET;
			n++;
		}
	}

	if (n > 0) {
		double dummy;
		for (i = 0; i < n - 1; i++) {
			for (j = i + 1; j < n; j++) {
				if (vecET[i] > vecET[j]) {
					dummy = vecET[i];
					vecET[i] = vecET[j];
					vecET[j] = dummy;
				}
			}
		}

		if (n % 2 == 0) {
			medianET = (vecET[n / 2 - 1] + vecET[n / 2]) / 2.0;
		}
		else {
			medianET = vecET[n / 2];
		}
	}

	// number of correct and wrong trials
	gNumWrong = trialNum - gNumCorr;

	sprintf(buffer, "End of Block");
	gs.line[0] = buffer;
	gs.lineColor[0] = 1;

	sprintf(buffer, "Perc Correct = %d/%d", gNumCorr, gNumCorr + gNumWrong);
	gs.line[1] = buffer;
	gs.lineColor[1] = 1;

	if (n > 0) {
		sprintf(buffer, "Median execution time = %.2f s", medianET);
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
	trialErrorType = 0;	// flag for the type of trial error -> 0: no error , 1: planning error , 2: execution error
	RT = 0;
	ET = 0;
	fixationCross.setColor(SCR_WHITE);

}

///////////////////////////////////////////////////////////////
// Read - Done
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	in  >> subNum
		>> chordID
		>> planTime
		>> success_holdTime
		>> execMaxTime
		>> feedbackTime
		>> iti;
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	// write to .dat file
	out << subNum << "\t"
		<< chordID << "\t"
		<< planTime << "\t"
		<< max_holdTime << "\t"
		<< success_holdTime << "\t"
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
		<< RT << "\t"							// reaction time of each trial. 
		<< ET << "\t"
		<< trialPoint << "\t"					// points received in each trial
		<< planError << "\t"
		<< current_time << "\t"
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
		<< "max_holdTime" << "\t"
		<< "success_holdTime" << "\t"
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
		<< "RT" << "\t"
		<< "ET" << '\t'
		<< "trialPoint" << "\t"
		<< "planError" << "\t"
		<< "DateTime" << "\t"
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
	sprintf(buffer, "time: %2.2f", gCounter.readTime());
	tDisp.setText(buffer, 2, 0);
	sprintf(buffer, "Time : %2.2f", gTimer[1]);
	tDisp.setText(buffer, 3, 0);

	sprintf(buffer, "State : %d   Trial: %d    Hold time: %d    Max hold time: %d, trialPoint: %d, RT: %4.2f, ET: %4.2f, nCorr: %d",
		state, gExp->theBlock->trialNum, holdTime, max_holdTime, trialPoint, RT, ET, gNumCorr);
	tDisp.setText(buffer, 4, 0);


	// display forces
	tDisp.setText("Forces", 6, 0);
	sprintf(buffer, "F1: %2.2f   F2: %2.2f   F3: %2.2f   F4: %2.2f   F5: %2.2f", gBox[1].getForce(0), gBox[1].getForce(1), gBox[1].getForce(2),
		gBox[1].getForce(3), gBox[1].getForce(4));
	tDisp.setText(buffer, 7, 0);
	sprintf(buffer, "E1: %2.2f   E2: %2.2f   E3: %2.2f   E4: %2.2f   E5: %2.2f", gBox[0].getForce(0), gBox[0].getForce(1), gBox[0].getForce(2),
		gBox[0].getForce(3), gBox[0].getForce(4));
	tDisp.setText(buffer, 8, 0);

	// differential forces
	for (i = 0; i < 5; i++) {
		diffForce[i] = gBox[0].getForce(i) - gBox[1].getForce(i);
	}
	sprintf(buffer, "D1: %2.2f   D2: %2.2f   D3: %2.2f   D4: %2.2f   D5: %2.2f", diffForce[0], diffForce[1], diffForce[2],
		diffForce[3], diffForce[4]);
	tDisp.setText(buffer, 9, 0);

	// force gains
	sprintf(buffer, "GlobalGain = %1.1f     forceGain = %1.1f %1.1f %1.1f %1.1f %1.1f ", forceGain, fGain[0], fGain[1], fGain[2], fGain[3], fGain[4]);
	tDisp.setText(buffer, 10, 0);
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
		if (gs.flipscreen == 1) {
			TransforMatrix = Matrix2D(0, 1, 1, 0);
			gScreen.setScale(Vector2D(-SCR_SCALE, SCR_SCALE)); // this is the flipped
			//flipscreen = true;
		}

		else { // flipscreen is true, is in mri mode, going to training mode
			TransforMatrix = Matrix2D(1, 0, 0, 1);
			gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
			//flipscreen = false;
		}

	}

	if (gs.showTarget == 1) {
		for (i = 0; i < 5; i++) {
			tmpChord = chordID[i];
			x1 = ((i * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) + FINGER_SPACING;
			x2 = (((i + 1) * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) - FINGER_SPACING;
			xPos = (x1 + x2) * 0.5;
			xSize = x2 - x1;
			ySize = FLX_TOP_Y1 - FLX_BOT_Y1;
			if (tmpChord == '9') {

			}
			else if (tmpChord == '1') {
				yPos = (FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
				if (gs.fingerCorrectGraphic[i]) {
					gScreen.setColor(Screen::green);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
				else {
					gScreen.setColor(Screen::grey);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
			}
			else if (tmpChord == '2') {
				yPos = -(FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
				if (gs.fingerCorrectGraphic[i]) {
					gScreen.setColor(Screen::green);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
				else {
					gScreen.setColor(Screen::grey);
					gScreen.drawBox(xSize, ySize, xPos, yPos);
				}
			}
		}
	}

	if (gs.flipscreen == 1) {
		TransforMatrix = Matrix2D(0, 1, 1, 0);
		gScreen.setScale(Vector2D(-SCR_SCALE, SCR_SCALE)); // this is the flipped
		//flipscreen = true;
	}

	else { // flipscreen is true, is in mri mode, going to training mode
		TransforMatrix = Matrix2D(1, 0, 0, 1);
		gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
		//flipscreen = false;
	}

	if (gs.showLines == 1) {
		// Baseline box
		if (gs.planError) {
			gScreen.setColor(myColor[3]); // red baseline box
		}
		else {
			gScreen.setColor(myColor[5]); // grey baseline box
		}
		gScreen.drawBox(FINGWIDTH * N_FINGERS, (baseTHhi) * 2, 0, VERT_SHIFT);

		// Baseline lines
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT + baseTHhi, BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT + baseTHhi);
		gScreen.drawLine(BASELINE_X1 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT - (baseTHhi), BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT - (baseTHhi));

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
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(1. * BASELINE_X1, VERT_SHIFT - FLX_BOT_Y1, 1. * BASELINE_X2, VERT_SHIFT - (FLX_BOT_Y2));
		// Flx Bottom threshold
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(1. * BASELINE_X1, VERT_SHIFT - (FLX_TOP_Y1), 1. * BASELINE_X2, VERT_SHIFT - (FLX_TOP_Y2));
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



		if (gs.showTimer5) {
			gScreen.setColor(Screen::white);
			gScreen.print("Time elapsed: ", 6, 3, 4);
			gScreen.print(to_string(gTimer[5]), 10, 3, 4);
		}
	}

	// Other letters
	gScreen.setColor(Screen::white);
	for (i = 0; i < NUMDISPLAYLINES; i++) {
		if (!gs.line[i].empty()) {
			gScreen.setColor(gs.lineColor[i]);
			gScreen.print(gs.line[i].c_str(), gs.lineXpos[i], gs.lineYpos[i], gs.size[i] * 1.5);
		}
	}

	if (gs.showFeedback) {
		gScreen.setColor(Screen::white);
		if (gs.rewardTrial > 0)
			sprintf(buffer, "+%d, execution time = %.2fs", gs.rewardTrial, ET);
		gs.line[2] = buffer;

		if (gs.planError)
			gScreen.print("-Moved during planning-", 0, 3, 7);
	}

	if (gs.showForces) {
		for (i = 0; i < 5; i++) {
			diffForce[i] = fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
		}

		for (i = 0; i < 5; i++) {

			forceCursor[i].position[0] = (((2 * i + 1) * FINGWIDTH) - (FINGWIDTH * N_FINGERS)) / 2.0;
			forceCursor[i].position[1] = VERT_SHIFT + forceGain * diffForce[i];

			forceCursor[i].draw();

		}
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
	gBox[0].update();
	gBox[1].update();
	/// Call the Trial for control 
	currentTrial->control();

	/// record the data at record frequency 
	if (dataman.isRecording()) {
		bool x = dataman.record(DataRecord(state, gExp->theBlock->trialNum, chordStarted));
		if (!x) {
			dataman.stopRecording();
		}
	}
}

//////////////////////////////////////////////////////////////////////
// control Trial: A state-driven routine to guide through the process of a trial
//////////////////////////////////////////////////////////////////////
bool fingerCorrect[5] = { 0,0,0,0,0 };
bool chordCorrect = 0;
bool prev_chordCorrect;
void MyTrial::control() {
	int i;
	double fingerForceTmp;
	char tmpChord;
	bool check_baseline_hold = 0;

	switch (state) {
	case WAIT_TRIAL: // state = 0
		gs.showLines = 1;
		gs.showFeedback = 0;
		gs.showTarget = 0;
		gs.showTimer5 = 0;
		gs.showForces = 1;
		gs.showDiagnostics = 0;
		gs.rewardTrial = -1;
		trialPoint = 0;
		gs.boxColor = 5;	// grey baseline box color

		for (i = 0; i < NUMDISPLAYLINES; i++) {
			if (!gs.line[i].empty()) {
				gs.lineColor[i] = 0;
				gs.line[i] = "";
			}
		}
		break;
	
	case START_TRIAL: // state =1	
		max_holdTime = 0;
		holdTime = 0;
		chordCorrect = 0;
		prev_chordCorrect = 0;
		chordStarted = 0;
		RT = 0;
		ET = 0;
		gs.showLines = 1;
		gs.showFeedback = 0;
		gs.showTimer5 = 0;
		gs.showForces = 1;
		gs.boxColor = 5;	// grey baseline box color
		gs.chordError = 0;
		trialPoint = 0;
		gs.rewardTrial = -1;
		planError = 0;
		gs.planError = 0;

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
		current_time = get_current_time();
		state = WAIT_PLAN;
		break;

	case WAIT_PLAN: // state = 2
		gs.showTimer5 = 0;
		gs.showForces = 1;
		gs.showLines = 1;
		gs.showTarget = 0;
		gs.planError = 0;

		for (i = 0; i < 5; i++) {
			fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
			if (fingerForceTmp >= (VERT_SHIFT + baseTHhi) || fingerForceTmp <= (VERT_SHIFT - (baseTHhi))) {
				planError = 1;
				gs.planError = 1;
				gs.rewardTrial = 0;
				break;
			}
		}
		
		// if subjects holds the baseline zone for plan time after visual cue was shown go to execution state:
		if (gTimer[3] >= planTime) {
			state = WAIT_EXEC;
			chordStarted = 0;
			prev_chordCorrect = 0;
			gTimer.reset(2);	// resetting timer 2 to use in next state
			gTimer.reset(3);	// resetting timer 3 to use in next state
			gTimer.reset(5);	// resetting timer 4 to use in next state
		}
		break;

	case WAIT_EXEC: // state = 3
		gs.planError = 0;
		gs.showLines = 1;		// show force bars and thresholds
		gs.showTarget = 1;		// show the targets on the screen (grey bars)
		gs.showTimer5 = 0;		// show timer 4 value on screen (duration of holding a chord)
		gs.showForceBars = 1;
		gs.boxColor = 5;		// grey baseline box color
		
		if (chordCorrect == 0 && chordStarted == 0) {
			for (i = 0; i < 5; i++) {	// RT is the time of the first finger outside the baseline area
				fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
				if (fingerForceTmp >= (VERT_SHIFT + baseTHhi) || fingerForceTmp <= (VERT_SHIFT - (baseTHhi))) {
					RT = gTimer[2];
					chordStarted = 1;
					break;
				}
			}
		}

		// checking state of each finger
		for (i = 0; i < 5; i++) {
			tmpChord = chordID[i];	// required state of finger i -> 0:relaxed , 1:extended , 2:flexed -- chordID comes from the target file
			fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
			switch (tmpChord) {
			case '9':	// finger i should be in the baseline zone (relaxed)
				fingerCorrect[i] = ((fingerForceTmp <= VERT_SHIFT + baseTHhi) && (fingerForceTmp >= VERT_SHIFT - baseTHhi));
				break;
			case '1':	// finger i should be in the top zone (extended)
				fingerCorrect[i] = ((fingerForceTmp <= VERT_SHIFT + FLX_TOP_Y1) && (fingerForceTmp >= VERT_SHIFT + FLX_BOT_Y1));
				break;
			case '2':	// finger i should be in the bottom zone (flexed)
				fingerCorrect[i] = ((fingerForceTmp <= VERT_SHIFT - FLX_BOT_Y1) && (fingerForceTmp >= VERT_SHIFT - (FLX_TOP_Y1)));
				break;
			}
			gs.fingerCorrectGraphic[i] = fingerCorrect[i];
		}

		// Checking if the whole chord is correct
		chordCorrect = fingerCorrect[0];
		for (i = 1; i < 5; i++) {
			chordCorrect = chordCorrect && fingerCorrect[i];
		}

		// Measure hold time
		if (chordCorrect == 1 && prev_chordCorrect == 0) {
			ET = gTimer[2] - RT;
			holdTime = 0;
			gTimer.reset(3);
		}
		else if (chordCorrect == 1 && prev_chordCorrect == 1) {
			holdTime = gTimer[3];
			if (holdTime > max_holdTime) {
				max_holdTime = holdTime;
			}
		}

		prev_chordCorrect = chordCorrect;

		// if subject held the chord for success_holdTime (accepting hold time), trial is correct -> go to feedback state:
		if (chordCorrect == 1 && max_holdTime >= success_holdTime) {
			state = GIVE_FEEDBACK;
			gTimer.reset(2);
			gTimer.reset(3);
		}
		
		// if took too long to hold the chord, trial is incorrect -> go to feedback state:
		if (gTimer[5] >= execMaxTime) {
			// go to the give_feedback state:
			state = GIVE_FEEDBACK;

			// resetting timers:
			gTimer.reset(2);
			gTimer.reset(3);
		}

		break;

	case GIVE_FEEDBACK: // state = 4		
		gs.showLines = 1;			// no force lines/thresholds
		gs.showTarget = 0;			// no visual targets
		gs.showTimer5 = 0;
		gs.showFeedback = 0;		// showing feedback (refer to MyTrial::updateGraphics() for details)
		gs.rewardTrial = 0;
		gs.showForces = 1;
		gs.planError = planError;
		if (planError) {
			trialPoint = 0;
			gs.showFeedback = 1;
			gs.rewardTrial = 0;
			RT = execMaxTime;
			ET = execMaxTime;
		}
		else if (chordStarted == 1 && chordCorrect == 1 && max_holdTime >= success_holdTime) {
			trialPoint = 1;
			gs.showFeedback = 1;
			gs.rewardTrial = trialPoint;
		}
		else {
			RT = execMaxTime;
			ET = execMaxTime;
			trialPoint = 0;
		}

		if (gTimer[2] >= feedbackTime) {
			state = WAIT_ITI;
			gTimer.reset(2);
			gNumCorr = gNumCorr + trialPoint;
		}
		break;

	case WAIT_ITI: // state = 5
		gs.showLines = 1;
		gs.showTarget = 0;
		gs.showForces = 1;
		gs.showFeedback = 0;
		gs.rewardTrial = -1;
		if (gTimer[2] >= iti) {
			state = END_TRIAL;
			dataman.stopRecording();
			gTimer.reset(2);
		}
		break;

	case END_TRIAL: // state = 6
		gTimer.reset(1);
		dataman.stopRecording();
		break;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/// Data Record: creator records the current data from the device 
/////////////////////////////////////////////////////////////////////////////////////
DataRecord::DataRecord(int s, int t, bool started) {
	int i, j;
	state = s;
	trialNum = t;
	time = gTimer[1];
	timeReal = gTimer.getRealtime();


	for (i = 0; i < 2; i++) {
		for (j = 0; j < 5; j++) {
			fforce[i][j] = gBox[i].getForce(j);
		}
	}

	vector<double> currentDiffForce(5);
	for (i = 0; i < 5; i++) {
		diffForceMov[i] = (gBox[0].getForce(i) - gBox[1].getForce(i));	// diffForceMov = f_ext - f_flex
		visualizedForce[i] = VERT_SHIFT + forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));	// The position of the force bars that are shown on the screen

		currentDiffForce[i] = forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream& out) {
	int i, j;
	out << trialNum + 1 << "\t"
		<< state << "\t"
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
	lineYpos[0] = 2.4;			// feedback 	
	lineColor[0] = 1;			// white 
	size[0] = 5;

	lineXpos[1] = 0;
	lineYpos[1] = .8;			// feedback 	
	lineColor[1] = 1;			// white 
	size[1] = 5;

	lineXpos[2] = 0;
	lineYpos[2] = -.8;			// block points	
	lineColor[2] = 1;			// white 
	size[2] = 5;

	lineXpos[3] = 0;
	lineYpos[3] = -2.4;			// block points	
	lineColor[3] = 1;			// white 
	size[3] = 5;

	showLines = true;
	showBoxes = 0;
	boxColor = 5;
}

void GraphicState::reset(void) {
	for (int i = 0; i < NUMDISPLAYLINES; i++) {
		line[i] = "";
	}
}