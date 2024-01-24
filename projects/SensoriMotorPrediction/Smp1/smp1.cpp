///////////////////////////////////////////////////////////////
/// SensoriMotorPrediction - Marco Emanuele , Sept 2023
///////////////////////////////////////////////////////////////
#include "smp1.h" 
#include "Vector2d.h"
#include <algorithm> 
#include <iostream>
#include <cmath>
#include <windows.h>

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

double baselineCorrection = -0.8; //-1.7;    // move force cursor away from baseline area at rest (Marco)
//bool maxF = 0;					///< 0>Task 1>Max Voluntary Force Measurament (Marco)
int finger[2] = { 1, 3 };					///< Finger from which Max Voluntary Force is measured (Marco)
double currentForce = 0;			// max force recorded from <finger>
double maxForce[5] = { 0, 0, 0 ,0, 0 };  // max force recorded from each finger
bool showCue = 0;
string probCue;

int sliceNumber = 32;			///< How many slices do we have

string fingers[5] = { "thumb", "index", "middle", "ring", "pinkie" };
string fingerTask[2] = { fingers[finger[0]], fingers[finger[1]] };

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
#define baseTHhi  0.5			// Half height of baseline area
double fGain[5] = { 1.0 ,1.0,1.0,1.0,1.0 };	// finger specific force gains -> applied on each finger
double forceGain = 1;						// universal force gain -> applied on all the fingers
bool blockFeedbackFlag = 0;
bool wait_baseline_zone = 1;				// if 1, waits until the subject's fingers are all in the baseline zone. 
int baseline_wait_time = 500;


#define FINGWIDTH 2 //previously 1.3
#define X_CURSOR_DEV 1.5
#define BASELINE_X1 -3//-(FINGWIDTH*N_FINGERS/2)
#define BASELINE_X2 3//+(FINGWIDTH*N_FINGERS/2)

double xPosBox[2] = { -X_CURSOR_DEV, X_CURSOR_DEV };
#define FLX_ZONE_WIDTH 5
#define FLX_BOT_Y1 6.5 - abs(baselineCorrection)  //fGain[1]*maxForce[1]*tgForce - FLX_ZONE_WIDTH / 2 + baselineCorrection
#define FLX_TOP_Y1 FLX_BOT_Y1+FLX_ZONE_WIDTH 
#define FLX_BOT_Y2 FLX_BOT_Y1
#define FLX_TOP_Y2 FLX_TOP_Y1

#define CROSSW 0.6 // adjust to visual angle based on the distance in the scanner
#define CROSSP (FLX_BOT_Y1 + FLX_TOP_Y1) / 2

#define VERT_SHIFT 0 //-2.5	// vertical shift of the screen graphics

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
char gKey;
bool gKeyPressed;
double gVolts[5] = { 0,0,0,0,0 };   // volts sent to the valves (Marco)
int fingerVolt = 3;


///////////
////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the fingerBox and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	// 1. initialization window, text display and screen
	gThisInst = hThisInst;
	gExp = new MyExperiment("smp1", "smp1", "C:/data/SensoriMotorPrediction/smp1/");
	//gExp->redirectIOToConsole();

	// gExp->redirectIOToConsole();		// I uncommented this!!!
	tDisp.init(gThisInst, 0, 0, 600, 30, 9, 2, &(::parseCommand));		// Default setting for the Windows 10 PC
	tDisp.setText("Subj", 0, 0);
	gScreen.init(gThisInst, 1920, 0, 1920, 1080, &(::updateGraphics));	// Default setting for the Windows 10 PC
	//gScreen.init(gThisInst, 640, 0, 640, 1024, &(::updateGraphics));
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
	gBox.init(BOX_RIGHT, "c:/robotcode/calib/right_lowforce_pneumatic.txt");

	gCounter.init3(3, 0, sliceNumber); // TTL pulse for counting TR
	gCounter.simulate(TRTIME);

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
	float emgTrigVolt = 0;	// Ali EMG

	do {
		//This function checks if there are any messages in the application's message queue. 
		//If a message is available, it's retrieved and stored in the msg variable.
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		theBlock->control();
		currentTrial->copyHaptics();		// Thread save copy 
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
	//int tmpChord;
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

	/// Zero the force of the two fingerBox
	else if (arguments[0] == "showCue") {
		if (numArgs != 2) {
			tDisp.print("USAGE: showCue <probability1probability2)>");
		}

		else if (arguments[1] == "-1") {
			showCue = 0;
		}

		else {
			showCue = 1;
			probCue = arguments[1];

		}
	}

	//// Compute Max Force of each finger
	//else if (arguments[0] == "maxF") {

	//	if (numArgs == 2 && arguments[1] == "-1") {
	//		tDisp.print("Exit maxF");
	//		maxF = 0;
	//		//theTrial->MyTri();
	//	}

	//	else if (numArgs == 3){
	//		finger[0] = std::stoi(arguments[1]);
	//		finger[1] = std::stoi(arguments[2]);
	//		maxF = 1;
	//	}

	//	else if (numArgs == 6) {
	//		for (i = 1; i < 6; i++) {
	//			maxForce[i - 1] = std::stod(arguments[i]);
	//			fGain[i - 1] = abs(baselineCorrection) / (bsForce * maxForce[i - 1]);
	//		}
	//	}

	//	else {
	//		tDisp.print("USAGE: maxF <finger> or maxF <maxF1> <maxF2> <maxF3> <maxF4> <maxF5>, exit>-1)");
	//	}


	//}

	// Baseline correction
	else if (arguments[0] == "baselineC") {
		if (numArgs != 2) {
			tDisp.print("USAGE: baselineC <value>");
		}

		else {
			baselineCorrection = std::stoi(arguments[1]);
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

	/*/// Show the force lines
	else if (arguments[0] == "showlines") {
		if (numArgs != 2) {
			tDisp.print("USAGE: showlines 0/1");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			if (arg[0] > 0) {
				gs.showTgLines = 1;
			}
			else {
				gs.showTgLines = 0;

			}
		}
	}*/

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

	/// set voltage for finger stimulation
	else if (arguments[0] == "fingerVolt") {
		if (numArgs != 2) {
			tDisp.print("USAGE: fingerVolt <Volt>");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			fingerVolt = arg[0];
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
	blockFeedbackFlag = 0;
}

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state 
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback() {
	gs.showTgLines = 0;
	gs.showBsLines = 0;
	gs.showForces = 0;
	int i, j, n = 0;
	MyTrial* tpnr;
	//double medianRT;
	//double vecRT[2000];
	blockFeedbackFlag = 1;

	////gScreen.setColor(Screen::white);
	sprintf(buffer, "End of Block");
	gs.line[0] = buffer;
	gs.lineColor[0] = 1;
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
}

///////////////////////////////////////////////////////////////
// Read - Done
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	in >> subNum
		>> cueID
		>> stimFinger
		>> planTime
		>> execMaxTime
		>> feedbackTime
		>> iti
		>> trialLabel
		>> GoNogo
		>> startTime;
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	// write to .dat file
	// name of file is: smp0_<name of subject>_<session number>.dat
	out << subNum << "\t"
		<< cueID << "\t"
		<< stimFinger << "\t"
		<< planTime << "\t"
		<< execMaxTime << "\t"
		<< feedbackTime << "\t"
		<< startTimeReal << "\t"
		<< startTRReal << "\t"
		<< GoNogo << "\t"
		<< iti << "\t"
		<< baseline_wait_time << "\t"
		<< trialLabel << "\t"
		<< fGain[0] << "\t"						// finger specific gains
		<< fGain[1] << "\t"
		<< fGain[2] << "\t"
		<< fGain[3] << "\t"
		<< fGain[4] << "\t"
		<< forceGain << "\t"					// Global force gain for all fingers
		<< VERT_SHIFT << "\t"					// vertical shift applied to the screen
		<< VERT_SHIFT - baseTHhi << "\t"		// baseline bottom thresh
		<< VERT_SHIFT + baseTHhi << "\t"		// baseline top thresh
		<< VERT_SHIFT + FLX_BOT_Y1 << "\t"		// min response force
		<< baselineCorrection << "\t"
		<< fingerVolt << "\t"					// 
		<< endl;
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) {
	char header[200];
	out << "subNum" << "\t"
		<< "chordID" << "\t"
		<< "stimFinger" << "\t"
		<< "planTime" << "\t"
		<< "execMaxTime" << "\t"
		<< "feedbackTime" << "\t"
		<< "startTimeReal" << "\t"
		<< "startTRReal" << "\t"
		<< "GoNogo" << "\t"
		<< "iti" << "\t"
		<< "baselineWait" << "\t"
		<< "trialLabel" << "\t"
		<< "fGain1" << "\t"
		<< "fGain2" << "\t"
		<< "fGain3" << "\t"
		<< "fGain4" << "\t"
		<< "fGain5" << "\t"
		<< "forceGain" << "\t"
		<< "verticalShift" << '\t'
		<< "baselineBottomThresh" << '\t'
		<< "baselineTopThresh" << '\t'
		<< "minResponseForce" << '\t'
		<< "baselineCorrection" << '\t'
		<< "fingerVolt" << '\t'
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
	sprintf(buffer, "TR : %d time: %2.2f Tot time: %2.2f slice:%d", gCounter.readTR(), gCounter.readTime(), gCounter.readTotTime(), gCounter.readSlice());
	tDisp.setText(buffer, 2, 0);
	sprintf(buffer, "Time : %2.2f", gTimer[1]);
	tDisp.setText(buffer, 3, 0);
	tDisp.setText("Experiment: Smp1", 2, 1);

	sprintf(buffer, "State : %d   Trial: %d", state, gExp->theBlock->trialNum);
	tDisp.setText(buffer, 4, 0);

	tDisp.setText("Fingers in task: " + fingerTask[0] + " " + fingerTask[1], 4, 1);

	tDisp.setText("Condition: " + trialLabel, 5, 0);

	sprintf(buffer, "fingerVolt : %d", fingerVolt);
	tDisp.setText(buffer, 5, 1);

	// display forces
	tDisp.setText("Forces", 7, 0);
	sprintf(buffer, "F1: %2.2f   F2: %2.2f   F3: %2.2f   F4: %2.2f   F5: %2.2f", gBox.getForce(0), gBox.getForce(1), gBox.getForce(2),
		gBox.getForce(3), gBox.getForce(4));
	tDisp.setText(buffer, 8, 0);

	//// display max forces
	//tDisp.setText("Max forces", 10, 0);
	//sprintf(buffer, "maxF1: %2.2f   maxF2: %2.2f   maxF3: %2.2f   maxF4: %2.2f   maxF5: %2.2f", maxForce[0], maxForce[1], maxForce[2],
	//	maxForce[3], maxForce[4]);
	//tDisp.setText(buffer, 11, 0);

	// display forces
	tDisp.setText("Volts", 10, 0);
	sprintf(buffer, "F1: %2.2f   F2: %2.2f   F3: %2.2f   F4: %2.2f   F5: %2.2f", gBox.getVolts(0), gBox.getVolts(1), gBox.getVolts(2),
		gBox.getVolts(3), gBox.getVolts(4));
	tDisp.setText(buffer, 11, 0);

	// display maxF
	sprintf(buffer, "showCue: %d", showCue);
	tDisp.setText(buffer, 10, 1);

	// force gains
	sprintf(buffer, "GlobalGain = %1.1f     forceGain = %1.1f %1.1f %1.1f %1.1f %1.1f", forceGain, fGain[0], fGain[1], fGain[2], fGain[3], fGain[4]);
	tDisp.setText(buffer, 13, 0);
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

		//gScreen.setColor(Screen::white);
		//gScreen.print("End of Block", 0, 0, 5);
	}

	// Other letters
	gScreen.setColor(Screen::white);
	for (i = 0; i < NUMDISPLAYLINES; i++) {
		if (!gs.line[i].empty()) {
			gScreen.setColor(gs.lineColor[i]);
			gScreen.print(gs.line[i].c_str(), gs.lineXpos[i], gs.lineYpos[i], gs.size[i] * 1.5);
		}
	}

	if (gs.showTarget == 1) { // show target squares
		for (i = 0; i < 2; i++) {
			tmpChord = cueID[i];
			xPos = xPosBox[i];//(x1 + x2) * 0.5;
			yPos = (FLX_BOT_Y1 + 3); // *0.5 + VERT_SHIFT;
			xSize = FINGWIDTH;
			ySize = FLX_TOP_Y1 - FLX_BOT_Y1;

			// 0% probability
			if (tmpChord == '0') {

			}
			// 25% probability
			else if (tmpChord == '1') {
				ySize = 0.25 * FLX_ZONE_WIDTH;
				yPos = FLX_BOT_Y1 + ySize * 0.5 + VERT_SHIFT;
				gScreen.setColor(Screen::white);
				gScreen.drawBox(xSize, ySize, xPos, yPos);

			}
			// 75% probability
			else if (tmpChord == '2') {
				ySize = 0.75 * FLX_ZONE_WIDTH;
				yPos = FLX_BOT_Y1 + ySize * 0.5 + VERT_SHIFT;
				gScreen.setColor(Screen::white);
				gScreen.drawBox(xSize, ySize, xPos, yPos);

			}
			// 100% probability
			else if (tmpChord == '3') {
				ySize = 1 * FLX_ZONE_WIDTH;
				yPos = FLX_BOT_Y1 + ySize * 0.5 + VERT_SHIFT;
				gScreen.setColor(Screen::white);
				gScreen.drawBox(xSize, ySize, xPos, yPos);

			}

			// 50% probability
			else if (tmpChord == '4') {
				ySize = 0.5 * FLX_ZONE_WIDTH;
				yPos = FLX_BOT_Y1 + ySize * 0.5 + VERT_SHIFT;
				gScreen.setColor(Screen::white);
				gScreen.drawBox(xSize, ySize, xPos, yPos);

			}
		}
	}

	if (gs.showBsLines == 1) { // show lines
		// Baseline box
		gScreen.setColor(myColor[gs.boxColor]);
		gScreen.drawBox(BASELINE_X2 - BASELINE_X1, (baseTHhi) * 2, 0, VERT_SHIFT);

		// Baseline lines
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + baseTHhi, BASELINE_X2, VERT_SHIFT + baseTHhi);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT - baseTHhi, BASELINE_X2, VERT_SHIFT - baseTHhi);

	}

	if (gs.showFxCross == 1) { // show lines

		// Baseline lines
		gScreen.setColor(Screen::red);
		gScreen.drawLine(-CROSSW / 2, VERT_SHIFT + CROSSP, CROSSW / 2, VERT_SHIFT + CROSSP);
		gScreen.drawLine(0, VERT_SHIFT + CROSSP - CROSSW /2 , 0, VERT_SHIFT + CROSSP + CROSSW/2);

	}

	if (gs.showTgLines == 1) {
		// Ext Bottom threshold
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_BOT_Y1, BASELINE_X2, VERT_SHIFT + FLX_BOT_Y2);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_TOP_Y1, BASELINE_X2, VERT_SHIFT + FLX_TOP_Y2);


	}

	if (gs.showPrLines == 1) {

		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_BOT_Y1 + FLX_ZONE_WIDTH, BASELINE_X2, VERT_SHIFT + FLX_BOT_Y2 + FLX_ZONE_WIDTH);

		gScreen.setColor(myColor[7]);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_BOT_Y1 + 0.25 * FLX_ZONE_WIDTH, BASELINE_X2, VERT_SHIFT + FLX_BOT_Y2 + 0.25 * FLX_ZONE_WIDTH);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_BOT_Y1 + 0.5 * FLX_ZONE_WIDTH, BASELINE_X2, VERT_SHIFT + FLX_BOT_Y2 + 0.5 * FLX_ZONE_WIDTH);
		gScreen.drawLine(BASELINE_X1, VERT_SHIFT + FLX_BOT_Y1 + 0.75 * FLX_ZONE_WIDTH, BASELINE_X2, VERT_SHIFT + FLX_BOT_Y2 + 0.75 * FLX_ZONE_WIDTH);

		gScreen.print("100%", BASELINE_X2 + 0.5, VERT_SHIFT + FLX_BOT_Y1 + FLX_ZONE_WIDTH, 2.5);
		gScreen.print("50%", BASELINE_X2 + 0.5, VERT_SHIFT + FLX_BOT_Y1 + 0.5 * FLX_ZONE_WIDTH, 2.5);
		gScreen.print("25%", BASELINE_X2 + 0.5, VERT_SHIFT + FLX_BOT_Y1 + 0.25 * FLX_ZONE_WIDTH, 2.5);
		gScreen.print("75%", BASELINE_X2 + 0.5, VERT_SHIFT + FLX_BOT_Y1 + 0.75 * FLX_ZONE_WIDTH, 2.5);
		gScreen.print("0%", BASELINE_X2 + 0.5, VERT_SHIFT + FLX_BOT_Y1, 2.5);

	}


	if (gs.showForces == 1) {
		for (i = 0; i < 5; i++) {
			diffForce[i] = fGain[i] * gBox.getForce(i);
		}
		// Finger forces (difference -> force = f_ext - f_flex)
		gScreen.setColor(Screen::red);
		gScreen.drawLine(-X_CURSOR_DEV - FINGWIDTH / 2,
			VERT_SHIFT + forceGain * diffForce[1] + baselineCorrection,
			-X_CURSOR_DEV + FINGWIDTH / 2,
			VERT_SHIFT + forceGain * diffForce[1] + baselineCorrection);
		gScreen.drawLine(X_CURSOR_DEV - FINGWIDTH / 2,
			VERT_SHIFT + forceGain * diffForce[3] + baselineCorrection,
			X_CURSOR_DEV + FINGWIDTH / 2,
			VERT_SHIFT + forceGain * diffForce[3] + baselineCorrection);

	}

	if (gs.showFeedback) { // show feedback
		gScreen.setColor(Screen::red);
		gScreen.drawLine(-X_CURSOR_DEV - FINGWIDTH / 2,
			Favf[0] / 1250,
			-X_CURSOR_DEV + FINGWIDTH / 2,
			Favf[0] / 1250);
		gScreen.drawLine(X_CURSOR_DEV - FINGWIDTH / 2,
			Favf[1] / 1250,
			X_CURSOR_DEV + FINGWIDTH / 2,
			Favf[1] / 1250);
	}

	if (gs.showDiagnostics) {
		string stateString;

		//if (trainF == 0) {
		switch (state)
		{
		case WAIT_TRIAL:
			stateString = "Wait Trial";
			break;
		case START_TRIAL:
			stateString = "Start Trial";
			break;
		case WAIT_TR:
			stateString = "Wait TR";
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
	gBox.update();
	//gBox[1].update();
	/// Call the Trial for control 
	currentTrial->control();

	/// record the data at record frequency 
	if (dataman.isRecording()) {
		bool x = dataman.record(DataRecord(state, gExp->theBlock->trialNum));
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
	int fi[2] = { finger[0], finger[1] };

	gs.showDiagnostics = 0;

	switch (state) {
	case WAIT_TRIAL: //0
		gs.showTgLines = 1;	// set screen lines/force bars to show
		gs.showBsLines = 1;
		gs.showPrLines = 0;
		gs.showForces = 1;
		gs.showFeedback = 0;
		if (showCue == 0) {
			gs.showTarget = 0;
			gs.showPrLines = 0;
		}
		else {
			gs.showTarget = 1;
			gs.showPrLines = 1;
			cueID = probCue;
		}
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

		break;

	case START_TRIAL: //1	e
		gs.showTgLines = 1;	// set screen lines/force bars to show
		gs.showBsLines = 1;
		gs.showFeedback = 0;
		gs.showForces = 1;
		gs.showTimer5 = 0;
		// gs.showForceBars = 1;
		gs.boxColor = 5;	// grey baseline box color
		gs.planError = 0;
		gs.chordError = 0;
		planErrorFlag = 0;	// initialize planErrorFlag variable in the begining of each trial
		chordErrorFlag = 1;	// initialize chordErrorFlag variable in the begining of each trial
		//startTriggerEMG = 1;	// Ali EMG: starts EMG trigger in the beginning of each trial

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

		state = WAIT_TR;
		break;

	case WAIT_TR: //2

		gs.showTgLines = 1;	// set screen lines/force bars to show
		gs.showBsLines = 1;
		gs.showFxCross = 1;
		gs.showForces = 0;
		gs.showTarget = 0;
		gs.showFeedback = 0;

		if (gCounter.readTR() > 0 && gCounter.readTotTime() >= startTime) {
			startTimeReal = gCounter.readTotTime();
			startTRReal = gCounter.readTR();

			//dataman.startRecording(); // see around line #660
			gTimer.reset(1);					//time for whole trial
			gTimer.reset(2);					//time for events in the trial			
			gTimer.reset(3);
			state = WAIT_PLAN;
		}
		break;;




	case WAIT_PLAN: //3
		gs.showTimer5 = 0;
		gs.showForces = 1;
		gs.showFxCross = 1;

		for (i = 0; i < 2; i++) {	// check fingers' states -> fingers should stay in the baseline during planning
			fingerForceTmp = VERT_SHIFT + forceGain * fGain[fi[i]] * gBox.getForce(fi[i]) + baselineCorrection;
			check_baseline_hold = 1;
			if (fingerForceTmp >= (VERT_SHIFT + baseTHhi) || fingerForceTmp <= (VERT_SHIFT - (baseTHhi))) {
				// if even one finger was out of baseline zone, reset timer(1):
				//gTimer.reset(3); // do not reset timer here or imaging runs will have different length
				check_baseline_hold = 0;
				break;
			}
		}

		/// Because of this wait time, total trial duration in .mov is 500 ms longer 
		/// than in the .tgt file and stim occurs 500 ms after planTime
		if (gTimer[3] > baseline_wait_time) {	// turn on visual target after 500ms of holding the baseline
			gs.showFxCross = 0;
			gs.showTarget = 1;	// show visual target	
			gs.showPrLines = 1;
		}

		else {
			gs.showTarget = 0;
			gs.showPrLines = 0;
		}

		if (check_baseline_hold == 0) {
			gs.boxColor = 3;	// baseline zone color becomes red
		}
		else {
			gs.boxColor = 5;	// baseline zone color becomes grey
		}

		// if subjects holds the baseline zone for plan time after visual cue was shown go to execution state:
		if (gTimer[3] > planTime + baseline_wait_time) {
			state = WAIT_EXEC;
			gTimer.reset(2);	// resetting timer 2 to use in next state
			gTimer.reset(3);	// resetting timer 3 to use in next state
			gTimer.reset(5);	// resetting timer 4 to use in next state

		}

		break;

	case WAIT_EXEC: //3


		// deliver finger perturbation
		for (i = 0; i < 5; i++) {
			if (stimFinger[i] == '1')
			{
				gVolts[i] = fingerVolt; // set the volts to be sent to the Pneumatic Box
			}

		}

		if (GoNogo == "go") {

			gBox.setVolts(5,
				gVolts[1],
				0,
				gVolts[3],
				0);

			gs.showTgLines = 1;	// set screen lines/force bars to show
			gs.showPrLines = 0;
			gs.showBsLines = 1;
			gs.showForces = 0;
			gs.showFxCross = 1;
			gs.showTarget = 0;		// show the targets on the screen (grey bars)
			gs.boxColor = 5;		// grey baseline box color

		}

		else if (GoNogo == "nogo") {
			gs.showTgLines = 1;	// set screen lines/force bars to show
			gs.showPrLines = 0;
			gs.showBsLines = 1;
			gs.showForces = 0;
			gs.showFxCross = 1;
			gs.showTarget = 0;		// show the targets on the screen (grey bars)
			gs.boxColor = 5;		// grey baseline box color
			state = ACQUIRE_HRF;
		}
		

		if (gTimer[3] > 500) {
			for (i = 0; i < 2; i++) {	
				fingerForceTmp = VERT_SHIFT + forceGain * fGain[fi[i]] * gBox.getForce(fi[i]) + baselineCorrection;
				Favf[i] = (Favf[i] + fingerForceTmp);
			}
		}

		// If subject runs out of time:
		if (gTimer[3] >= execMaxTime) {
			//chordErrorFlag = 1;
			//RT = 10000;
			state = GIVE_FEEDBACK;
			gTimer.reset(2);
			gTimer.reset(3);

		}
		break;

	case GIVE_FEEDBACK: //4
		//SetDacVoltage(0, 0); // Ali EMG
		SetDIOState(0, 0xFFFF);

		// end finger perturbation
		gVolts[1] = 0;
		gVolts[3] = 0;
		gBox.setVolts(0, 0, 0, 0, 0);

		gs.showTgLines = 1;	// set screen lines/force bars to show
		gs.showPrLines = 0;
		gs.showBsLines = 1;
		gs.showForces = 0;
		gs.showFxCross = 0;
		gs.showTarget = 0;			// no visual targets
		gs.showTimer5 = 0;
		gs.showFeedback = 1;		// showing feedback (refer to MyTrial::updateGraphics() for details)

		if (gTimer[2] >= feedbackTime) {
			state = ACQUIRE_HRF;
			gTimer.reset(2);
		}
		break;

	case ACQUIRE_HRF:

		if (gExp->theBlock->trialNum + 1 != gExp->theBlock->numTrials)
		{
			state = WAIT_ITI;
		}
		else
		{
			gs.showTgLines = 1;	// set screen lines/force bars to show
			gs.showBsLines = 1;
			gs.showFxCross = 1;
			gs.showForces = 0;
			gs.showTarget = 0;
			gs.showFeedback = 0;
			if (gTimer[2] > 12000) {  // wait 12 s at the end of the run
				state = END_TRIAL;
				gTimer.reset(2);
				cout << "HRF acquired for 10 seconds after trial" << endl;
			}
		}

	case WAIT_ITI:
		gs.showTgLines = 1;	// set screen lines/force bars to show
		gs.showBsLines = 1;
		gs.showFxCross = 1;
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
	//}

	//else if (maxF == 1) {

	//	forceFinger = finger[0];
	//	currentForce = gBox.getForce(forceFinger);
	//	if (currentForce > maxForce[forceFinger]) {
	//		maxForce[forceFinger] = currentForce;
	//	}
	//	fGain[forceFinger] = abs(baselineCorrection) / (bsForce * maxForce[forceFinger]);

	//	forceFinger = finger[1];
	//	currentForce = gBox.getForce(forceFinger);
	//	if (currentForce > maxForce[forceFinger]) {
	//		maxForce[forceFinger] = currentForce;
	//	}
	//	fGain[forceFinger] = abs(baselineCorrection) / (bsForce * maxForce[forceFinger]);
	//}

}

/////////////////////////////////////////////////////////////////////////////////////
/// Data Record: creator records the current data from the device 
/////////////////////////////////////////////////////////////////////////////////////
DataRecord::DataRecord(int s, int t) {
	int i;
	int fi[2] = { finger[0], finger[1] };
	state = s;
	trialNum = t;
	time = gTimer[1];
	timeReal = gTimer.getRealtime();

	TotTime = gCounter.readTotTime(); //internally generated time initiated at first TTL pulse
	TR = gCounter.readTR(); //counted TR pulse
	TRtime = gCounter.readTime(); //time since last TR
	currentSlice = gCounter.readSlice();


	for (i = 0; i < 5; i++) {
		fforce[i] = gBox.getForce(i);
	}

	for (i = 0; i < 2; i++) {
		visualizedForce[i] = VERT_SHIFT + forceGain * fGain[fi[i]] * gBox.getForce(fi[i]) + baselineCorrection;	// The position of the force bars that are shown on the screen
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream& out) {
	int i;

	out << trialNum + 1 << "\t"
		<< state << "\t"
		<< timeReal << "\t"
		<< time << "\t"
		<< TotTime << "\t"
		<< TR << "\t"
		<< TRtime << "\t"
		<< currentSlice << "\t";

	for (i = 0; i < 5; i++) {
		out << fforce[i] << "\t";
	}

	for (i = 0; i < 2; i++) {	// Position of visualized force bars
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
	lineYpos[0] = 0;			// feedback 	
	lineColor[0] = 2;			// white 
	size[0] = 5;

	//// RT 
	//lineXpos[1] = 0;
	//lineYpos[1] = 5;			// feedback 	
	//lineColor[1] = 1;			// white 
	//size[1] = 5;

	//// total points 
	//lineXpos[2] = 0;
	//lineYpos[2] = 4;			// block points	
	//lineColor[2] = 1;			// white 
	//size[2] = 5;

	showTgLines = true;
	showBsLines = true;
	showPrLines = true;
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