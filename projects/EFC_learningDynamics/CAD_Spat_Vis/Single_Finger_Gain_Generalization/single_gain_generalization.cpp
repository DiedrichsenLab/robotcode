///////////////////////////////////////////////////////////////
/// Single Finger Gain Generalization - Amin Nazerzadeh Ali Ghavampour August 2026
///////////////////////////////////////////////////////////////
#include "single_gain_generalization.h" 
#include "StimulatorBox.h"
#include "Vector2d.h"

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

ForceCursor forceCursor[5];

///< Basic imaging parameters
#define TRTIME 1000				///< timer for simulating timer
#define FEEDBACKTIME 1000;		///< duration of n points feedback on screen


///< Screen graphics defenitions
#define baseTH  0.5		// Baseline threshold (to check for premature movements during sequence planning phase)
// double fGain[5] = { 1.0,1.0,1.0,1.0, 1.0 };	// finger specific force gains -> applied on each finger
double forceGain = 1;						// universal force gain -> applied on all the fingers
bool blockFeedbackFlag = 0;

#define FINGWIDTH 1.3
#define N_FINGERS 5
#define FINGER_SPACING 0.2
#define BASELINE_X1 -(FINGWIDTH*N_FINGERS/2)
#define BASELINE_X2 +(FINGWIDTH*N_FINGERS/2)

#define FLX_ZONE_WIDTH 0.5


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
char gKey;
bool gKeyPressed;
double gTargetWidth = 0.25;
double execAccTime = 400; // in ms, window around the fourth tone
double execSampleTime = 500; // time point at which the execution is sampled after the go cue (for calculating points and giving feedback)
double SAMPLING_DURATION = 50;  /// duration of the window for sampling generated forces
double beepInterval = 800; // interval between the beeps
double relaxTime = 2000; // time after execution to relax and zero the forces before next trial starts

///////////
////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the fingerBox and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	// 1. initialization window, text display and screen
	gThisInst = hThisInst;
	gExp = new MyExperiment("single_gain_generalization", "single_gain_generalization", "C:/data/EFC_learningDynamics/CAD_Spat_Vis/Single_Finger_Gain_Generalization/"); 
	//gExp->redirectIOToConsole();
	
	// gExp->redirectIOToConsole();		// I uncommented this!!!
	tDisp.init(gThisInst, 0, 0, 600, 20, 9, 2, &(::parseCommand));		// Default setting for the Windows 10 PC
	tDisp.setText("Subj", 0, 0);
	gScreen.init(gThisInst, 1920, 0, 1440, 900, &(::updateGraphics));	// Default setting for the Windows 10 PC
	//gScreen.init(gThisInst, 1920, 0, 1680, 1050, &(::updateGraphics)); ///< Display for subject (for setups at 3128 and sensorimotor room, by the window)
	//gScreen.init(gThisInst, 1920, 0, 1920, 1050, &(::updateGraphics)); ///< Display for subject (for setups at 3128 and sensorimotor room, by the window)


	gScreen.setCenter(Vector2D(0, 0));									// In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));					// cm/pixel

	// 2. initalize s626cards 
	s626.init("c:/robotcode/calib/s626_single.txt");
	if (s626.getErrorState() == 0) {
		cout << "Initializing S626 Card" << endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); // initialize at 200 Hz update rate 
	}
	gTimer.init(); // Ali Changed Here!!!!

	for (size_t i = 0; i < 5; ++i) {
		forceCursor[i].size = Vector2D(FINGWIDTH - FINGER_SPACING * 2, FINGWIDTH - FINGER_SPACING * 2);
		forceCursor[i].setColor(SCR_RED);
	}

	// 3. stimulation box initialization and calibration
	// high force 1
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/Flatbox1_highforce_LEFT_07-Jun-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/Flatbox1_highforce_RIGHT_31-July-2017.txt");

	// STARK
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/Flatbox1_highforce2_LEFT_12-Feb-2022.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/Flatbox1_highforce2_RIGHT_03-Dec-2021.txt");

	// CHOMSKY
	//gBox[0].init(BOX_LEFT, "c:/robot/calib/LEFT_lowForce_FlatBox2_24-Jan-2018.txt");
	//gBox[1].init(BOX_RIGHT, "c:/robot/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");

	gBox[0].init(BOX_LEFT,"c:/robotcode/calib/Flatbox1_highforce2_LEFT_12-Feb-2022.txt");
	//gBox[0].init(BOX_LEFT,"c:/robotcode/calib/Flatbox1_highforce2_LEFT_12-Aug-2026-Amin-Single-Gain.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robotcode/calib/Flatbox1_highforce2_RIGHT_03-Dec-2021.txt");

	//low force Flatbox3
	gBox[1].init(BOX_RIGHT, "c:/robotcode/calib/Flatbox3_lowforce_RIGHT_22_Aug_2024.txt");
	//gBox[1].init(BOX_RIGHT, "c:/robotcode/calib/Flatbox3_lowforce_RIGHT_12-Aug-2026-Amin_Single-Gain.txt");


	// low force
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/flatbox2_lowforce_LEFT_03-Mar-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");

	//gBox[0].filterconst = 0.8;
	//gBox[1].filterconst = 0.8;

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

	///// set individual finger force gain. You can set any arbitrary force gain for every participant if they cant do the chord.
	//else if (arguments[0] == "setFingerGain") {
	//	if (numArgs != 6) {
	//		tDisp.print("USAGE: setFingerGain <gain1> <gain2> ... <gain5>");
	//	}
	//	else {
	//		for (i = 0; i < 5; i++) {
	//			sscanf(arguments[i+1].c_str(), "%f", &arg[0]);
	//			fGain[i] = arg[0];
	//		}
	//	}
	//}
	
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

	else if (arguments[0] == "execAccTime") {
		if (numArgs != 2) {
			tDisp.print("USAGE: execAccTime <time in milliseconds>");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			execAccTime = arg[0];
			
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
	blockFeedbackFlag = 1;

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
	///< INIT TRIAL VARIABLE
	trialCorr = 0;		// flag for tiral being correct or incorrect -> 0: trial error , 1: trial correct
	trialErrorType = 0;	// flag for the type of trial error -> 0: no error , 1: planning error , 2: execution error
	purturbation1 = 0;
	purturbation2 = 0;
}

double MyTrial::trialPerturbation(int fingerIndex) const {
	// purturbation1 = index (finger 2), purturbation2 = ring (finger 4)
	if (fingerIndex == 1) return purturbation1;
	if (fingerIndex == 3) return purturbation2;
	return 0.0;
}

///////////////////////////////////////////////////////////////
// Read - Done
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	in >> subNum
		>> targetForces[0]
		>> targetForces[1]
		>> targetForces[2]
		>> targetForces[3]
		>> targetForces[4]
		>> isTargetVisible
		>> purturbation1
		>> purturbation2
		>> planTime
		>> feedbackTime
		>> iti;
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	// write to .dat file
	out << subNum << "\t"
		<< targetForces[0] << "\t"
		<< targetForces[1] << "\t"
		<< targetForces[2] << "\t"
		<< targetForces[3] << "\t"
		<< targetForces[4] << "\t"
		<< endForces[0] << "\t"
		<< endForces[1] << "\t"
		<< endForces[2] << "\t"
		<< endForces[3] << "\t"
		<< endForces[4] << "\t"
		<< endForcesPurturbed[0] << "\t"
		<< endForcesPurturbed[1] << "\t"
		<< endForcesPurturbed[2] << "\t"
		<< endForcesPurturbed[3] << "\t"
		<< endForcesPurturbed[4] << "\t"
		<< isTargetVisible << "\t"
		<< purturbation1 << "\t"
		<< purturbation2 << "\t"
		<< planTime << "\t" 
		<< feedbackTime << "\t" 
		<< iti << "\t" 
		<< forceGain << "\t"					// Global force gain for all fingers
		<< trialCorr << "\t"					// trial is correct or not
		<< trialErrorType << "\t"				// trial error type
		<< endl;
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) {
	char header[200];
	out << "subNum" << "\t" 
		<< "targetForce1" << "\t"
		<< "targetForce2" << "\t"
		<< "targetForce3" << "\t"
		<< "targetForce4" << "\t"
		<< "targetForce5" << "\t"
		<< "endForce1" << "\t"
		<< "endForce2" << "\t"
		<< "endForce3" << "\t"
		<< "endForce4" << "\t"
		<< "endForce5" << "\t"
		<< "endForcePurturbed1" << "\t"
		<< "endForcePurturbed2" << "\t"
		<< "endForcePurturbed3" << "\t"
		<< "endForcePurturbed4" << "\t"
		<< "endForcePurturbed5" << "\t"
		<< "isTargetVisible" << "\t"
		<< "purturbation1" << "\t"
		<< "purturbation2" << "\t"
		<< "planTime" << "\t" 
		<< "feedbackTime" << "\t" 
		<< "iti" << "\t" 
		<< "forceGain" << "\t"
		<< "trialCorr" << "\t"
		<< "trialErrorType" << "\t"
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
	sprintf(buffer, "GlobalGain = %1.1f", forceGain);
	tDisp.setText(buffer, 10, 0);
}

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD 
///////////////////////////////////////////////////////////////


void MyTrial::updateGraphics(int what) {
	int i;
	double x1,x2,xPos,yPos,xSize,ySize, targetForce;
	double diffForce[5] = { 0,0,0,0,0 };
	
	if (blockFeedbackFlag) {
		gScreen.setCenter(Vector2D(0, 0));    // In cm //0,2
		gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
	}


	if (gs.showLines == 1) {
		// Baseline box
		gScreen.setColor(myColor[gs.boxColor]);
		gScreen.drawBox(FINGWIDTH * N_FINGERS, (baseTH)*2, 0, VERT_SHIFT);

		// Baseline lines
		gScreen.setColor(Screen::grey);
		gScreen.drawLine(BASELINE_X1 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT + baseTH, BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT + baseTH);
		gScreen.drawLine(BASELINE_X1 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT -(baseTH), BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), VERT_SHIFT -(baseTH));

	}

	if (gs.showTarget == 1) {
		for (i = 0; i < 5; i++) {
			targetForce = gs.targetForces[i];
			if (targetForce != 0) {
				x1 = ((i * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) + FINGER_SPACING;
				x2 = (((i + 1) * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) - FINGER_SPACING;
				xPos = (x1 + x2) * 0.5;
				xSize = x2 - x1;
				ySize = FLX_ZONE_WIDTH;
				yPos = targetForce + VERT_SHIFT;
				gScreen.setColor(myColor[5]);
				gScreen.drawBox(xSize, ySize, xPos, yPos);
			}
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

	if (gs.showForces) {
		for (i = 0; i < 5; i++) {
			targetForce = gs.targetForces[i];
			if (targetForce != 0) {
				if (gs.isFrozenForces){
					diffForce[i] = (gs.frozenExtForces[i] - gs.frozenFlexForces[i]);
				}
				else{
					diffForce[i] = (gBox[0].getForce(i) - gBox[1].getForce(i));
					}	
				forceCursor[i].position[0] = (((2 * i + 1) * FINGWIDTH) - (FINGWIDTH * N_FINGERS)) / 2.0;;
				forceCursor[i].position[1] = VERT_SHIFT + forceGain * diffForce[i];

				forceCursor[i].draw();
			}

		}
	}

	if (gs.showFeedback) {
		gScreen.setColor(Screen::white);

		if (gs.earlyMovError) {
			gScreen.print("Early!", 0, 3, 7);
		}
		else if (gs.lateMovError) {
			gScreen.print("Late!", 0, 3, 7);
		}
		else {
			sprintf(buffer, "");
			gScreen.print(buffer, 0, 3, 7);
		}
	}

	if (gs.showRelax) {
		gScreen.setColor(Screen::white);
		sprintf(buffer, "Relax!");
		gScreen.print(buffer, 0, 3, 7);

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
		case RELAX:
			stateString = "Relax";
			break;
		case WAIT_ITI:
			stateString = "Wait ITI";
			break;
		case END_TRIAL:
			stateString = "End Trial";
			break;
		}
		gScreen.setColor(Screen::white);
		gScreen.print(stateString, 0, 12, 5);
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
		bool x = dataman.record(DataRecord(state));
		if (!x) {
			dataman.stopRecording();
		}
	}
}

//////////////////////////////////////////////////////////////////////
// control Trial: A state-driven routine to guide through the process of a trial
//////////////////////////////////////////////////////////////////////
bool earlyMovFlag = 0;		// flag for checking if early movement.
bool lateMovFlag = 0;     // flag for checking if late movement.
double volts[2][5] = { {0,0,0,0,0},{0,0,0,0,0} }; // variable to store the volt values for zeroing the force boxes.
double forceTemps[5] = { 0,0,0,0,0 }; // temporary variable to store the force values for smoothing.
double extForceTemps[5] = { 0,0,0,0,0 }; // temporary variable to store the extension force values for smoothing.
double flexForceTemps[5] = { 0,0,0,0,0 }; // temporary variable to store the flexion force values for smoothing.

// counter for fingers and box
int i;
int b;
int j;

int zeroFCounter; // counter for zeroing the force boxes
int samplingCounter; // counter for sampling the generated force in trial

void MyTrial::control() {
	double fingerForceTmp;
	double targetForceTmp;
	// PROBLEM WAS HERE: YOU CAN'T SET VARIABLES TO 0 HERE. THEY WILL ALWAYS REMAIN 0. THE CONTROL() IS CALLED EVERY SINGLE UPDATE RATE.
	// boold check_last_beep_done = 0; MOVED TO .h file. IT'S BETTER THERE.
	switch (state) {
	case WAIT_TRIAL: //0
		gs.showLines = 1;	// set screen lines/force bars to show
		gs.showFeedback = 0;
		gs.showTarget = 0;
		gs.showForces = 1;
		gs.showDiagnostics = 0;

		gs.earlyMovError = 0;
		gs.lateMovError = 0;
		gs.boxColor = 5;	// grey baseline box color
		earlyMovFlag = 0;
		lateMovFlag = 0;


		for (i = 0; i < NUMDISPLAYLINES; i++) {
			if (!gs.line[i].empty()) {
				gs.lineColor[i] = 0;
				gs.line[i] = "";
			}
		}

		for (i = 0; i < 5; i++) {
			forceTemps[i] = 0;
			extForceTemps[i] = 0;
			flexForceTemps[i] = 0;
		}
		
		zeroFCounter = 0; // counter for zeroing the force boxes

		samplingCounter = 0; // counter for sampling the generated force in trial

		//gs.chord = chord;

		break;

	case START_TRIAL: //1
		// gs.targetForce = targetForce;
		for (i = 0; i < 5; i++){
			gs.targetForces[i] = targetForces[i];
		}

		check_last_beep_done = 0;
		check_mov_initiation = 0;

		//Amin
		gs.showLines = 1;	// set screen lines/force bars to show
		gs.showFeedback = 0;
		gs.showTarget = 0;
		gs.showForces = 1;
		gs.showDiagnostics = 0;
		gs.earlyMovError = 0;
		gs.lateMovError = 0;
		gs.boxColor = 5;	// grey baseline box color
		earlyMovFlag = 0;
		lateMovFlag = 0;

		for (i = 0; i < 5; i++) {
			forceTemps[i] = 0;
			extForceTemps[i] = 0;
			flexForceTemps[i] = 0;
		}

		zeroFCounter = 0; // counter for zeroing the force boxes

		samplingCounter = 0; // counter for sampling the generated force in trial

		for (b = 0; b < 2; b++) {
			for (j = 0; j < 5; j++) {
				volts[b][j] = 0;
			}
		}

		for (i = 0; i < 5; i++) {
			gs.fingerCorrectGraphic[i] = 0; // Amin: check
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

		// ring the first sound 
		PlaySound(TASKSOUNDS[0].c_str(), NULL, SND_ASYNC);

		state = WAIT_PLAN;
		break;
	
	case WAIT_PLAN: //2

		gs.showTimer5 = 0;

		// gTimer[2] is used to time the rings
		if (gTimer[2] >= beepInterval){
			// cout << "sound" << endl;
			PlaySound(TASKSOUNDS[0].c_str(), NULL, SND_ASYNC);
			gTimer.reset(2);
		}
		
		if (gTimer[3] >= beepInterval * 3 - planTime) {	// turn on visual target //Amin: check
			gs.showTarget = 1;	// show visual target
		}
		else {
			gs.showTarget = 0;	// dont show visual target
		}

		for (i = 0; i < 5; i++){
			if (targetForces[i] == 0) continue;
			fingerForceTmp = VERT_SHIFT + forceGain * (gBox[0].getForce(i) - gBox[1].getForce(i));
			if (fingerForceTmp >= (VERT_SHIFT + baseTH) || fingerForceTmp <= (VERT_SHIFT - (baseTH))) {
				earlyMovFlag = 1; //early movement
				gs.showForces = 0; // hide forces if subject goes out of baseline zone
				state = GIVE_FEEDBACK;
				break;
			}
		}

		// if (earlyMovFlag) {
		// 	gs.boxColor = 3;	// baseline box becomes red
		// 	state = GIVE_FEEDBACK;
		// }

		if (gTimer[1] >= (beepInterval * 3 - execAccTime/2 )) {
			state = WAIT_EXEC;
			// gTimer.reset(2);	// resetting timer 2 to use in next state
			gTimer.reset(3);	// resetting timer 3 to use in next state
			gTimer.reset(5);	// resetting timer 5 to use in next state
		}
		break;
		
	case WAIT_EXEC:

		
		// gTimer[2] is used to time the rings
		if ((gTimer[2] >= beepInterval) && (check_last_beep_done == 0)){
			check_last_beep_done = 1;
			PlaySound(TASKSOUNDS[0].c_str(), NULL, SND_ASYNC);
		}

		for (i = 0; i < 5; i++){
			if (targetForces[i] == 0) continue;
			fingerForceTmp = VERT_SHIFT + forceGain * (gBox[0].getForce(i) - gBox[1].getForce(i));
			if (fingerForceTmp >= (VERT_SHIFT + baseTH) || fingerForceTmp <= (VERT_SHIFT - (baseTH))) {
				gs.showForces = 0; // hide forces if subject goes out of baseline zone
				check_mov_initiation = 1; // flag to check if movement was initiated
				break;
			}
		}


		if (gTimer[1] >= (beepInterval * 3 + execAccTime/2)) { // NEEDS FIXING
			if (check_mov_initiation == 0) {	// if movement was not initiated until the time of the last beep, consider it as an execution error
				lateMovFlag = 1;
				gs.showForces = 0; // hide forces if subject was late
				state = GIVE_FEEDBACK;
			}
		}

		// time window for sampling the generated forces to give feedback based on them
		if ((beepInterval * 3 + execSampleTime - SAMPLING_DURATION/2) <= gTimer[1] && gTimer[1] < (beepInterval * 3 + execSampleTime + SAMPLING_DURATION/2)) { // NEEDS FIXING
			for (i = 0; i < 5; i++) {
				forceTemps[i] += VERT_SHIFT + forceGain * (gBox[0].getForce(i) - gBox[1].getForce(i));
				extForceTemps[i] += gBox[0].getForce(i);
				flexForceTemps[i] += gBox[1].getForce(i);
			}
			samplingCounter++;
		}

		// Calculating points based on the sampled forces during the smoothing window
		if (gTimer[1] >= (beepInterval * 3 + execSampleTime + SAMPLING_DURATION / 2)) { // NEEDS FIXING

			for (i = 0; i < 5; i++) {
				endForces[i] = forceTemps[i] / samplingCounter; // average generated force for each finger during the sampling window
				double p = (targetForces[i] != 0) ? trialPerturbation(i) : 0.0;
				endForcesPurturbed[i] = endForces[i] + p;
				FinalExtForces[i] = extForceTemps[i] / samplingCounter;
				FinalFlexForces[i] = flexForceTemps[i] / samplingCounter;

				gs.frozenExtForces[i] = FinalExtForces[i];
				gs.frozenFlexForces[i] = FinalFlexForces[i] - p; // positive purturbation: less flexion force
				gs.isFrozenForces = 1;
			}

			state = GIVE_FEEDBACK;

			// resetting timers:
			gTimer.reset(2);
			gTimer.reset(3);
		}
		
		break;

	case GIVE_FEEDBACK:
	
		gs.showLines = 1;			// no force lines/thresholds
		gs.showTarget = 1;
		gs.showFeedback = 1;		// showing feedback (refer to MyTrial::updateGraphics() for details)

		if (earlyMovFlag == 1) {	// if early movement occurred during planning
			gs.earlyMovError = 1;		// flag to show "moved early" message in the feedback
			trialCorr = 0;
			trialErrorType = 1;		// trial error type saved in the .dat file to know this was an early movement error
			}
		else if (lateMovFlag) {	// if movement didn't initiate on time
			gs.lateMovError = 1;		// flag to show "moved late" message in the feedback
			trialCorr = 0;
			trialErrorType = 2; // trial error type saved in the .dat file to know this was a late movement error		
			}
		else {
			trialCorr = 1;
			trialErrorType = 0;
			if (isTargetVisible) {
				gs.showForces = 1; // show frozen forces during feedback
			}
			else {
				gs.showForces = 0; // don't show frozen forces during feedback
			}
			}
		
		if (gTimer[2] >= feedbackTime) {
			state = RELAX;
			gTimer.reset(2);
		}
		break;

	case RELAX:
		gs.showForces = 0;
		gs.showTarget = 0;
		gs.showFeedback = 0;
		gs.showRelax = 1;
		gs.isFrozenForces = 0;

		if (gTimer[2] >= relaxTime/2){
			
			for (b = 0; b < 2; b++){
				for (j = 0; j < 5; j++){
					volts[b][j] += gBox[b].getVolts(j);
				}
			}
			zeroFCounter += 1;

			if (gTimer[2] >= relaxTime) { // zero the force boxes after 1 second of relax time
				for (b = 0; b < 2; b++) {
					for (j = 0; j < 5; j++) {
						volts[b][j] /= zeroFCounter;
					}
					gBox[b].zeroForce(volts[b]);
				}
				state = WAIT_ITI;
				gTimer.reset(2);
			}
		}
		break;

	case WAIT_ITI:
		gs.showLines = 1;
		gs.showForces = 1;
		gs.showRelax = 0;

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
	int i,j;
	state = s;
	time = gTimer[1];
	timeReal = gTimer.getRealtime();

	for (i = 0; i < 2; i++) {
		for (j = 0; j < 5; j++) {
			fforce[i][j] = gBox[i].getForce(j);
		}
	}
	//// Amin
	// for (i = 0; i < 5; i++) {
	// 	diffForceMov[i] = (gBox[0].getForce(i) - gBox[1].getForce(i));	// diffForceMov = f_ext - f_flex
	// 	visualizedForce[i] = VERT_SHIFT + forceGain * (gBox[0].getForce(i) - gBox[1].getForce(i));	// The position of the force bars that are shown on the screen
	// }
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
	// for (i = 0; i < 5; i++) {	// Differential forces -> diffForceMov = extension force - flexion force
	// 	out << diffForceMov[i] << "\t";
	// }
	// for (i = 0; i < 5; i++) {	// Position of visualized force bars
	// 	out << visualizedForce[i] << "\t";
	// }
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

