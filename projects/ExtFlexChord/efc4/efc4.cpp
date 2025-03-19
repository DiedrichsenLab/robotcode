///////////////////////////////////////////////////////////////
/// ExtensionFlextionChord - Ali Ghavampour , Nov 2022
///////////////////////////////////////////////////////////////
#include "efc4.h" 
#include "StimulatorBox.h"
#include "Vector2d.h"
#include <vector>
#include <iostream>
#include <numeric>

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
bool startTriggerEMG = 0; // Ali added this: experimental - under construction
float emgTrigVolt = 2;	// Ali added this: experimental - under construction

Matrix2D 	TransforMatrix(1, 0, 0, 1);	///< adjusts for the fact that subject screen is flipped. used in angles (0,1,1,0)

bool chordStarted = 0;

double t1;

FixCross fixationCross;

ForceCursor forceCursor[5];
//ForceCursor forceCursor2;
//ForceCursor forceCursor3;
//ForceCursor forceCursor4;
//ForceCursor forceCursor5;

//std::vector<std::vector<double>> X;
//std::vector<double> fingerForceTmp5(5, 0.0);

int holdTime = 0;
//int totSuccess = 0;

///< Basic imaging parameters
#define TRTIME 1000				///< timer for simulating timer
//#define HOLDTIME 1000			///< timer for holding key press
#define MOVETHRESHOLD 0.8		///< above this force, finger detected as moving
#define RELEASETHRESHOLD 0.6	///< below this force, finger detected as released
#define FEEDBACKTIME 1000;		///< duration of n points feedback on screen
#define NOGOTIME 3000;			///< Duration if the cue is no-go
char counterSignal = '5';		///< What char is used to count the TR
//int sliceNumber = 32;			///< How many slices do we have

const char* sPoints;

///< Screen graphics defenitions
#define baseTHhi  1.2 //0.8//1.0			// Baseline higher threshold (to check for premature movements during sequence planning phase)
double fGain[5] = { 1.0,1.0,1.0,1.5,1.5 };	// finger specific force gains -> applied on each finger
double forceGain = 1;						// universal force gain -> applied on all the fingers
double mahdiyar = 123;
bool blockFeedbackFlag = 0;
bool wait_baseline_zone = 1;				// if 1, waits until the subject's fingers are all in the baseline zone. MARCO CHANGED TO 1

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

#define CROSSW 0.6 // adjust to visual angle based on the distance in the scanner
#define CROSSP (FLX_BOT_Y1 + FLX_TOP_Y1) / 2

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
double gErrors[2][5] = { {0,0,0,0,0},{0,0,0,0,0} };
//double execAccTime = 600;


///////////
////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the fingerBox and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	// 1. initialization window, text display and screen
	gThisInst = hThisInst;
	gExp = new MyExperiment("efc4", "efc4", "C:/data/ExtFlexChord/efc4/"); // Marco chmaged to efc2 here
	//gExp->redirectIOToConsole();

	gExp->redirectIOToConsole();		// I uncommented this!!!
	tDisp.init(gThisInst, 0, 0, 800, 30, 9, 2, &(::parseCommand));		
	tDisp.setText("Subj", 0, 0);
	//gScreen.init(gThisInst, 1920, 0, 1440, 900, &(::updateGraphics));	// Default setting for the Windows 10 PC (Behav training/testing)
	//gScreen.init(gThisInst, 1280, 0, 1024, 768, &(::updateGraphics)); // Default setting for the 7T control room
	gScreen.init(gThisInst, 1920, 0, 1680, 1080, &(::updateGraphics)); // Default setting for the Windows 10 PC (Mock scanner)
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
		forceCursor[i].size = Vector2D(FINGWIDTH - FINGER_SPACING *2, FINGWIDTH - FINGER_SPACING*2);
		forceCursor[i].setColor(SCR_RED);
	}

	gTimer.init(); // Ali Changed Here!!!!

	gBox[0].init(BOX_LEFT, "c:/robotcode/calib/Flatbox1_highforce2_LEFT_12-Feb-2022.txt");
	gBox[1].init(BOX_RIGHT, "c:/robotcode/calib/Flatbox1_highforce2_RIGHT_03-Dec-2021.txt");

	gCounter.init3(3, 0, 32); // TTL pulse for counting TR

	gExp->control();

	return 0;
}

//////////////////////
//// Stuff to calculate MD online 
/////////////

// Function to calculate the Euclidean norm of a vector
double calculate_norm(const std::vector<double>& vec) {
	double sum_of_squares = 0.0;
	for (double v : vec) {
		sum_of_squares += v * v;
	}
	return std::sqrt(sum_of_squares);
}

// Function to perform dot product between two vectors
double dot_product(const std::vector<double>& a, const std::vector<double>& b) {
	double result = 0.0;
	for (size_t i = 0; i < a.size(); ++i) {
		result += a[i] * b[i];
	}
	return result;
}

// Function to scale a vector by a scalar value
std::vector<double> scale_vector(const std::vector<double>& vec, double scalar) {
	std::vector<double> result(vec.size());
	for (size_t i = 0; i < vec.size(); ++i) {
		result[i] = vec[i] * scalar;
	}
	return result;
}

// Function to subtract two vectors
std::vector<double> subtract_vectors(const std::vector<double>& a, const std::vector<double>& b) {
	std::vector<double> result(a.size());
	for (size_t i = 0; i < a.size(); ++i) {
		result[i] = a[i] - b[i];
	}
	return result;
}

// Function to calculate MD and distance values
double calc_md(const std::vector<std::vector<double>>& X) {
	size_t N = X.size();
	size_t m = X[0].size();

	// Initial and final vectors
	std::vector<double> F1 = X[0];
	//std::vector<double> FN = subtract_vectors(X[N - 1], F1);  // Shift the end point
	std::vector<double> FN = X[N - 1];

	// Shift all points
	std::vector<std::vector<double>> shifted_matrix(N, std::vector<double>(m));
	for (size_t i = 0; i < N; ++i) {
		shifted_matrix[i] = subtract_vectors(X[i], F1);
	}

	double MD = 0.0;

	// Calculate distances
	for (size_t t = 1; t < N - 1; ++t) {
		std::vector<double> Ft = X[t];

		// Project Ft onto the ideal straight line
		double proj_scalar = dot_product(Ft, FN) / dot_product(FN, FN);
		std::vector<double> proj = scale_vector(FN, proj_scalar);

		// Calculate the Euclidean distance
		double d = calculate_norm(subtract_vectors(Ft, proj));

		MD += d;

		//cout << MD << endl;
	}

	MD /= (N - 2);

	return MD;
}
//
///////////////////////////////////////////////////////
//////////////////// end of MD online /////////////////
///////////////////////////////////////////////////////



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
	gs.showFxCross = 0;
	gs.showForces = 0;
	double MD;
	int max_holdTime;
	int i, j, n = 0;
	MyTrial* tpnr;
	double medianET=0;
	//double medianMD;
	double vecET[2000];
	//double vecMD[2000];
	blockFeedbackFlag = 1;

	
	// putting MD values in an array
	for (i = 0; i < 2000; i++) {
		vecET[i] = 0;
	}
	for (i = 0; i < trialNum; i++) { //check each trial
		tpnr = (MyTrial*)trialVec.at(i);
		vecET[n] = tpnr->ET + tpnr->RT;
		n++;
		//if (tpnr->trialPoint == 1) { //if trial was correct
		//	vecMD[n] = tpnr->MD;

		//		//count correct trials
		//}
	}

	// calculating the median ET and MD
	if (n > 2) {
		double dummy;
		for (i = 0; i < n - 1; i++) {
			for (j = i + 1; j < n; j++) {
				if (vecET[i] > vecET[j]) {
					dummy = vecET[i];
					vecET[i] = vecET[j];
					vecET[j] = dummy;
				}
				//if (vecMD[i] > vecMD[j]) {
				//	dummy = vecMD[i];
				//	vecMD[i] = vecMD[j];
				//	vecMD[j] = dummy;
				//}
			}
		}
		if (n % 2 == 0) {
			i = n / 2;
			medianET = ((vecET[i - 1] + vecET[i]) / 2);
			//medianMD = ((vecMD[i - 1] + vecMD[i]) / 2);
		}
		else {
			i = (n - 1) / 2;
			medianET = (vecET[i]);
			//medianMD = (vecMD[i]);
		}
	}

	// number of correct and wrong trials
	//gNumCorr = n;
	gNumWrong = trialNum - gNumCorr;
	//gNumWrong = trialNum - n;

	//gScreen.setColor(Screen::white);
	sprintf(buffer, "End of Block");
	gs.line[0] = buffer;
	gs.lineColor[0] = 1;

	sprintf(buffer, "Perc Correct = %d/%d", gNumCorr, gNumCorr + gNumWrong);
	gs.line[1] = buffer;
	gs.lineColor[1] = 1;

	//if (n > 2) {
	sprintf(buffer, "Median Execution Time = %.2f ms", medianET);
	gs.line[2] = buffer;
	gs.lineColor[2] = 1;

		/*sprintf(buffer, "Finger Synchrony = %.2f 1/N", 1 / medianMD);
		gs.line[3] = buffer;
		gs.lineColor[3] = 1;*/
	//}
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
	MD = 0;
	fixationCross.setColor(SCR_WHITE);
	
}

///////////////////////////////////////////////////////////////
// Read - Done
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	in >> subNum
		>> chordID
		>> planTime
		>> success_holdTime
		>> execMaxTime
		>> feedbackTime
		>> iti
		>> startTime
		>> endTime
		>> session
		>> day
		>> week;
		
		

}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	// write to .dat file
	out << subNum << "\t"
		<< chordID << "\t"
		<< planTime << "\t"
		<< max_holdTime <<  "\t"
		<< success_holdTime << "\t"
		<< execMaxTime << "\t"
		<< feedbackTime << "\t"
		<< startTime << "\t"
		<< startTimeReal << "\t"
		<< startTRReal << "\t"
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
		<< MD << "\t"
		<< ET << "\t"
		<< trialPoint << "\t"					// points received in each trial
		<< planError << "\t"
		<< day << "\t"
		<< week << "\t"
		<< session << "\t"
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
		<< "startTime" << "\t"
		<< "startTimeReal" << "\t"
		<< "startTRReal" << "\t"
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
		<< "MD" << "\t"
		<< "ET" << '\t'
		<< "trialPoint" << "\t"
		<< "planError" << "\t"
		<< "day" << "\t"
		<< "week" << "\t"
		<< "session" << "\t"
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

	//if (state == WAIT_EXEC || state == GIVE_FEEDBACK) {
	sprintf(buffer, "State : %d   Trial: %d    Hold time: %d    Max hold time: %d, trialPoint: %d, RT: %4.2f, ET: %4.2f, MD: %4.2f, nCorr: %d", 
		state, gExp->theBlock->trialNum, holdTime, max_holdTime, trialPoint, RT, ET, MD, gNumCorr);
	tDisp.setText(buffer, 4, 0);
	//}

	/*else {
		sprintf(buffer, "State : %d   Trial: %d    Hold time: %d    Max hold time: %d, trialPoint: %d, RT: %4.2f, ET: %4.2f, MD: %4.2f", 
			state, gExp->theBlock->trialNum, holdTime, max_holdTime, trialPoint, RT, ET, MD);
		tDisp.setText(buffer, 4, 0);
	}*/
	

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
			if (tmpChord == '0') {

			}
			else if (tmpChord == '1') {
				yPos = (FLX_TOP_Y1 + FLX_BOT_Y1) * 0.5 + VERT_SHIFT;
				if (state==WAIT_EXEC) {
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
				if (state == WAIT_EXEC) {
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
		gScreen.setColor(myColor[gs.boxColor]);
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
		if (gs.rewardTrial == 1)
			sprintf(buffer, "+1, execution time = %.2fs", ET);
			gs.line[2] = buffer;

		//if (gs.planError)
			//gScreen.print("-Moved during planning-", 0, 3, 7);
		//if (gs.chordError)
			//gScreen.print("-Chord too short-", 0, 3, 7);
	}

	if (gs.showFxCross == 1) { // show lines

		if (gs.rewardTrial == 1 /*&& state == GIVE_FEEDBACK*/) {
			fixationCross.setColor(SCR_GREEN);
		}
		else if (gs.rewardTrial == 0 /*&& state == GIVE_FEEDBACK*/) {
			fixationCross.setColor(SCR_RED);

		}
		else if (gs.rewardTrial == -1) {
			fixationCross.setColor(SCR_WHITE);
		}
		fixationCross.draw();
	}

	if (gs.showForces) {
		for (i = 0; i < 5; i++) {
			diffForce[i] = fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
		}

		for (i = 0; i < 5; i++) {

			forceCursor[i].position[0] = (((2 * i + 1) * FINGWIDTH) - (FINGWIDTH * N_FINGERS)) / 2.0;;
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
		case WAIT_TR:
			stateString = "Wait TR";
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
		case ACQUIRE_HRF:
			stateString = "Acquire HRF";
			break;
		case END_TRIAL:
			stateString = "End Trial";
			break;
		}
		gScreen.setColor(Screen::white);
		gScreen.print(stateString, -21, 12, 5);
	}

}

#include <future>  

std::future<void> mdFuture;  // Future to track async task

void MyTrial::calc_md_async() {
	mdFuture = std::async(std::launch::async, &MyTrial::calc_md, this);
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
	if (!mdFuture.valid()) {
		calc_md_async();  // Start first time
	}
	else if (mdFuture.valid() && mdFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
		calc_md_async();  // Start computation asynchronously
	}
}

void MyTrial::calc_md() {

	if (state == 5 && trialPoint == 1 && MD_done == FALSE) {
		MyBlock block;

		int trialNum = gExp->theBlock->trialNum;

		std::cout << "Calculating MD... TrialNum = " << trialNum << std::endl;

		if (DataRecord::X.find(trialNum) == DataRecord::X.end()) {
			std::cerr << "ERROR: No data for trialNum " << trialNum << std::endl;
			return;
		}

		vector<vector<double>> X = DataRecord::X[trialNum];

		if (X.empty()) {
			std::cerr << "ERROR: X[" << trialNum << "] is empty!" << std::endl;
			return;
		}

		else {
			std::cout << "X has " << X.size() << "timepoints" << trialNum << std::endl;
		}

		int max_holdTime_samples = std::round(max_holdTime / 2); // take max_holdTime in samples

		X.resize(X.size() - max_holdTime_samples); //remove holding time from X

		size_t N = X.size();
		size_t m = X[0].size();

		// Initial and final vectors
		std::vector<double> F1 = X[0];
		std::vector<double> FN = subtract_vectors(X[N - 1], F1);  // Shift the end point

		// Shift all points
		std::vector<std::vector<double>> shifted_matrix(N, std::vector<double>(m));
		for (size_t i = 0; i < N; ++i) {
			shifted_matrix[i] = subtract_vectors(X[i], F1);
		}

		MD = 0.0;

		// Calculate distances
		for (size_t t = 1; t < N - 1; ++t) {
			std::vector<double> Ft = shifted_matrix[t];

			// Project Ft onto the ideal straight line
			double proj_scalar = dot_product(Ft, FN) / dot_product(FN, FN);
			std::vector<double> proj = scale_vector(FN, proj_scalar);

			// Calculate the Euclidean distance
			double d = calculate_norm(subtract_vectors(Ft, proj));

			MD += d;

			
		}

		MD /= (N - 2);

		std::cout << "MD Computed: " << MD << std::endl;  // Debug output

		MD_done = TRUE;
	}
}

//////////////////////////////////////////////////////////////////////
// control Trial: A state-driven routine to guide through the process of a trial
//////////////////////////////////////////////////////////////////////
//bool planErrorFlag = 0;		// flag for checking if error happens during planning.
//bool chordErrorFlag = 0;	// flag for checking if the chord was correct or not.
bool fingerCorrect[5] = { 0,0,0,0,0 };
bool chordCorrect = 0;
bool prev_chordCorrect;
void MyTrial::control() {
	int i;
	double fingerForceTmp;
	char tmpChord;
	bool check_baseline_hold = 0;

	switch (state) {
	case WAIT_TRIAL: //0
		gs.showLines = 1;	// set screen lines/force bars to show
		gs.showFeedback = 0;
		gs.showTarget = 0;
		gs.showFxCross = 0;
		gs.showTimer5 = 0;
		gs.showForces = 1;
		gs.showDiagnostics = 0;
		// gs.showForceBars = 1;
		gs.rewardTrial = -1;
		trialPoint = 0;
		//gs.planError = 0;
		gs.boxColor = 5;	// grey baseline box color

		
		//planErrorFlag = 0;
		//SetDacVoltage(0, 0);	// Ali EMG - gets ~200us to change digital to analog. Does it interrupt the ADC?
		SetDIOState(0, 0xFFFF); // Ali EMG

		for (i = 0; i < NUMDISPLAYLINES; i++) {
			if (!gs.line[i].empty()) {
				gs.lineColor[i] = 0;
				gs.line[i] = "";
			}
		}
		break;

	case START_TRIAL: //1	e
		gs.showLines = 1;	// set screen lines/force bars to show
		gs.showFeedback = 0;
		gs.showTimer5 = 0;
		gs.showFxCross = 1;
		gs.showForces = 1;
		gs.boxColor = 5;	// grey baseline box color
		gs.chordError = 0;
		//planErrorFlag = 0;	// initialize planErrorFlag variable in the begining of each trial
		//chordErrorFlag = 1;	// initialize chordErrorFlag variable in the begining of each trial
		startTriggerEMG = 1;	// Ali EMG: starts EMG trigger in the beginning of each trial
		trialPoint = 0;
		gs.rewardTrial = -1;
		planError = 0;

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

		holdTime = 0;
		max_holdTime = 0;

		gs.rewardTrial = -1;

		gs.showTarget = 0;
		gs.showFeedback = 0;
		gs.showForces = 1;
		gs.showLines = 1;
		gs.showFxCross = 1;

		gs.reset();

		cout << gCounter.readTotTime() << '\n';\
		cout << startTime << '\n';
		if (gCounter.readTR() > 0 && gCounter.readTotTime() >= startTime) {
			startTimeReal = gCounter.readTotTime();
			startTRReal = gCounter.readTR(); // number of TR arrived so far

			gTimer.reset(1);					//time for whole trial
			gTimer.reset(2);					//time for events in the trial			
			gTimer.reset(3);
			state = WAIT_PLAN;

			planError = 0;

		}
		break;

	case WAIT_PLAN: //2
		//gs.planCue = 1;
		gs.showTimer5 = 0;
		gs.showFxCross = 1;
		gs.showForces = 1;
		gs.showLines = 1;
		gs.showTarget = 1;

		for (i = 0; i < 5; i++) {	// RT is the time of the first finger outside the baseline area
			fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
			//check_baseline_hold = 1;
			if (fingerForceTmp >= (VERT_SHIFT + baseTHhi) || fingerForceTmp <= (VERT_SHIFT - (baseTHhi))) {
				planError = 1;
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

	case WAIT_EXEC:
		gs.showLines = 1;		// show force bars and thresholds
		gs.showTarget = 1;		// show the targets on the screen (grey bars)
		gs.showTimer5 = 0;		// show timer 4 value on screen (duration of holding a chord)
		gs.showForceBars = 1;
		gs.showFxCross = 1;
		gs.boxColor = 5;		// grey baseline box color

		if (chordCorrect == 0 && chordStarted == 0) {
			for (i = 0; i < 5; i++) {	// RT is the time of the first finger outside the baseline area
				fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
				//check_baseline_hold = 1;
				if (fingerForceTmp >= (VERT_SHIFT + baseTHhi) || fingerForceTmp <= (VERT_SHIFT - (baseTHhi))) {
					diffForceMov1[i] = fingerForceTmp;
					RT = gTimer[2];
					chordStarted = 1;
					std::vector<double> X(5, 0.0);
					break;
				}
			}
		}
		else if (chordStarted == 1) {
			std::vector<double> diffForceMov_offset(5, 0.0);
			for (i = 0; i < 5; i++) {
				diffForceMov[i] = (gBox[0].getForce(i) - gBox[1].getForce(i));
				diffForceMov_offset[i] = diffForceMov[i] - diffForceMov1[i]; // Element-wise subtraction
			}
			X.push_back(diffForceMov_offset);  // Append to class member X
		}
		
		// checking state of each finger
		for (i = 0; i < 5; i++) {
			tmpChord = chordID[i];	// required state of finger i -> 0:relaxed , 1:extended , 2:flexed -- chordID comes from the target file
			fingerForceTmp = VERT_SHIFT + forceGain * fGain[i] * (gBox[0].getForce(i) - gBox[1].getForce(i));
			//fingerForceTmp5[i] = fingerForceTmp;
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
			gs.fingerCorrectGraphic[i] = 1;
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

		// if subject held the chord for execAccTime (accepting hold time), trial is correct -> go to feedback state:
		if (gTimer[5] >= execMaxTime) {

			// go to the give_feedback state:
			state = GIVE_FEEDBACK;

			// resetting timers:
			gTimer.reset(2);
			gTimer.reset(3);

			MD_done = FALSE;
		}

		break;

	case GIVE_FEEDBACK:
		//SetDacVoltage(0, 0); // Ali EMG
		SetDIOState(0, 0xFFFF);

		if (chordStarted == 1 && chordCorrect == 1 && planError == 0 && max_holdTime >= success_holdTime) {
			trialPoint = 1;
		}
		else {
			MD = 0;
			RT = 0;
			ET = execMaxTime;
			trialPoint = 0;
		}

		gs.showLines = 1;			// no force lines/thresholds
		gs.showTarget = 0;			// no visual targets
		gs.showTimer5 = 0;
		gs.showFeedback = 0;		// showing feedback (refer to MyTrial::updateGraphics() for details)
		gs.rewardTrial = trialPoint;	
		gs.showFxCross = 1;
		gs.showForces = 1;
				
		if (gTimer[2] >= feedbackTime) {
			state = WAIT_ITI;
			gTimer.reset(2);
			gNumCorr = gNumCorr + trialPoint;
		}
		break;

	case WAIT_ITI:
		gs.showLines = 1;
		gs.showTarget = 0;
		gs.showForces = 1;
		gs.showFxCross = 1;
		gs.showFeedback = 0;
		gs.rewardTrial = -1;
		if (gTimer[2] >= iti) {
			state = ACQUIRE_HRF;
			dataman.stopRecording();
			gTimer.reset(2);
		}
		break;

	case ACQUIRE_HRF: //6

		gs.showFxCross = 1;
		gs.showForces = 1;
		gs.showLines = 1;

		i = gCounter.readTotTime();

		if (gCounter.readTotTime() >= endTime)
		{
			state = END_TRIAL;
			gTimer.reset(2);
		}
		else
		{
			gs.showTarget = 0;
			gs.showFeedback = 0;
			//if (gTimer[2] > hrfTime) {  // wait 12 s at the end of the run
			//	state = END_TRIAL;
			//	
			//	//cout << "HRF acquired for 12 seconds after trial" << endl;

		}
		break;

	case END_TRIAL:
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

	TotTime = gCounter.readTotTime(); //internally generated time initiated at first TTL pulse
	TR = gCounter.readTR(); //counted TR pulse
	TRtime = gCounter.readTime(); //time since last TR
	currentSlice = 0; //gCounter.readSlice();

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

	if (state == 4 && started) {
		X[trialNum].push_back(currentDiffForce);
	}

}

//double DataRecord::get_MD() const {
//	return MD;
//}

/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream& out) {
	int i, j;
	out << trialNum + 1 << "\t"
		<< state << "\t"
		<< timeReal << "\t"
		<< time << "\t"
		<< TotTime << "\t" // time from first TR
		<< TR << "\t" // number of TR so far
		<< TRtime << "\t" // time since last TR
		<< currentSlice << "\t";

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