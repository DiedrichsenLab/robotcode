///////////////////////////////////////////////////////////////////////
/// Bimanual wrist movement fmri mode only, minimal version
/// Diogo Duarte, based on complete version by Atsushi Yokoi and me 
////////////////////////////////////////////////////////////////////////
#include "BimanualWrist_MR.h"
#include <chrono>
#include <thread>

/*
#include "TRCounter626.h"
#include "Screen.h"
#include "S626sManager.h"
#include "Timer626.h"
#include <string.h>
#include <sstream>
#include "WristManipulandum.h" //WristManipulandum
*/
//////////// Control vars
#define SAMPLERATE 			5		// Sample and control rate must be multiples 
#define CONTROLRATE 		5		// of update rate
#define UPDATERATE 			1		// 1 for manipulandomDR
#define TEXTDISPLAYRATE 	100 
#define SCR_SCALE 			2.8/72
#ifndef PI
#define PI 					3.141592653589793238
#endif

////////// exp vars
#define TARGET_WIDTH 		1.6			///< Width of target (cm)
#define TARGET_HEIGHT 		1.6
#define CURSORSIZE 			0.5
#define STARTSIZE 			1.6
#define FIXCROSS_SIZE		3
#define FIXCROSS_THICK		0.3 
#define TARGET_Y 			0 		///< Y position for target 
#define START_Y 			0		///< Y position for start point, world coordinates (cm) 
#define DISTX_HAND_CNTR 	8 		
#define MAXPOSSIBLEPOINTS 	168		/// max num of points
#define N_TARGETS			6

/////// TR counter
#define TRTIME				2720 // must be adjusted

//////// function headers
inline bool int2bool(int num);
inline std::string int2string(int num);
inline double atan_deg(double y, double x);

///////////// global vars
S626sManager s626;
TextDisplay tDisp;
Screen gScreen;						///< Subject Screen 
WristManipulandum manip[N_ROBOTS];	///< WristManipulandum
Timer gTimer(UPDATERATE);			///< Timer Class that uses a hard timer on the s626 board 
HINSTANCE gThisInst;				///< Instance of Windows application 
Experiment* gExp;					///< Pointer to myExperiment 
Trial* currentTrial;				///< Pointer to current Trial 
bool gKeyPressed;					///< Key pressed? 
char gKey;							///< Which key?
bool blockFeedbackFlag = 0;
bool gTimerFlagFirst = 0;

char 	gBuffer[200];
bool 	blockEnded = false;
string 	totalPoints;
int 	gPointsBlock = 0;
int 	gPoints = 0;
bool 	showtargets = true;
bool 	togglesound = false;
bool 	flipscreen = false;
bool 	taskoutlimit = false;		/// if the subjs break boxes too soon, too late, or do not behave properly, tolerate in mri

FixCross 	fixationCross;
Target 		startBox[N_ROBOTS];
Target 		targetBox[N_ROBOTS];
double 		targetspreview[N_TARGETS];
Target 		previewBoxL[N_TARGETS];
Target 		previewBoxR[N_TARGETS];
Vector2D 	targetOrig[N_ROBOTS];
Vector2D 	pos[N_ROBOTS];
Vector2D 	hPos[N_ROBOTS];
Vector2D 	cPos[N_ROBOTS];
double 		cAngle[N_ROBOTS];
double 		radius[N_ROBOTS];
double 		hAngle[N_ROBOTS];
Matrix2D 	TransforMatrix(1, 0, 0, 1);	///< adjusts for the fact that subject screen is flipped. used in angles (0,1,1,0)
Vector2D 	DeviceFlip[N_ROBOTS];		///< 1, no flip in hPos, -1 flip.

string stateName[10] = {
					"WAIT_TRIAL",			///< 0 Trial is not started yet, is default mode
					"START_TRIAL",			///< 1 Start trial	
					"WAIT_TR",				///< 2 Wait for TR pulse to begin the trial	
					"WAIT_CENTER_HOLD",		///< 3 Wait for the hand to be start position 	
					"SHOW_TARGET",			///< 4 Show target but don't move yet
					"GO_CUE",				///< 5 Show go signal
					"MOVING",				///< 6 During the movement 
					"GIVE_FEEDBACK",		///< 7 Show feedback 
					"ACQUIRE_HRF", 			///< 8 hold recording and subj hand pos in scanner
					"END_TRIAL",			///< 9 Absorbant State: Trial is finished										
};

TRCounter gCounter;				///< TR counter, simulated and pulse-triggered 
char counterSignal = '5';		///< What char is used to count the TR
int sliceNumber = 32;			///< How many slices do we have

int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst, LPSTR kposzArgs, int nWinMode) {

	gThisInst = hThisInst;
	gExp = new MyExperiment("BimanualWrist_MR", "BimanualWrist_MR", "C:/data/BimanualWrist_MR/");  // Set experiment name and code for data files 
	gExp->redirectIOToConsole();
	tDisp.init(gThisInst, 0, 0, 400, 20, 5, 4, &(::parseCommand));
	tDisp.setText("Subj:", 0, 0);

	for (int targetn = 0; targetn < N_TARGETS; targetn++) { targetspreview[targetn] = targetn * 360 / N_TARGETS; }

	//gScreen.init(gThisInst, 1920, 0, 1920, 1080, &(::updateGraphics)); ///< Display for subject
	gScreen.init(gThisInst, 640, 0, 640, 1024, &(::updateGraphics)); ///< Display for subject
	gScreen.setCenter(Vector2D(0, 0));
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));

	s626.init("c:/robotcode/calib/s626_single.txt");

	int manipCount;
	for (manipCount = 0; manipCount < N_ROBOTS; manipCount++) {
		if (manipCount == 0) {
			manip[manipCount].init(BOX_LEFT, "c:/robotcode/calib/WristManip.dat");
			double baseL[] = { 2.50,2.50 };		///< Left Manip Baseline
			manip[manipCount].setScale(Vector2D(1, 1));
			DeviceFlip[0] = Vector2D(-1, -1);
			targetOrig[manipCount] = Vector2D(-DISTX_HAND_CNTR, TARGET_Y);
			startBox[manipCount].position = Vector2D(-DISTX_HAND_CNTR, START_Y);
		}
		if (manipCount == 1) {
			manip[1].init(BOX_RIGHT, "c:/robotcode/calib/WristManip.dat");
			double baseR[] = { 2.50,2.50 };		///< Right Manip Baseline 
			manip[manipCount].setScale(Vector2D(1, 1));
			DeviceFlip[manipCount] = Vector2D(1, -1);
			targetOrig[manipCount] = Vector2D(DISTX_HAND_CNTR, TARGET_Y);
			startBox[manipCount].position = Vector2D(DISTX_HAND_CNTR, START_Y);
		}

		targetBox[manipCount].position = targetOrig[manipCount];
		targetBox[manipCount].size = Vector2D(TARGET_WIDTH, TARGET_HEIGHT);
		targetBox[manipCount].setShape(SHAPE_DISC);
		startBox[manipCount].size = Vector2D(STARTSIZE, STARTSIZE);
		startBox[manipCount].setShape(SHAPE_CIRC);	// DISC
	}

	fixationCross.position = gScreen.getCenter();
	fixationCross.size = Vector2D(FIXCROSS_SIZE, FIXCROSS_SIZE);
	fixationCross.setShape(SHAPE_PLUS);

	

	for (int i = 0; i < N_TARGETS; i++) {
		// left preview boxes
		previewBoxL[i].position = (Vector2D(
			5 * cos(targetspreview[i] * PI / 180),
			5 * sin(targetspreview[i] * PI / 180))
			+ startBox[0].position);
		previewBoxL[i].setShape(SHAPE_CIRC);
		previewBoxL[i].size = Vector2D(TARGET_WIDTH + 0.1, TARGET_HEIGHT + 0.1);
		// right preview boxes
		previewBoxR[i].position = (Vector2D(
			5 * cos(targetspreview[i] * PI / 180),
			5 * sin(targetspreview[i] * PI / 180))
			+ startBox[1].position);
		previewBoxR[i].setShape(SHAPE_CIRC);
		previewBoxR[i].size = Vector2D(TARGET_WIDTH + 0.1, TARGET_HEIGHT + 0.1);
	}

	if (s626.getErrorState() == 0) {
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE);
	}

	//gTimer.init(1, 4, 0); // Ali Changed Here!!!!
	//gTimer.clear();

	// TR counter
	//gCounter.initSerial("COM1",9600, counterSignal, sliceNumber); 
	gCounter.init3(3, 0, sliceNumber); // TTL pulse for counting TR
	gCounter.simulate(TRTIME);

	gExp->control();
	return 0;

}

MyExperiment::MyExperiment(string name, string code, string dDir) :Experiment(name, code, dDir)
{
	theBlock = new MyBlock();
	theTrial = new MyTrial();
	currentTrial = theTrial;
}

void MyExperiment::control(void) {
	MSG msg;
	if (!gTimerFlagFirst) { // Ali Changed Here!!!!
		gTimer.init(1, 4, 0); 
		gTimer.clear();
		gTimerFlagFirst = TRUE;
	}

	do {
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		theBlock->control();
		if (gTimer[3] > TEXTDISPLAYRATE) {
			currentTrial->updateTextDisplay();
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			gTimer.reset(3);
		};
		InvalidateRect(gScreen.windowHnd, NULL, TRUE);
		UpdateWindow(gScreen.windowHnd);
	} while (msg.message != WM_QUIT);
}

bool MyExperiment::parseCommand(string arguments[], int numArgs)
{
	int i;
	float dummy1;

	if (arguments[0] == "showtargets" || arguments[0] == "showtargets") {
		showtargets = !showtargets;

	}
	else if (arguments[0] == "TR" || arguments[0] == "tr") {
		if (numArgs != 2) {
			tDisp.print("USAGE: TR delay [ms]");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &dummy1);
			if (dummy1 >= 0) {
				gCounter.simulate(dummy1);
			}
			else {
				gCounter.simulate(0);
			}
		}

	}
	else if (arguments[0] == "zero" || arguments[0] == "ZERO") {
		tDisp.print("0 points");
		gPointsBlock = 0;
		/// Enable Force production again 

	}

	else if (arguments[0] == "baseline" || arguments[0] == "BASELINE") {
		if (numArgs == 1) {
			manip[0].setZero(Vector2D(manip[0].getVolts(0), manip[0].getVolts(1)));
			manip[1].setZero(Vector2D(manip[1].getVolts(0), manip[1].getVolts(1)));		// Set baseline Right
		}
		else {
			tDisp.print("Usage: BASELINE L[h] L[v] R[h] R[v] or BASELINE");
		}
	}

	else if (arguments[0] == "baselinebuffer" || arguments[0] == "zeroF") {
		int n, j;
		double volts_b[2][2] = { {0,0},{0,0} };
		for (n = 0; n < 100; n++) {
			for (i = 0; i < N_ROBOTS; i++) {    // i is manip, j is encoder
				for (j = 0; j < 2; j++) {
					volts_b[i][j] += manip[i].getVolts(j);
				}
			}
			Sleep(5);
		}
		for (i = 0; i < N_ROBOTS; i++) {
			for (j = 0; j < 2; j++) {
				volts_b[i][j] /= 100;
			}
			manip[i].setZero(volts_b[i]);
		}
	}

	else if (arguments[0] == "flipscreen" || arguments[0] == "FLIPSCREEN") {
		if (!flipscreen) {
			TransforMatrix = Matrix2D(0, 1, 1, 0);
			gScreen.setScale(Vector2D(-SCR_SCALE, SCR_SCALE));
			flipscreen = true;
		}
		else { // flipscreen is true, is in mri mode, going to training mode
			TransforMatrix = Matrix2D(1, 0, 0, 1);
			gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
			flipscreen = false;
		}
	}

	else if (arguments[0] == "resetTR" || arguments[0] == "resettr") {
		gCounter.reset();
	}

	else if (arguments[0] == "togglesound") {
		togglesound = !togglesound;
	}

	else if (arguments[0] == "resize") {
		gScreen.setCenter(Vector2D(0, 0));
		gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
	}

	else {
		return false;			///< Command not recognized 
	}

	return true;				///< Command recognized 
}

void MyExperiment::onExit() {
	s626.stopInterrupt();
	tDisp.close();
	gScreen.close();
}

MyBlock::MyBlock() :Block() {
	state = WAIT_BLOCK;
}

void MyBlock::start() {
	gCounter.reset();  // reset and start kind of overlap here, remove reset?
	gCounter.start();
	gTimer.clear();
	gPointsBlock = 0;
	blockFeedbackFlag = 0;
}

void MyBlock::giveFeedback() {
	gCounter.stop();
	totalPoints = int2string(gPointsBlock) + " / " + int2string(MAXPOSSIBLEPOINTS) + " points";
	blockEnded = true;
	blockFeedbackFlag = 1;
}

Trial* MyBlock::getTrial() {
	return new MyTrial();
}

MyTrial::MyTrial() {

	state = WAIT_TRIAL;
	errState = WAIT_TRIAL; // this means no bad trial in this block
	hand = 0;
	targetDistance = 5;
	gHoldTime = 1000;
	time2plan = 2000;
	reachTime = 3000;
	MaxTrialTime = 10000;
	RT_min = 0;
	RT_max = 800;
	for (int hRobot = 0; hRobot < N_ROBOTS; hRobot++) {
		targetAngle[hRobot] = 0;
		maxRadius[hRobot] = 0;
		endRadius[hRobot] = 0;
		startBox[hRobot].setColor(SCR_WHITE);
		targetBox[hRobot].setColor(SCR_WHITE);
	}
	u_or_b = 0;  						// unimanual (0) or bimanual (1)
	MT = 0;								// movement time
	RT = 0; 							// reaction time
	MaxTrialTime = 10000;
	//startSlice = 0;
	//startSlicereal=0; 
	startTR = 0;
	startTRReal = 0;
	startTime = 0;
	startTimeReal = 0;
	// reset task stuff
	ranOnce = false;					// flag to sum total points only once
	fixationCross.setColor(SCR_WHITE);
	blockEnded = false; 				// reset blockEnded
	gPointsBlock = 0;
	gPoints = 0;
}

void MyTrial::read(istream& input) {

	input >> u_or_b  // unimanual = 0, bimanual = 1
		>> hand
		>> startTime //counts up since the first triggering TTL pulse from scanner. If 0, not in the scanner (training mode)
		>> targetAngle[0]
		>> targetAngle[1]
		>> targetDistance
		>> gHoldTime
		>> time2plan
		>> reachTime
		>> feedbackTime
		>> MaxTrialTime
		>> RT_min
		>> RT_max
		>> innerTargetTolerance
		>> outerTargetTolerance
		>> targetAngleTolerance
		>> centerTolerance;
}

void MyTrial::writeDat(ostream& output) {

	output << u_or_b << "\t"
		<< hand << "\t"
		<< startTime << "\t" //repeat of target file. if 0, training mode
		<< startTimeReal << "\t" //actual time of the beginning of each trial since T=0
		<< targetAngle[0] << "\t"
		<< targetAngle[1] << "\t"
		<< targetDistance << "\t"

		<< RT << "\t"	 // reaction time
		<< MT << "\t"
		<< time2plan << "\t"
		<< MaxTrialTime << "\t"
		<< endRadius[0] << "\t"
		<< endRadius[1] << "\t"
		<< endAngle[0] << "\t"
		<< endAngle[1] << "\t"

		<< GoodMovement << "\t"
		<< pointsMyTrial << "\t"
		<< errState << "\t"
		<< endl;
}

void MyTrial::writeHeader(ostream& output) {

	output << "Uni_or_Bi" << "\t"
		<< "Hand" << "\t"
		<< "startTime" << "\t" //repeat of target file: TIME BEGINNING FOR EACH TRIAL SINCE T=0 (1st TTL)
		<< "startTimeReal" << "\t" //actual time of the beginning of each trial since T=0
		<< "targetAngle_L" << "\t"
		<< "targetAngle_R" << "\t"
		<< "targetDistance" << "\t"

		<< "RT" << "\t"
		<< "MT" << "\t"
		<< "time2plan" << "\t"
		<< "MaxTrialTime" << "\t" //^repeat of tgt file
		<< "dEndRadius_L" << "\t"
		<< "dEndRadius_R" << "\t"
		<< "dEndAngle_L" << "\t"
		<< "dEndAngle_R" << "\t"

		<< "GoodMovement" << "\t"
		<< "Points" << "\t"
		<< "errState" << "\t"
		<< endl;
}

void MyTrial::updateHaptics() {

	// Update timer626
	gTimer.countup();
	gTimer.countupReal();

	// update s626
	s626.updateAD(0);
	gCounter.update();

	for (int i = 0; i < N_ROBOTS;/*=u_or_b;*/ i++) {  // before : i<N_ROBOTS;

		manip[i].update(gTimer.dt() / 1000);	// Read the new position and velocity from the sensors 
		hPos[i][0] = manip[i].getPositionFilt(0) * DeviceFlip[i].x[0];//manip[i].getPosition(0);
		hPos[i][1] = manip[i].getPositionFilt(1) * DeviceFlip[i].x[1];//manip[i].getPosition(1);
		radius[i] = hPos[i].norm(); // this caused crash norm(hPos[i]); // replace for std::abs								// Distance from the startbox
		hAngle[i] = atan_deg(hPos[i].x[1], hPos[i].x[0]);
		cPos[i] = (hPos[i]) + startBox[i].position;				// Location of cursor
		cAngle[i] = atan2(cPos[i].x[1], cPos[i].x[0]) / PI * 180; 		// Angle of the cursor movement
	}

	if ((gTimer[4]) >= CONTROLRATE)
	{
		gTimer.reset(4);
		control();
	}
	if ((gTimer[5]) >= SAMPLERATE && dataman.isRecording())
	{
		gTimer.reset(5);
		dataman.record(DataRecord(state, hand, u_or_b, gPointsBlock, RT));
	}
}

void MyTrial::updateGraphics(int eye) {
	int i;	// Ali added this line

	if (blockFeedbackFlag) {
		gScreen.setCenter(Vector2D(0, 0));    // In cm //0,2
		gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));
	}

	if (showtargets) {
		for (int a = 0; a < N_TARGETS; a++) {
			previewBoxL[a].setColor(SCR_WHITE);
			previewBoxR[a].setColor(SCR_WHITE);
			previewBoxL[a].draw();
			previewBoxR[a].draw();
		}
	}

	// screen text is white except in feedback, fixationCross always there
	if ((state != GIVE_FEEDBACK && state != END_TRIAL && !blockEnded)) {
		gScreen.setColor(0);
		fixationCross.draw();
	}

	// blue warning / tolerance paradigm. if dont agree just comment this out
	if ((state >= WAIT_CENTER_HOLD && state < GIVE_FEEDBACK && taskoutlimit)) {
		fixationCross.setColor(SCR_BLUE);
	}
	else {
		//
	}

	// startBox and cursors
	for (int i = 0; i < N_ROBOTS; i++) {
		startBox[i].draw();
		gScreen.drawDisk(Vector2D(CURSORSIZE, CURSORSIZE), cPos[i]);
	}

	// target boxes
	if (state >= WAIT_CENTER_HOLD && state <= GIVE_FEEDBACK) {
		if (u_or_b == 0) {  // unimanual
			targetBox[hand].draw();
		}
		else {				// bimanual
			for (i = 0; i < N_ROBOTS; i++) {
				targetBox[i].draw();
			}
		}
	}

	// draw point feedback
	if (state == GIVE_FEEDBACK) {

		if (GoodMovement) {
			gScreen.setColor(SCR_GREEN);
		}
		else {
			gScreen.setColor(SCR_RED);
		}
		gScreen.print(int2string(gPointsBlock), Vector2D(0, 0), 8); //print the score
		gScreen.print(int2string(gPoints), Vector2D(0, 2), 8); //print the score
	}

	// fixation cross bigger in GO_CUE
	if (state >= GO_CUE && state <= MOVING) {
		fixationCross.size = Vector2D(FIXCROSS_SIZE * 1.2, FIXCROSS_SIZE * 1.2);
	}
	else {
		fixationCross.size = Vector2D(FIXCROSS_SIZE, FIXCROSS_SIZE);
	}

	// points in the end of the block
	if (blockEnded)
	{
		gScreen.print(totalPoints, Vector2D(0, 6), 10);
	}

}

void MyTrial::updateTextDisplay() {
	sprintf(gBuffer, "State : %s", stateName[state].c_str());
	tDisp.setText(gBuffer, 1, 0);

	sprintf(gBuffer, "BN: %d  TN: %d", gExp->theBlock->blockNumber, gExp->theBlock->trialNum + 1);
	tDisp.setText(gBuffer, 2, 0);

	//sprintf(gBuffer,"Slice: %d",gCounter.readSlice());
	//tDisp.setText(gBuffer,1,2);

	sprintf(gBuffer, "TotTime: %d", gCounter.readTotTime());
	tDisp.setText(gBuffer, 1, 2);

	sprintf(gBuffer, "TR: %d", gCounter.readTR());
	tDisp.setText(gBuffer, 2, 2);

	sprintf(gBuffer, "Points: %d  Points total: %d", gPoints, gPointsBlock);
	tDisp.setText(gBuffer, 5, 0);

	sprintf(gBuffer, "Timer: %3.2f ", gTimer.readReal(0));
	tDisp.setText(gBuffer, 7, 1);
	sprintf(gBuffer, "gTimer 1: %3.2f ", gTimer[1]);
	tDisp.setText(gBuffer, 7, 2);

	// volts
	sprintf(gBuffer, "-> Volts0: [%3.2f] [%3.2f]", manip[0].getVolts(0), manip[0].getVolts(1));
	tDisp.setText(gBuffer, 9, 1); //9
	sprintf(gBuffer, "-> Volts1: [%3.2f] [%3.2f]", manip[1].getVolts(0), manip[1].getVolts(1));
	tDisp.setText(gBuffer, 9 + 1, 1);//10

}

void MyTrial::control() {

	if (state <= WAIT_CENTER_HOLD) {			// reset timer1
		gTimer.reset(1);				// Timer for the block. Generates trial time out
	}

	if (gTimer[1] > MaxTrialTime - feedbackTime * 2) {		// 500 is feedback time
		badNoPoints(); //penalty();
		state = GIVE_FEEDBACK;
		gTimer.reset(2);
		gTimer.reset(1);
	}

	switch (state) {

	case WAIT_TRIAL:			// trial didn't start yet, default mode	
		gTimer.reset(0);				// global timer. Not actually being used		
		gTimer.reset(2);				// Timer within each state
		gTimer.reset(1);
		break;

	case START_TRIAL:
		cout << "Starting TN: " << gExp->theBlock->trialNum + 1 << endl;
		RT = 0;
		gPoints = 0;
		overTime = false;
		ranOnce = false;
		taskoutlimit = false;

		// Set target position for this trial
		for (r = 0; r < N_ROBOTS; r++) {
			targetBox[r].position = (Vector2D(
				targetDistance * cos(targetAngle[r] * PI / 180),
				targetDistance * sin(targetAngle[r] * PI / 180))
				+ startBox[r].position);
			endpoint[r] = Vector2D(0, 0);
			targetBox[r].setColor(0);
		}
		state = WAIT_TR;
		fixationCross.setColor(SCR_WHITE);
		break;

	case WAIT_TR:
		/*
			if (startSlice<1){	// training mode
					state = WAIT_CENTER_HOLD;
			}
			else {		// start slice is 1 or higher, mri mode
				if (gCounter.readSlice() <= startSlice){
					state = WAIT_TR;
				}
				else {
					state = WAIT_CENTER_HOLD;
					startSlicereal = gCounter.readSlice();
					cout<<"startSlice(>1)="<<startSlice<<endl;
					cout<<"gCounter.readSlice="<<gCounter.readSlice()<<endl;
				}
			}
		*/
		if (gCounter.readTR() > 0 && gCounter.readTotTime() >= startTime) {
			startTimeReal = gCounter.readTotTime();
			startTRReal = gCounter.readTR();

			//dataman.startRecording(); // see around line #660
			gTimer.reset(1);					//time for whole trial
			gTimer.reset(2);					//time for events in the trial			

			state = WAIT_CENTER_HOLD;
		}
		break;


	case WAIT_CENTER_HOLD:				// If in MRI, skip hold time


		if (!taskoutlimit) { fixationCross.setColor(SCR_RED); }

		if (startTime > 0) {		// mri mode

			state = SHOW_TARGET;				// go to show target anyway
			if (radius[0] > (STARTSIZE / 2 + centerTolerance)
				|| radius[1] > (STARTSIZE / 2 + centerTolerance)) {		// in start boxes
				taskoutlimit = true; 		// subj screwed up, in the end they should get no points
			}
			gTimer.reset(2);
		}
		else {						// non mri mode
			if (radius[0] < (STARTSIZE / 2 + centerTolerance)
				&& radius[1] < (STARTSIZE / 2 + centerTolerance)
				&& gTimer[2] > gHoldTime) {		// in start boxes and held for time
				state = SHOW_TARGET;
				gTimer.reset(2);
			}
		}
		break;


	case SHOW_TARGET:					// prompt target red and wait, plan movement
		if (!dataman.isRecording()) { dataman.startRecording(); }
		if (!taskoutlimit) { fixationCross.setColor(SCR_RED); }


		if (gTimer[2] > time2plan) {		// time for planning movemement
			gTimer.reset(2);
			state = GO_CUE;
		}

		// target boxes to white
		if (u_or_b == 0) {
			targetBox[hand].setColor(SCR_WHITE);
		}
		else {		// bimanual
			for (r = 0; r < N_ROBOTS; r++) {
				targetBox[r].setColor(SCR_WHITE);
			}
		}

		// if they breach startbox area (jump the gun)
		if (radius[0] > (STARTSIZE / 2 + centerTolerance) || radius[1] > (STARTSIZE / 2 + centerTolerance)) {
			if (startTime > 0) {		// mri mode
				taskoutlimit = true;
			}
			else {						// behav mode
				penalty();
				if (startTime == 0 || togglesound) { PlaySound("wav/chord.wav", NULL, SND_ASYNC | SND_FILENAME); }
				gTimer.reset(2);
				state = GIVE_FEEDBACK;
			}
			errState = SHOW_TARGET;
		}

		break;



	case GO_CUE:					// give go cue, fixation cross 

		if (!taskoutlimit) { fixationCross.setColor(SCR_GREEN); }

		// time limit
		if ((gTimer[2] > RT_max || gTimer[2] < RT_min)) {
			if (startTime == 0) {	// behav mode
				badNoPoints(); //penalty();
				gTimer.reset(2);
				state = GIVE_FEEDBACK;
			}
			else {				// mri mode
				taskoutlimit = true;
			}
			errState = GO_CUE;
		}

		// if unimanual, penalize if other hand breaches, // trial is only interrupted in mri mode
		if (!int2bool(u_or_b) && radius[abs(hand - 1)] > ((TARGET_WIDTH / 2) + centerTolerance)) {
			if (startTime == 0) {	// behav mode
				gTimer.reset(2);
				state = GIVE_FEEDBACK;
				badNoPoints();
			}
			else {				// mri mode
				taskoutlimit = true;
			}
			errState = GO_CUE;
		}

		// if they breach starting circle + factor, measure reaction time
		if (!u_or_b && radius[hand] > ((TARGET_WIDTH / 2) + centerTolerance)) {
			RT = gTimer[2];
			gTimer.reset(2);
			state = MOVING;
		}
		else if (u_or_b && radius[0] > ((TARGET_WIDTH / 2) + centerTolerance) || radius[1] > ((TARGET_WIDTH / 2) + centerTolerance)) {
			RT = gTimer[2];
			gTimer.reset(2);
			state = MOVING;
		}

		break;



	case MOVING:

		for (r = 0; r < N_ROBOTS; r++) {			// save max radius
			if (radius[r] > maxRadius[r]) { // reached enpoint
				maxRadius[r] = radius[r];
				endpoint[r] = Vector2D(hPos[r].x[0], hPos[r].x[1]);
			}
		}

		MT = max(MT, gTimer[2]);					// save movement time
		// time limit		
		if (MT > reachTime) {						// subjs have reachTime to complete the trial
			overTime = true;
			errState = MOVING;
		}

		// return to center, jump to feedback (trial is finished, compute outcome)
		if (radius[0] < ((TARGET_WIDTH / 2) + centerTolerance) && radius[1] < ((TARGET_WIDTH / 2) + centerTolerance)) {
			computeOutcome();					// outcome is determined by this routine
			gTimer.reset(2);
			state = GIVE_FEEDBACK;
		}
		// save movement time

		break;



	case GIVE_FEEDBACK:		// stay here for feedbackTime ms

		//		gPointsBlock = max(0,gPoints+(!ranOnce)*gPoints); ranOnce = true;

		if (gTimer[2] > feedbackTime) {
			state = ACQUIRE_HRF;
			gTimer.reset(2);
		}

		if (!GoodMovement) {
			for (r = 0; r < N_ROBOTS; r++) {
				targetBox[r].setColor(0);
			}
		}
		else {		// goodmovement
			targetBox[r].setColor(SCR_WHITE);
			if (u_or_b == 0 && startTime == -1) {
				targetBox[hand].explode(1);
			}
			else if (u_or_b == 1 && startTime == -1) {
				for (r = 0; r < N_ROBOTS; r++) {	// bimanual
					targetBox[r].explode(1);
				}
			}
		}
		pointsMyTrial = gPoints;			// kind of cheap trick

		break;

	case ACQUIRE_HRF:

		if (gExp->theBlock->trialNum + 1 != gExp->theBlock->numTrials)
		{
			state = END_TRIAL;
		}
		else
		{
			if (gTimer[2] > 10000) {
				state = END_TRIAL;
				gTimer.reset(2);
				cout << "HRF acquired for 10 seconds after trial" << endl;
			}
		}




	case END_TRIAL:			// end trial, count total points
		gTimer.reset(1);
		dataman.stopRecording();
		fixationCross.setColor(SCR_WHITE);
		break;

	}

}

void MyTrial::end()
{
	state = END_TRIAL;
	dataman.stopRecording();

	for (int r = 0; r < N_ROBOTS; r++) {  // r<N_ROBOTS
		targetBox[r].setColor(0);
	}
}

void MyTrial::writeMov(ostream& out)
{
	dataman.save(out);
}

DataRecord::DataRecord(int s, int hand, int UB, int gPoints, int rt) {

	time = gTimer[1];
	timeReal = gTimer.readReal(1);
	state = s;

	TotTime = gCounter.readTotTime(); //internally generated time initiated at first TTL pulse
	TR = gCounter.readTR(); //counted TR pulse
	TRtime = gCounter.readTime(); //time since last TR
	currentSlice = 0;//gCounter.readSlice();

	for (int HandIndex = 0; HandIndex < N_ROBOTS; HandIndex++) {
		position[HandIndex] = hPos[HandIndex]; //position relative to startbox  //* position[N_ROBOTS]
		dRadius[HandIndex] = radius[HandIndex];
		dAngle[HandIndex] = hAngle[HandIndex];
		//	cursor[HandIndex] =	cPos[HandIndex]-startBox[HandIndex].position; //cursor relative to sta
		encoderAngles[HandIndex] = Vector2D(
			manip[HandIndex].getAnglesFilt(0), manip[HandIndex].getAnglesFilt(1));
	}

	u_or_b = UB;
	h = hand;
	points = gPoints;
	reactiontime = rt;
}

void DataRecord::write(ostream& out) {

	out << time << "\t" 									// A
		<< timeReal << "\t"										// B
		<< state << "\t"										// C
		<< u_or_b << "\t"		// Unimanual 0, Bimanual 1		// D
		<< h << "\t"		// Hand							// E
		<< dRadius[0] << "\t"										// F
		<< dRadius[1] << "\t"										// G
		<< dAngle[0] << "\t"										// H
		<< dAngle[1] << "\t"										// I

		<< TotTime << "\t" 									// J
		<< TR << "\t"			// from gCounter.readTR		// K
		<< currentSlice << "\t"										// L
		<< encoderAngles[0].x[0] << "\t"								// M
		<< encoderAngles[0].x[1] << "\t"								// N
		<< encoderAngles[1].x[0] << "\t"								// O
		<< encoderAngles[1].x[1] << "\t"								// P
		<< endl;
}

inline bool int2bool(int num) {
	return((num != 0));
}

inline std::string int2string(int num) {
	std::stringstream ss; ss.str(""); ss << num;
	return(ss.str());
}

inline double atan_deg(double y, double x) {
	double angle;
	angle = atan2(y, x) * 180 / PI;
	if (y < 0) { angle = angle + 360; }
	return(angle);
}

void MyTrial::penalty() {
	if (gPointsBlock > 0) {
		gPoints = gPoints - 1;
		gPointsBlock = gPointsBlock - 1;
	}
	GoodMovement = false;
	errState = GIVE_FEEDBACK;
}

void MyTrial::reward() {
	gPoints = gPoints + 1;
	gPointsBlock = gPointsBlock + 1;
	GoodMovement = true;

}

void MyTrial::passNoPoints() {
	GoodMovement = true;
	errState = GIVE_FEEDBACK;
}

void MyTrial::badNoPoints() {
	GoodMovement = false;
	errState = GIVE_FEEDBACK;
}

void MyTrial::computeOutcome() {
	// define valid area: "pizza slice" definition
	// unimanual
	if (!int2bool(u_or_b)) {
		// in reward area
		if (inTargetArea(hand))
		{
			if (!overTime && !taskoutlimit) {
				reward();
				if (startTime == 0 || togglesound) { PlaySound("wav/smb_coin.wav", NULL, SND_ASYNC | SND_FILENAME); }
			}
			else {
				passNoPoints();
				if (startTime == 0 || togglesound) { PlaySound("wav/chord.wav", NULL, SND_ASYNC | SND_FILENAME); }
			}
		}
		// out of reward area
		else {
			badNoPoints(); //penalty();
			if (startTime == 0 || togglesound) { PlaySound("wav/chord.wav", NULL, SND_ASYNC | SND_FILENAME); }
		}
	}
	// bimanual
	else {
		for (r = 0; r < N_ROBOTS; r++) {
			// reward area
			if (inTargetArea(r))
			{
				if (!overTime && !taskoutlimit) { reward(); /*reward(); */ } 	// +2 points
				else {
					passNoPoints();
				}
			}
			// out of reward area
			else {
				badNoPoints(); //penalty();								
				//penalty();

			}
		}
		if (gPoints < 2)			// this should keep the fixationcross red 
		{							// in case the right hand hits the target and the left
			GoodMovement = false;	// does not
			if (startTime == 0 || togglesound) { PlaySound("wav/chord.wav", NULL, SND_ASYNC | SND_FILENAME); }
		}
		else {
			if (startTime == 0 || togglesound) { PlaySound("wav/smb_coin.wav", NULL, SND_ASYNC | SND_FILENAME); }
		}
	}
}

inline bool MyTrial::inTargetArea(int gHand) {
	endRadius[gHand] = norm(endpoint[gHand]);
	endAngle[gHand] = atan_deg(endpoint[gHand].x[1], endpoint[gHand].x[0]);
	if (endRadius[gHand] > (targetDistance - TARGET_WIDTH / 2 - innerTargetTolerance) &&
		endRadius[gHand] < (targetDistance + TARGET_WIDTH / 2 + outerTargetTolerance) && /*change 1 to var*/
		(fabs(endAngle[gHand] - targetAngle[gHand]) < targetAngleTolerance ||
			fabs(endAngle[gHand] - 360 - targetAngle[gHand]) < targetAngleTolerance ||
			fabs(endAngle[gHand] + 360 - targetAngle[gHand]) < targetAngleTolerance)
		)
	{
		return true;
	}
	else {
		return false;

	}
}
