///////////////////////////////////////////////////////////////
/// WorkingMemoryPlanning - ....
///////////////////////////////////////////////////////////////

#include "eir2.h" 
#include "StimulatorBox.h"

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

///< For Thread safety this SHOULD NOT be assessed While 
///< the interrupt is running. Use Thread-safe copy to 
///< Get the current haptic state for graphical display 

GraphicState gs;
char buffer[300];					///< String buffer 
HINSTANCE gThisInst;				///< Instance of Windows application 
Experiment* gExp;					///< Pointer to myExperiment 
Trial* currentTrial;				///< Pointer to current Trial 
bool gKeyPressed;					///< Key pressed? 
char gKey;							///< Which key?
int gNumErrorsBlock = 0;				///< How many erros did you make during a block
int gNumErrors = 0;					///< How many erros did you make in a session
int finger[10];						///< State of each finger
int finger_cross[10];				///< State of each finger with respect to pre-movement threshold
int b = 0;							///< Counter for relative block number with respect to start of session
int gNumCrossesBlock = 0;				///< How many thres crosses did you make during a block
int gNumCrosses = 0;					///< How many thres crosses did you make during a session?
double gNumPointsBlock = 0;			///< How many points in this block?
double gNumPoints = 0;				///< How many points so far in the entire session? 
int timeMet = 0;						///< Initialize time for metronome
int showLine = 0;					///< Show force lines or not
float timeThresPercent = 105;		///< 105% of current median MT (best block)
float superThresPercent = 98;		///< 98% of current median MT (best block)
float ERthreshold = 20;				///< Trheshold of 25% of error rate in order to lower MT thresholds
double timeThreshold = 3500; 			///< Time threshold for normal points (+1)
double timeThresholdSuper = 2940;		///< Time threshold for super points (+3)
double tempThres1 = timeThreshold;
double tempThres2 = timeThresholdSuper;
double responseArray[11] = { 1,1,1,1,1,1,1,1,1,1,1 };


double medianMTarray[50];			///< 24 blocks per subject, preallocate array to keep track of MTs within session
double ERarray[50];					///< 24 blocks per subject, preallocate array to keep track of ERs within session



string TASKSOUNDS[8] = {
	"C:/robotcode/util/wav/ding.wav",			// 0
		"C:/robotcode/util/wav/smb_coin.wav",	// 1
		"C:/robotcode/util/wav/chimes.wav",		// 2
		"C:/robotcode/util/wav/smb_kick.wav",	// 3	
		"C:/robotcode/util/wav/bump.wav",		// 4
		"C:/robotcode/util/wav/chord.wav",		// 5
		"C:/robotcode/util/wav/smb_pipe.wav",	// 6
		"C:/robotcode/util/wav/error.wav"		// 7
};

// timings	 
#define FEEDBACKTIME 1000						// duration of n points feedback on screen
//#define TRIAL_MAX_TIME 7500					// maximum time allowed to complete the sequence (from beginning of trial, includes prepTime + execution time)
//#define EXECUTION_TIME 2500					// maximum time allowed to complete the sequence (from GO cue)
#define IPI_MET_TIME (EXECUTION_TIME/MAX_PRESS)	// between presses - for fMRI

/////////////////////////////
// Imaging-related stuff  //
/////////////////////////////
double TR = 0; // 0|1000	///< What is the TR in the scanner? (1000=simulation 1s TR; 0=wait for scanner)
//char counterSignal = '5';	///< What key simulates the trigger from the scanner? (not working at the moment)	
//int sliceNumber = 42;		///< How many slices do we have?
double endOfRunRest = 10000;///< How long to wait after last trial? (10 s)
/////////////////////////////

///////////
////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the robot and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	gThisInst = hThisInst;
	gExp = new MyExperiment("eir4", "eir4", "C:/data/ExtrinsicIntrinsicRepetition/eir4/");
	//gExp->redirectIOToConsole();

	//tDisp.init(gThisInst,100,200,600,30,9,2,&(::parseCommand)); // STARK
	tDisp.init(gThisInst, 0, 0, 400, 20, 9, 2, &(::parseCommand));
	tDisp.setText("Subj", 0, 0);

	//gScreen.init(gThisInst, -1024, 0, 1920, 1024, &(::updateGraphics)); // CHOMSKY
	gScreen.init(gThisInst, 1920, 0, 1920, 1080, &(::updateGraphics)); // Windows 10 PC
	//gScreen.init(gThisInst, 2200, 0, 1366, 768, &(::updateGraphics)); // Windows 10 PC
	//gScreen.init(gThisInst,1280,0,1280,1024,&(::updateGraphics)); // STARK
	gScreen.setCenter(Vector2D(0, 0));    // In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE)); // cm/pixel

	// initialize s626cards 
	s626.init("c:/robotcode/calib/s626_single.txt");

	if (s626.getErrorState() == 0) {
		cout << "Hello" << endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); // initialize at 200 Hz update rate 
	}

	// high force 1
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/Flatbox1_highforce_LEFT_07-Jun-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/Flatbox1_highforce_RIGHT_31-July-2017.txt");

	// high force 2
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/Flatbox1_highforce2_LEFT_03-Dec-2021.txt");

	// STARK
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/Flatbox1_highforce2_LEFT_12-Feb-2022.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/Flatbox1_highforce2_RIGHT_03-Dec-2021.txt");


	// CHOMSKY
	gBox[0].init(BOX_LEFT, "c:/robotcode/calib/LEFT_lowForce_FlatBox2_24-Jan-2018.txt");
	gBox[1].init(BOX_RIGHT, "c:/robotcode/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");

	// low force
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/flatbox2_lowforce_LEFT_03-Mar-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");

	gBox[0].filterconst = 0.8;
	gBox[1].filterconst = 0.8;

	gTimer.init();

	// initialize TR counter 
	gCounter.init3();
	gCounter.simulate(TR);
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
		//cout << gTimer[4] << " , ";
		if (gTimer[4] > UPDATE_TEXTDISP) {   //currently every 10ms
			currentTrial->updateTextDisplay();
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			gTimer.reset(4);
			//cout << "A";
		};
		InvalidateRect(gScreen.windowHnd, NULL, TRUE);
		UpdateWindow(gScreen.windowHnd);
	} while (msg.message != WM_QUIT);
}

///////////////////////////////////////////////////////////////
// Parse additional commands 
///////////////////////////////////////////////////////////////
bool MyExperiment::parseCommand(string arguments[], int numArgs) {
	int b;
	float arg[4];
	MSG msg;

	/// Print continuously state of the encoders 
	if (arguments[0] == "state") {
		tDisp.keyPressed = 0;
		tDisp.lock();

		while (!tDisp.keyPressed) {
			if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			s626.updateAD(0);

			gBox[0].update();
			gBox[1].update();

			// LEFT
			sprintf(buffer, "Force : %2.2f %2.2f %2.2f %2.2f %2.2f",
				gBox[0].getForce(4), gBox[0].getForce(3), gBox[0].getForce(2), gBox[0].getForce(1), gBox[0].getForce(0));
			// RIGHT
			sprintf(buffer, "Force : %2.2f %2.2f %2.2f %2.2f %2.2f",
				gBox[1].getForce(0), gBox[1].getForce(1), gBox[1].getForce(2), gBox[1].getForce(3), gBox[1].getForce(4));

			tDisp.setText(buffer, 5, 0);
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10);
		}
		tDisp.unlock();
	}

	/// Print continuously state of the encoders 
	else if (arguments[0] == "zeroF") {
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

	/// Set simulated TR duration (default is given by TR in ms)
	else if (arguments[0] == "TR") {
		if (numArgs != 2) {
			tDisp.print("USAGE: TR duration [in ms]");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			TR = arg[0];
			if (TR > 0) {
				gCounter.simulate(arg[0]);  // TR>0  -> simulate trigger (practice sessions) and define TR time (custom duration defined by input TR)
			}
			else {
				gCounter.simulate(0);       // TR<=0 -> wait for trigger from scanner (scanning sessions)
			}
		}
	}

	/// Flip display left-right or up-down 
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

	/// Show the force lines 
	else if (arguments[0] == "showlines") {
		if (numArgs != 2) {
			tDisp.print("USAGE: showlines 0|1");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			if (arg[0] > 0) {
				gs.showlines = true;
			}
			else {
				gs.showlines = false;
			}
		}
	}

	/// Compute/update MT thresholds
	else if (arguments[0] == "thres") {
		if (numArgs != 2) {
			tDisp.print("USAGE: thres medianMT");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			timeThreshold = (arg[0] * (timeThresPercent / 100));
			timeThresholdSuper = (arg[0] * (superThresPercent / 100));
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
	for (int i = 0; i < NUMDISPLAYLINES; i++) { gs.line[i] = ""; }
	gs.boxOn = true;
	gCounter.reset();
	gCounter.start();
	gNumErrorsBlock = 0;
	gNumPointsBlock = 0;
	gNumCrossesBlock = 0;
	gs.clearCues();
}

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state 
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback() {
	gCounter.stop();
	///*
	int i;
	int n = 0; //number of correct trials
	int nn = 0; //number of total trials
	double MTarray[65]; //60 trials per block
	//double medianMTarray;				///< preallocate array to keep track of MTs within session
	double ER;
	MyTrial* tpnr;

	medianMTarray[0] = 20000;	//initialize MT array for the 0th block to be 20000 (impossible value, we have no median MT before that)
	ERarray[0] = 0;			//initialize ER array for the 0th block to be 0 

	
	for (i = 0; i < trialNum; i++) { //check each trial
		tpnr = (MyTrial*)trialVec.at(i);
		if (tpnr->isError == 0 && tpnr->exeType == 1) { //if correct go trial
			MTarray[n] = tpnr->norm_MT; //normalized MT from the correct go trials and add them
			n++; //remember number of correct trials
		}
		nn++; //count total trials
	}

	// before eventual thres update, store the previous thres for writing in the .dat file
	tempThres1 = timeThreshold;
	tempThres2 = timeThresholdSuper;

	if (n > 0) { //if at least one correct trial
		b = b++; //increase counter of block number
		medianMTarray[b] = median(MTarray, n); //median of movement times
		ERarray[b] = (((double)gNumErrorsBlock) / (double)(nn) * 100); //error rate

		
		if ((ERarray[b] <= ERthreshold) && (ERarray[b - 1] >= ERthreshold)) { //if ER on previous block > 20%
			if (medianMTarray[b] < (timeThreshold / (timeThresPercent / 100))) { //adjust only if MT of current block faster than MT that generated current threshold
				timeThreshold = medianMTarray[b] * (timeThresPercent / 100); //previous MT+20%
				timeThresholdSuper = medianMTarray[b] * (superThresPercent / 100); //previous MT-5% 	
			}
		}
		else if ((ERarray[b] <= ERthreshold) && (ERarray[b - 1] <= ERthreshold)) { //if ER on previous block <20%	
			if (medianMTarray[b] < (timeThreshold / (timeThresPercent / 100))) { //adjust only if MT of current block faster than MT that generated current threshold
				timeThreshold = medianMTarray[b] * (timeThresPercent / 100); //previous MT+20%
				timeThresholdSuper = medianMTarray[b] * (superThresPercent / 100); //previous MT-5%
			}
		}
		

	}
	else {
		b = b++;
		medianMTarray[b] = 0;
		ERarray[b] = 100;
	}

	

	// print FEEDBACK on the screen 
	sprintf(buffer, "ER %3.1f%%           MT %2.0fms", ERarray[b], medianMTarray[b]);
	//sprintf(buffer,"ER %3.1f%%", 0);
	gs.line[1] = buffer;
	gs.lineColor[1] = 1;

	sprintf(buffer, "PTS %2.1f", gNumPointsBlock);
	gs.line[2] = buffer;
	gs.lineColor[2] = 1;

	
	
	//sprintf(buffer, "TEST");
	//gs.line[1] = buffer;
	//gs.lineColor[1] = 1;
	
	//gScreen.setCenter(Vector2D(0, 0));    // In cm //0,2
	//gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE)); // cm/pixel
}

///////////////////////////////////////////////////////////////
///	My Trial class contains the main info of how a trial in this experiment is run 
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Constructor 
///////////////////////////////////////////////////////////////
MyTrial::MyTrial() {
	state = WAIT_TRIAL;
	//INIT TRIAL VARIABLE	
	complete = 0;                       // end sequence flag
	isError = 0;							// init error flag
	isCross = 0;							// init threshold cross flag
	numCrosses = 0;						// init how many times pre-mov threshold has been crossed in this trial
	timingError = 0;						// init timing error flag
	seqCounter = 0;						// init the sequence index variable
	norm_MT = 0;						// init normalized MT = (RT + ET)/seqLength
	RT = 0;								// init reaction time
	ET = 0;								// init sequence execution time
	points = 0;							// init points gained
	waitTime = 3000;						// how long to wait for first trial in block
	stimOnsetTime = 100;					// stimulus onset time in free-RT task
	timeStamp = 0;						// when was the pre-movement threshold crossed?
	useMetronome = 1;//0;

	for (int i = 0; i < MAX_PRESS; i++) {
		response[i] = 0;					// finger response
		pressTime[i] = 0;					// press time	
		releaseTime[i] = 0;				// release time
		press[i] = 0;                     // fingers need to be pressed
		handPressed[i] = 0;               // 

	}
}

///////////////////////////////////////////////////////////////
// Read - Done
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	in >> SN >> mask >> hand >> load >> show >> coord >> exeType >> cueType
		>> seqNum >> cuePress >> isExtrinsic >> isIntrinsic >> isRepetition
		>> handTrans >> cueMask >> cueTime >> prepTime >> exeTime >> iti
		>> startTime >> fixed_dur;
	seqLength = cuePress.length(); //get seqLength	 
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	// write to .dat file
	out << SN << "\t" << mask << "\t" << hand << "\t" << load << "\t" << show << "\t" << coord << "\t" << exeType << "\t" << cueType << "\t"
		<< seqNum << "\t" << cuePress << "\t" << isExtrinsic << "\t" << isIntrinsic << "\t" << isRepetition << "\t" << handTrans << "\t"
		<< cueMask << "\t" << cueTime << "\t" << prepTime << "\t" << exeTime << "\t" << iti << "\t" << startTime << "\t" << fixed_dur << "\t";
	int i;

	for (i = 0; i < MAX_PRESS; i++) {
		out << press[i] << "\t";
	}

	out << RT << "\t"
		<< ET << "\t"
		<< norm_MT << "\t"
		<< isError << "\t"
		<< timingError << "\t"
		<< points << "\t";

	for (i = 0; i < MAX_PRESS; i++) {
		out << response[i] << "\t";
	}

	for (i = 0; i < MAX_PRESS; i++) {
		out << handPressed[i] << "\t";
	}

	for (i = 0; i < MAX_PRESS; i++) {
		out << pressTime[i] << "\t";
	}

	out << tempThres1 << "\t"
		<< tempThres2 << "\t"
		<< startTime << "\t"
		<< startTimeReal << "\t"
		<< trialDur << "\t"
		<< startTR << "\t"
		<< startTRtime << "\t"
		<< useMetronome << "\t"
		<< isCross << "\t";			//whether pre-movement threshold has been crossed in this trial

	out << tempThres1 << "\t"
		<< tempThres2 << "\t"
		<< endl;
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) {
	char header[200];
	out << "SN" << "\t" << "mask" << "\t" << "hand" << "\t" << "load" << "\t" << "show" << "\t" << "coord" << "\t" << "exeType" << "\t" << "cueType" << "\t"
		<< "seqNum" << "\t" << "cuePress" << "\t" << "isExtrinsic" << "\t" << "isIntrinsic" << "\t" << "isRepetition" << "\t" << "handTrans" << "\t"
		<< "cueMask" << "\t" << "cueTime" << "\t" << "prepTime" << "\t" << "exeTime" << "\t" << "iti" << "\t" << "startTime" << "\t" << "fixed_dur" << "\t";

	int i;


	for (i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "press%d", i + 1);
		out << header << "\t";
	}

	out << "RT" << "\t"
		<< "ET" << "\t"
		<< "norm_MT" << "\t"
		<< "isError" << "\t"
		<< "timingError" << "\t"
		<< "points" << "\t";

	for (i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "response%d", i + 1);
		out << header << "\t";
	}

	for (i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "handPressed%d", i + 1);
		out << header << "\t";
	}


	for (i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "pressTime%d", i + 1);
		out << header << "\t";
	}

	out << "timeThreshold" << "\t"
		<< "timeThresholdSuper" << "\t"
		<< "startTime" << "\t"
		<< "startTimeReal" << "\t"
		<< "trialDur" << "\t"
		<< "startTR" << "\t"
		<< "startTRtime" << "\t"
		<< "useMetronome" << "\t"
		<< "isCross" << "\t";

	out << "timeThreshold" << "\t"
		<< "timeThresholdSuper" << "\t"
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
	sprintf(buffer, "startTime: %d   TR: %2.0f", startTime, TR);
	tDisp.setText(buffer, 1, 0);

	sprintf(buffer, "time: %2.2f   TRtime: %d   slice:%d   metronome: %d ", gCounter.readTotTime(), gCounter.readTR(), gCounter.readSlice(), timeMet);
	tDisp.setText(buffer, 2, 0);

	sprintf(buffer, "gTimer1: %2.2f   gTimer2: %2.2f   gTimer5: %2.2f", gTimer[1], gTimer[2], gTimer[5]);
	tDisp.setText(buffer, 3, 0);

	sprintf(buffer, "upper Threshold: %2.0f   lower Threshold: %2.0f", timeThreshold, timeThresholdSuper);
	tDisp.setText(buffer, 4, 0);

	sprintf(buffer, "trial: %d/%d   state: %d   seqNum: %d", gExp->theBlock->trialNum + 1, gExp->theBlock->numTrials, state, seqNum);
	tDisp.setText(buffer, 5, 0);



	sprintf(buffer, "press LH: %d %d %d %d %d    force LH: %2.2f %2.2f %2.2f %2.2f %2.2f", finger[0], finger[1], finger[2], finger[3], finger[4], gBox[0].getForce(0), gBox[0].getForce(1), gBox[0].getForce(2), gBox[0].getForce(3), gBox[0].getForce(4));
	tDisp.setText(buffer, 6, 0);

	//sprintf(buffer, "press RH: %d %d %d %d %d    force RH: %2.2f %2.2f %2.2f %2.2f %2.2f", finger[5], finger[6], finger[7], finger[8], finger[9], gBox[1].getForce(0), gBox[1].getForce(1), gBox[1].getForce(2), gBox[1].getForce(3), gBox[1].getForce(4));
	//tDisp.setText(buffer, 7, 0);

	sprintf(buffer, "pressTime1: %2.0f   pressTime2: %2.0f   pressTime3   pressTime4: %2.0f   pressTime5: %2.0f", pressTime[0], pressTime[1], pressTime[2], pressTime[3], pressTime[4]);
	tDisp.setText(buffer, 7, 0);


	sprintf(buffer, "prepTime: %d   seqCounter: %d   seqLength: %d", prepTime, seqCounter, seqLength);
	tDisp.setText(buffer, 11, 0);

	sprintf(buffer, "isError: %d   errors block: %d   points block: %2.1f", isError, gNumErrorsBlock, gNumPointsBlock);
	tDisp.setText(buffer, 12, 0);



	sprintf(buffer, "newPress: %d   released: %d", newPress, released);
	tDisp.setText(buffer, 14, 0);

}

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD 
///////////////////////////////////////////////////////////////

// force thresholds 
#define preTH 1.5				// Press threshold
#define relTH 0.8//1.0//1.0		// Release threshold
#define baseTHhi  0.5 //0.8//1.0		// Baseline higher threshold (to check for premature movements during sequence planning phase)
#define baseTHlow 0 //0.8//1.0		// Baseline lower threshold (to check for premature movements during sequence planning phase)

double THRESHOLD[2][5] = { {preTH, preTH, preTH, preTH, preTH}, {relTH, relTH, relTH, relTH, relTH} };
double BASE_THRESHOLD_HI[2][5] = { {baseTHhi, baseTHhi, baseTHhi, baseTHhi, baseTHhi}, {baseTHhi - 0.1, baseTHhi - 0.1, baseTHhi - 0.1, baseTHhi - 0.1, baseTHhi - 0.1} };
double BASE_THRESHOLD_LOW[2][5] = { {baseTHlow, baseTHlow, baseTHlow, baseTHlow, baseTHlow}, {baseTHlow + 0.1, baseTHlow + 0.1, baseTHlow + 0.1, baseTHlow + 0.1, baseTHlow + 0.1} };
double fGain[5] = { 1.0,1.0,1.0,1.0,1.0 };

// finger graphics
#define FINGWIDTH 1.3
#define N_FINGERS 5
#define FINGER_SPACING 0.2
#define FORCESCALE 2
#define BASELINE_X1 -(FINGWIDTH*N_FINGERS/2)
#define BASELINE_X2 +(FINGWIDTH*N_FINGERS/2)
#define BASELINE_Y1 0
#define BASELINE_Y2 BASELINE_Y1

// text
char TEXT[5] = { '1','2','3','4','5' };
#define CUE_PRESS_yPOS 5
#define SIZE_CUE 15
#define WIDTH_CHAR_CUE 2
#define OTHER_LETTERS_SIZE 1.5

// cuePress rectangle 
#define RECWIDTH_X WIDTH_CHAR_CUE*MAX_PRESS
#define RECWIDTH_Y WIDTH_CHAR_CUE
#define REC_xPOS 0
#define REC_yPOS CUE_PRESS_yPOS + 0.75

// metronome line
#define MET_LINE_X1 -(MAX_PRESS/2) * WIDTH_CHAR_CUE
#define MET_LINE_X2 -(MET_LINE_X1)
#define MET_LINE_Y1 4.3
#define MET_LINE_Y2 MET_LINE_Y1

void MyTrial::updateGraphics(int what) {
	int i;

	//--------------------------
	if (gs.showlines == 1) {
		if (state > -1 && state < 8) {

			// Baseline box
			gScreen.setColor(Screen::darkred);
			// right
			gScreen.drawBox(FINGWIDTH * N_FINGERS, (baseTHhi - baseTHlow) * FORCESCALE, -0 * FINGWIDTH * N_FINGERS, (baseTHlow * FORCESCALE) + ((baseTHhi - baseTHlow) * FORCESCALE) / 2);
			// left
			//gScreen.drawBox(FINGWIDTH*N_FINGERS, (baseTHhi-baseTHlow)*FORCESCALE, -0.75*FINGWIDTH*N_FINGERS, (baseTHlow*FORCESCALE)+((baseTHhi-baseTHlow)*FORCESCALE)/2);


			gScreen.setColor(Screen::grey);
			// right Hi
			//gScreen.drawLine(BASELINE_X1-0.75*(FINGWIDTH*N_FINGERS), baseTHhi*FORCESCALE, BASELINE_X2-0.75*(FINGWIDTH*N_FINGERS), baseTHhi*FORCESCALE);
			// left Hi
			gScreen.drawLine(BASELINE_X1 + 0 * (FINGWIDTH * N_FINGERS), baseTHhi * FORCESCALE, BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), baseTHhi * FORCESCALE);


			// right Lo
			//gScreen.drawLine(BASELINE_X1-0.75*(FINGWIDTH*N_FINGERS), baseTHlow*FORCESCALE, BASELINE_X2-0.75*(FINGWIDTH*N_FINGERS), baseTHlow*FORCESCALE);
			// left Lo
			gScreen.drawLine(BASELINE_X1 + 0. * (FINGWIDTH * N_FINGERS), baseTHlow * FORCESCALE, BASELINE_X2 + 0 * (FINGWIDTH * N_FINGERS), baseTHlow * FORCESCALE);


			// Force threshold		
			gScreen.setColor(Screen::grey);
			gScreen.drawLine(1. * BASELINE_X1, preTH * FORCESCALE + BASELINE_Y1, 1. * BASELINE_X2, preTH * FORCESCALE + BASELINE_Y2);

			// Finger forces (right)
			for (i = 0; i < 5; i++) {
				gScreen.setColor(Screen::white);
				gScreen.drawLine(((i * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) + FINGER_SPACING, gBox[1].getForce(i) * FORCESCALE + BASELINE_Y1, (((i + 1) * FINGWIDTH) - 0.5 * (FINGWIDTH * N_FINGERS)) - FINGER_SPACING, gBox[1].getForce(i) * FORCESCALE + BASELINE_Y2);
			}
			/*
			// Finger forces (left)
			for (i=0;i<5;i++) {
				gScreen.setColor(Screen::white);
				gScreen.drawLine(((i*FINGWIDTH)-1.25*(FINGWIDTH*N_FINGERS))+FINGER_SPACING, gBox[0].getForce(4-i)*FORCESCALE+BASELINE_Y1, (((i+1)*FINGWIDTH)-1.25*(FINGWIDTH*N_FINGERS))-FINGER_SPACING, gBox[0].getForce(4-i)*FORCESCALE+BASELINE_Y2);
			}
			*/
		}
	}

	// Other letters
	gScreen.setColor(Screen::white);
	for (i = 0; i < NUMDISPLAYLINES; i++) {
		if (!gs.line[i].empty()) {
			gScreen.setColor(gs.lineColor[i]);
			gScreen.print(gs.line[i].c_str(), gs.lineXpos[i], gs.lineYpos[i], gs.size[i] * OTHER_LETTERS_SIZE);
		}
	}

	// Press Cue
	gScreen.setColor(1);		// White
	for (i = 0; i < MAX_PRESS; i++) {
		if (gs.cuePress[i] > 0) { // Numbers
			gScreen.setColor(responseArray[i]);
			gScreen.printChar(gs.cuePress[i], (i - ((double)(MAX_PRESS / 2) - 0.5)) * WIDTH_CHAR_CUE + WIDTH_CHAR_CUE * ((double)(MAX_PRESS - seqLength) / 2), CUE_PRESS_yPOS, SIZE_CUE);
		}
		if (gs.cueMask[i] > 0) { // Mask (asterisk)
			gScreen.setColor(responseArray[i]);
			gScreen.printChar(gs.cueMask[i], (i - ((double)(MAX_PRESS / 2) - 0.5)) * WIDTH_CHAR_CUE + WIDTH_CHAR_CUE * ((double)(MAX_PRESS - seqLength) / 2), CUE_PRESS_yPOS, SIZE_CUE);
		}
	}

	Vector2D recSize, recPos;
	// press rectangle
	if ((state >= 0 && state <= 6) || state == 7) {
		gScreen.setColor(Screen::grey);
		gScreen.drawRect(RECWIDTH_X, RECWIDTH_Y, REC_xPOS, REC_yPOS);

		//----------------------------------
		// DELAYED-MOVEMENT GO-NOGO PARADIGM
		if ((state >= 3 && state <= 5) && gTimer[1] >= (prepTime + cueTime)) { // go/nogo after preparatory period defined by prepTime

			if (exeType == 1) { // GO 
				//gScreen.setColor(Screen::oceanblue);
				//gScreen.drawRect(RECWIDTH_X + 0.3, RECWIDTH_Y + 0.3, REC_xPOS, REC_yPOS);
				gScreen.setColor(Screen::black);
				gScreen.drawRect(RECWIDTH_X, RECWIDTH_Y, REC_xPOS, REC_yPOS);
				gScreen.setColor(Screen::white);
			}
			else if (exeType == 0) { // NO-GO
				gScreen.setColor(Screen::darkorange);
				gScreen.drawRect(RECWIDTH_X + 0.3, RECWIDTH_Y + 0.3, REC_xPOS, REC_yPOS);
				gScreen.setColor(Screen::white);
			}
		}
	}

	if (useMetronome == 1 && state == 3 && timeMet == 0) {
		gScreen.setColor(Screen::white);
		gScreen.drawLine(MET_LINE_X1, MET_LINE_Y1, MET_LINE_X2, MET_LINE_Y2);
	}
	else if (useMetronome == 1 && state == 4 && timeMet <= MAX_PRESS) {
		gScreen.setColor(Screen::white);
		gScreen.drawLine(MET_LINE_X1, MET_LINE_Y1, MET_LINE_X2, MET_LINE_Y2);
		gScreen.setColor(Screen::black);
		gScreen.drawLine(MET_LINE_X1, MET_LINE_Y1, MET_LINE_X1 + gTimer[2] / (exeTime / MAX_PRESS) * (WIDTH_CHAR_CUE), MET_LINE_Y2);
	}
	else if (useMetronome == 1 && state == 4 && timeMet > MAX_PRESS) {
		gScreen.setColor(Screen::black);
		gScreen.drawLine(MET_LINE_X1, MET_LINE_Y1, MET_LINE_X2, MET_LINE_Y2);
	}

	// Debugging tool to visualize current state of program control
	/*
	string stateString;
	switch(state)
	{
	case WAIT_TRIAL:
	stateString = "0. WAIT TRIAL";		// 0
	break;
	case START_TRIAL:
	stateString = "1. START TRIAL";		// 1
	break;
	case WAIT_TR:
	stateString = "2. WAIT TR";			// 2
	break;
	case WAIT_PREP:
	stateString = "3. WAIT PREP";		// 3
	break;
	case WAIT_PRESS:
	stateString = "4. WAIT PRESS";		// 4
	break;
	case WAIT_RELEASE:
	stateString = "5. WAIT RELEASE";	// 5
	break;
	case WAIT_FEEDBACK:
	stateString = "6. WAIT FEEDBACK";	// 6
	break;
	case WAIT_ITI:
	stateString = "7. WAIT ITI";		// 7
	break;
	case END_TRIAL:
	stateString = "8. END TRIAL";		// 8
	break;
	}
	gScreen.print(stateString,0,9,5);
	//*/
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
	if (dataman.isRecording() && gTimer[3] >= RECORDRATE) {
		gTimer.reset(3);
		bool x = dataman.record(DataRecord(state));
		if (!x) {
			dataman.stopRecording();
		}
	}
}

//////////////////////////////////////////////////////////////////////
// control Trial: A state-driven routine to guide through the process of a trial 
//////////////////////////////////////////////////////////////////////
void MyTrial::control() {
	int i, f;
	Vector2D recSize;
	Vector2D recPos;
	double force;
	released = 0;							// init whether fingers released
	pressedFinger = 0;
	pressedHand = 0;
	newPress = 0;						// is there a new press?

	int crossedFinger = 0;
	int numNewThresCross = 0;			// has the pre-movement trheshold been crossed?
	int withinThres = 1;
	//int FalseAlarm = 0;			// has the false alarm trheshold been crossed?

	for (f = 5; f < 10; f++) {
		if (f < 5) { // left hand
			if (coord == 2) {
				force = gBox[0].getForce(4 - f);
				if (finger[4 - f] == 0 && force > THRESHOLD[0][4 - f]) { // Press threshold comparison
					newPress++;
					pressedFinger = (4 - f) + 1;
					pressedHand = 1;
					finger[4 - f] = 1;
					released = 0;
				}
				if (force <= THRESHOLD[1][4 - f]) { // Release threshold comparison
					finger[4 - f] = 0;
					released++;
				}
			}
			else if (coord == 1) {
				force = gBox[0].getForce(4 - f);
				if (finger[f] == 0 && force > THRESHOLD[0][f]) { // Press threshold comparison
					newPress++;
					pressedFinger = f + 1;
					pressedHand = 1;
					finger[f] = 1;
					released = 0;
				}
				if (force <= THRESHOLD[1][f]) { // Release threshold comparison
					finger[f] = 0;
					released++;
				}
			}
		}
		else {
			force = gBox[1].getForce(f - 5);
			if (finger[f] == 0 && force > THRESHOLD[0][f - 5]) { // Press threshold comparison
				newPress++;
				pressedFinger = (f - 5) + 1;
				pressedHand = 2;
				finger[f] = 1;
				released = 0;
			}
			if (force <= THRESHOLD[1][f - 5]) { // Release threshold comparison
				finger[f] = 0;
				released++;
			}
		}
	}

	switch (state) {

	case WAIT_TRIAL: //0
		// Initialize timers
		gTimer.reset(1); // overall trial timer (pressTime)	
		gTimer.reset(2); // timer from go cue (RT)
		gTimer.reset(5); // timer from first press (ET)
		//	gs.reset();
		for (i = 0; i < NUMDISPLAYLINES; i++) {
			if (!gs.line[i].empty()) {
				gs.lineColor[i] = 0;
				gs.line[i] = "";
			}
		}
		gs.clearCues();
		break;

	case START_TRIAL: //1		
		trialDur = 0;
		for (i = 0; i < MAX_PRESS; i++) {
			response[i] = 0;
			pressTime[i] = 0;
			releaseTime[i] = 0;
		}
		dataman.clear();
		timeMet = 0; //reset metronome for next trial

		for (i = 0; i < NUMDISPLAYLINES; i++) {
			gs.line[i] = ""; // clear screen
		}

		// present cuePress (visual cues)
		if (startTime > 0) {							// use externally imposed timing (follow startTime, for scanning)
			state = WAIT_TR;
		}
		else {									// use natural experiment timing (behavioral version only)
			startTimeReal = -1;
			startTR = -1;
			startTRtime = -1;
			if ((gExp->theBlock->trialNum + 1) == 1) {	// if first trial of the block 
				if (gTimer[1] >= waitTime) {			// wait waitTime before presenting cuePress
					for (i = 0; i < seqLength; i++) {
						press[i] = cuePress.at(i) - '0';
						gs.cuePress[i] = cuePress.at(i);
						if (show == 1) {
							if (hand == 2) {
								responseArray[i] = 6; // orange for the right
							}
							else {
								responseArray[i] = 9; // light blue for the left
							}
						}
					}
					gTimer.reset(1); gTimer.reset(2); gTimer.reset(5);
					dataman.startRecording();
					state = WAIT_PREP;
				}
			}
			else { // every other trial of the block
				if (gTimer[1] >= stimOnsetTime) {		// wait stimOnsetTime before presenting cuePress
					for (i = 0; i < seqLength; i++) {
						press[i] = cuePress.at(i) - '0';
						gs.cuePress[i] = cuePress.at(i);
						if (show == 1) {
							if (hand == 2) {
								responseArray[i] = 6; // orange for the right
							}
							else {
								responseArray[i] = 9; // light blue for the left
							}
						}
					}
					gTimer.reset(1); gTimer.reset(2); gTimer.reset(5);
					dataman.startRecording();
					state = WAIT_PREP;
				}
			}
		}
		break;

	case WAIT_TR: //2
		if (gCounter.readTR() > 0 && gCounter.readTotTime() >= startTime) { // use externally imposed timing (follow startTime)
			startTimeReal = gCounter.readTotTime();
			startTR = gCounter.readTR();
			startTRtime = gCounter.readTime();
			// present cuePress (visual cues)
			for (i = 0; i < seqLength; i++) {
				press[i] = cuePress.at(i) - '0';
				gs.cuePress[i] = cuePress.at(i);
				if (show == 1) {
					if (hand == 2) {
						responseArray[i] = 6; // orange for the right
					}
					else {
						responseArray[i] = 9; // light blue for the left
					}
				}
			}
			gTimer.reset(1); gTimer.reset(2); gTimer.reset(5);
			dataman.startRecording();
			state = WAIT_PREP;
		}
		break;

	case WAIT_PREP: //3 
		//----------------------------------
		// CHECK FOR BASELINE FINGER FORCES
		if (numNewThresCross > 0 && gTimer[1] < (cueTime + prepTime)) { // check for pre-movement finger presses
			timeStamp = gTimer[1];
			isCross = 1;
			if (startTime < 0) { // display warning message (practice blocks only)
				sprintf(buffer, "Please remain within the red area");
				gs.lineColor[0] = 1;
				gs.line[0] = buffer;
				gs.lineYpos[0] = 8;
			}
		}
		/*
		if (gTimer[1]>timeStamp+500) {
		sprintf(buffer,"");
		gs.lineColor[0] = 1;
		gs.line[0] = buffer;
		gs.lineYpos[0] = 8;
		}
		//*/

		// DELAYED-MOVEMENT GO-NOGO PARADIGM
		if (newPress > 0 && gTimer[1] < (cueTime + prepTime)) { // check for error: go cue anticipation
			response[seqCounter] = pressedFinger;
			handPressed[seqCounter] = pressedHand;
			pressTime[seqCounter] = gTimer[1];
			RT = gTimer[1] - (cueTime + prepTime);	// anticipation: negative reaction time
			ET = 0;								// execution time
			norm_MT = (RT + ET);		// normalized movement time, in this exp defined as (RT + ET) (what gets rewarded)
			isError = 1;
			timingError = 1;
			gTimer.reset(5);
			sprintf(buffer, "TOO EARLY");
			gs.lineColor[0] = 1;
			gs.line[0] = buffer;
			gs.lineYpos[0] = 8;
			isError = 1;
			timingError = 1;
			responseArray[seqCounter] = 2; // red
			seqCounter++;
			state = WAIT_RELEASE;
		}

		if (gTimer[1] >= cueTime && mask == 1) { // mask at delay
			gs.clearCues();
			for (i = 0; i < seqLength; i++) {
				gs.cueMask[i] = cueMask.at(i);
			}
		}

		if (gTimer[1] >= (cueTime + prepTime)) { // give go/nogo signal

			sprintf(buffer, "");
			gs.lineColor[0] = 1;
			gs.line[0] = buffer;
			gs.lineYpos[0] = 8;

			if (mask == 2) {      // only mask at Go time
				gs.clearCues();
				for (i = 0; i < seqLength; i++) {
					gs.cueMask[i] = cueMask.at(i);
				}
			}

			if (exeType == 1) { // GO 
				// PLAY SOUND
				PlaySound(TASKSOUNDS[0].c_str(), NULL, SND_ASYNC);
			}
			else if (exeType == 0) { // NO-GO
				// PLAY SOUND
				//PlaySound(TASKSOUNDS[7].c_str(), NULL, SND_ASYNC);
			}
			gTimer.reset(2);
			state = WAIT_PRESS;
		}
		break;

	case WAIT_PRESS: //4
		//----------------------------------
		// REVEAL HAND AT GO CUE ONLY
		for (i = 0; i < seqLength; i++) {
			if (show == 2) {
				if (hand == 2) {
					responseArray[i] = 6; // orange for the right
				}
				else {
					responseArray[i] = 9; // light blue for the left
				}
			}
		}

		if (useMetronome > 0 && gTimer[2] > timeMet * (exeTime / MAX_PRESS) && timeMet < MAX_PRESS + 1) {
			timeMet++;	// update counter
		};

		// Wait for a key press
		if (exeType == 1) { // GO TRIAL

			// START OF SEQUENCE
			if (newPress > 0 && seqCounter < seqLength) { // correct timing
				response[seqCounter] = pressedFinger;
				handPressed[seqCounter] = pressedHand;
				pressTime[seqCounter] = gTimer[1];
				if (seqCounter == 0) {			// if first press 
					RT = gTimer[2];
					gTimer.reset(5);
				}
				if (response[seqCounter] == press[seqCounter] && handPressed[seqCounter] == hand) { // correct press	
					responseArray[seqCounter] = 3; // green
				}
				else { // error: wrong key pressed
					responseArray[seqCounter] = 2; // red
					isError = 1;
				};
				seqCounter++;
			}

			// END OF SEQUENCE: get execution time and movement time
			if (seqCounter >= seqLength && released == NUMFINGERS) {
				if (complete == 0) {
					ET = gTimer[5]; // execution time
					norm_MT = (RT + ET);
					complete = 1;
					gTimer.reset(5);
				}
				if (fixed_dur == 1) { // fixed trial duration: wait exeTime before moving on to wait release (same time for GO and NOGO trials)
					if (gTimer[2] >= exeTime) {
						state = WAIT_RELEASE;
					}
				}
				else { // flexible trial duration: move on to next trial, just wait an extra 200ms to make color feedback visible
					if (gTimer[5] >= 200) {
						state = WAIT_RELEASE;
					}
				}

				// SEQUENCE TIME OUT
			}
			else if (gTimer[2] >= exeTime) { // time out (if sequence not completed in time)
				RT = RT;				// reaction time
				ET = prepTime + exeTime;	// execution time
				norm_MT = (RT + ET);
				isError = 1;
				timingError = 1;
				// PLAY SOUND 
				//PlaySound(TASKSOUNDS[6].c_str(), NULL, SND_ASYNC);
				gs.clearCues(); sprintf(buffer, "TOO SLOW");
				gs.lineColor[0] = 1;
				gs.line[0] = buffer;
				gs.lineYpos[0] = 8;
				gTimer.reset(5);
				state = WAIT_RELEASE;
			}
		}

		else if (exeType == 0) { // NO-GO TRIAL
			if (numNewThresCross > 0 && seqCounter < seqLength) { // wrong timing
				sprintf(buffer, "HAD TO STAY");
				gs.lineColor[0] = 1;
				gs.line[0] = buffer;
				gs.lineYpos[0] = 8;
				response[seqCounter] = pressedFinger;
				handPressed[seqCounter] = pressedHand;
				pressTime[seqCounter] = gTimer[1];
				RT = gTimer[2];	// reaction time
				ET = 0;			// execution time
				norm_MT = (RT + ET);
				gTimer.reset(5);
				isError = 1;
				timingError = 1;
				responseArray[seqCounter] = 2; // red
				seqCounter++;
				state = WAIT_RELEASE;
			}
			if (gTimer[2] >= exeTime) { // wait exeTime before moving on to wait release (same time for GO and NOGO trials) 
				if (isError == 0) {
					isError = 0;
					timingError = 0;
					RT = 0;			// reaction time
					ET = 0;			// execution time
					norm_MT = (RT + ET);
					gTimer.reset(5);

				}
				state = WAIT_RELEASE;
			}
		}
		break;

	case WAIT_RELEASE: //5
		// Wait for the release of all keys, assign points
		if (released == NUMFINGERS) {

			if (exeType == 1) { // GO

				if (isError == 0) {

					// assign points
					//norm_MT = (RT + ET);

					if (norm_MT < timeThresholdSuper) {
						points = 3;
					}
					else if (norm_MT < timeThreshold) {
						points = 2;
					}
					else{
						points = 1;
					}

					gs.clearCues();
					sprintf(buffer, "+%d", points);
					gs.lineColor[1] = 1; // white
					gs.line[1] = buffer; gs.lineYpos[1] = 5.4;
					

				}
				else if (isError == 1 && timingError == 0) {
					points = 0;
					// PLAY SOUND 
					//PlaySound(TASKSOUNDS[5].c_str(), NULL, SND_ASYNC);
					gs.clearCues(); sprintf(buffer, "%d", points);
					gs.lineColor[1] = 1; // white
					gs.line[1] = buffer; gs.lineYpos[1] = 5.4;
				}
				else if (isError == 1 && timingError == 1) {
					points = -1;
					// PLAY SOUND 
					//PlaySound(TASKSOUNDS[6].c_str(), NULL, SND_ASYNC);
					gs.clearCues(); sprintf(buffer, "%d", points);
					gs.lineColor[1] = 1; // white
					gs.line[1] = buffer; gs.lineYpos[1] = 5.4;
				}
				gTimer.reset(2);
				state = WAIT_FEEDBACK;
			}

			else if (exeType == 0) { // NO-GO

				if (isError == 0) {
					points = 1;
					gs.clearCues(); sprintf(buffer, "+%d", points);
					gs.lineColor[1] = 1; // white
					gs.line[1] = buffer; gs.lineYpos[1] = 5.4;
				}
				else {
					points = -1;
					// PLAY SOUND 
					//PlaySound(TASKSOUNDS[6].c_str(), NULL, SND_ASYNC);
					gs.clearCues(); sprintf(buffer, "+%d", points);
					gs.lineColor[1] = 1; // white
					gs.line[1] = buffer; gs.lineYpos[1] = 5.4;
				}

				gTimer.reset(2);
				state = WAIT_FEEDBACK;
			}

		}
		break;

	case WAIT_FEEDBACK: //6
		//keep feedback on screen for specified time
		for (i = 0; i < MAX_PRESS; i++) {
			responseArray[i] = 1; // white
		}

		// Wait for feedback time, keep count of ponits
		if (gTimer[2] > FEEDBACKTIME) {
			// count how many points and errors in the block
			gNumPointsBlock += (double)points;
			gNumCrossesBlock += isCross;
			gNumErrorsBlock += isError;
			gTimer.reset(2);
			state = WAIT_ITI;
		}
		break;

	case WAIT_ITI: //7
		for (i = 0; i < NUMDISPLAYLINES; i++) { // clear screen
			if (!gs.line[i].empty()) {
				gs.lineColor[i] = 0;
				gs.line[i] = "";
			}
		}
		gs.clearCues();
		// Implement inter-trial interval (ITI) and end of run rest (if externally imposed timing only)							
		if (startTime > 0 && (gExp->theBlock->trialNum + 1) >= (gExp->theBlock->numTrials)) {
			if (gTimer[2] > (iti - FEEDBACKTIME + endOfRunRest)) { // for scanner only (wait rest periord before presenting block results)
				dataman.stopRecording();
				trialDur = gTimer[1];
				state = END_TRIAL;
			}
		}
		else {
			if (gTimer[2] > (iti - FEEDBACKTIME)) {
				dataman.stopRecording();
				trialDur = gTimer[1];
				state = END_TRIAL;
			}
		}
		break;

	case END_TRIAL: //8
		// End of trial
		gs.clearCues();
		gs.lineColor[0] = 0;
		break;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
/// Data Record: creator records the current data from the device 
/////////////////////////////////////////////////////////////////////////////////////
DataRecord::DataRecord(int s) {
	int i;
	state = s;
	time = gTimer[1];
	timeReal = gTimer.getRealtime();

	for (i = 0; i < 5; i++) {
		force_left[i] = gBox[0].getForce(i);
		force_right[i] = gBox[1].getForce(i);
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream& out) {
	out << state << "\t" << timeReal << "\t" << time << "\t"
		<< force_right[0] << "\t" << force_right[1] << "\t" << force_right[2] << "\t" << force_right[3] << "\t" << force_right[4] << "\t" << endl;
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

	clearCues();

	boxOn = false;
	showlines = true;
}

void GraphicState::clearCues(void) {
	int i;
	for (i = 0; i < MAX_PRESS; i++) {
		cuePress[i] = 0;
		cueMask[i] = 0;
	}
}

void GraphicState::reset(void) {
	for (int i = 0; i < NUMDISPLAYLINES; i++) {
		line[i] = "";
	}
}