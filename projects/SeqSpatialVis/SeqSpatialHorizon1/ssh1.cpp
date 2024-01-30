
///////////////////////////////////////////////////////////////
///
/// Sequence Learning and Eye Tracking
///////////////////////////////////////////////////////////////

#include "ssh1.h"
#include "StimulatorBox.h"
#include "Target.h"
#include <ctime>
///////////////////////////////////////////////////////////////
/// Global variables
///
///////////////////////////////////////////////////////////////
S626sManager s626; ///< Hardware Manager
TextDisplay tDisp; ///< Text Display
Screen gScreen; ///< Screen
StimulatorBox gBox[2]; ///< Stimulator Box
Target gTarget(SHAPE_DISC, 1); // Draw a white box for visual target, SKim
Target gHorizon(SHAPE_BOX, 1); // Draw a white box for visual target, SKim


Timer gTimer(UPDATERATE); ///< Timer from S626 board experiments
/// Usually the timers are used in the following fashion
/// \li 0: Timer that gives you the time from the start of the last block
/// \li 1: Timer that gives you the time from the start of the current trial
/// \li 2: Flexible event timer that you can use in MyTrial::control() to stop the time since the last event.
/// \li 3: USed by screen to keep track of the screen refresh rate
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

Vector2D fixationPos; ///< position of the fixation cross
#define CALIB_NUM 9             /// present 5 stimuli for eye calibration
# define EYE_MISSING 30000
bool calib_mode = false;
int curr_calib;
int refPointNumX = 0; // Basically, this is 0(central fixation)
int jumpPointNumX = 1; // Basically, this is fixation jump to rightward
int refPointNumY = 0; // Basically, this is 0(central fixation)
int jumpPointNumY = 2; // Basically, this is fixation jump to upward

# define TRINUMforMEDIAN 30// 30  ///
double gSaccadeTimeR[TRINUMforMEDIAN];
double gSaccadeTimeL[TRINUMforMEDIAN];
double gSaccadeTimeAll[TRINUMforMEDIAN * 2];
double prctileLevel[] = { 20.0,45.0,50.0,71.0 };
///____________________________________________________ Neda-End


char buffer[300]; ///< String buffer
HINSTANCE gThisInst; ///< defined in Experiment.cpp Instance of Windows application
Experiment* gExp; ///< defined in Experiment.cpp Pointer to myExperiment
Trial* currentTrial; ///< defined in Experiment.cpp Pointer to current Trial
bool gKeyPressed; ///< Key pressed?
char gKey; ///< Which key?
int gNumErrors = 0; ///< How many erros did you make during a block
int finger[5]; ///< State of each finger
int gNumPointsBlock = 0;
int gNumPoints = 0;
int gTrial = 0;

float timeThresPercent = 110; ///< 120% of current median MT (previous block)
float superThresPercent = 95; ///< 95% of current median MT (previous block)
float ERthreshold = 15; ///< Trheshold of 20% of error rate in order to lower MT thresholds
double medianMTarray[4][100]; ///< Initialise MT array across blocks (never more than 100 blocks)
double ERarray[100]; ///< Initialise ER array across blocks
int b = 0;

double timeThreshold[4] = { 800,1100,1500,6000 }; ///< Double and triple chunk time threshold
double timeThresholdSuper[4] = { 320,560,740,5000 }; ///< Time threshold for super points
#define FEEDBACKTIME 1500    // time for which the points of the trial is displayed at the end of a trial
// Neda increased feedback time so that the subject has time to blink
string FINGERSOUND[6] = { "A.wav", "C.wav", "D.wav", "E.wav", "G.wav" };
//string TASKSOUNDS[5] = { "../../util/wav/smb_kick.wav",
//"../../util/wav/smb_bump.wav",
//"../../util/wav/smb_1-up.wav",
//"../../util/wav/smb_coin.wav",
//"../../util/wav/perc2.wav" };

string TASKSOUNDS[2] = { "wav/chord.wav",
"wav/smb_coin.wav"
};

char TEXT[5] = { '1','2','3','4','5' };
#define CUE_SEQ 6
#define CUE_CHUNK 4.5
#define CUE_PRESS 2.3 // the Y position of the presses on the screen
#define SIZE_CUE 10    // the font size of presses
#define WIDTH_CHAR_CUE 2 // the distance between letters
#define WIDTH_REC_CUE 6 // SKim
#define HEIGHT_REC_CUE 3 // SKim

// Force Thresholds
#define STARTTH 0.4 // Threshold for start
#define preTH 1     // Press threshold
#define relTH 0.5  // Release threshold
#define maxTH 4     // max threshold  

double THRESHOLD[3][5] = { {preTH, preTH, preTH, preTH, preTH}, {relTH, relTH, relTH, relTH, relTH}, {maxTH, maxTH, maxTH, maxTH, maxTH} };
double fGain[5] = { 1.0,1.0,1.0,1.0,1.0 };  // Increased gains for index and little fingers, SKim

///////////////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the robot and run it
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{

	///__________________________________________Neda - End
	gThisInst = hThisInst;
	// gExp = new MyExperiment("SeqSpatialVis", "ssh_vis", "C:/data/SeqSpatial/ssh_vis");
	gExp = new MyExperiment("SeqSpat", "_", "C:/data/SeqSpatialHorizon/ssh");

	//gExp->redirectIOToConsole();

	tDisp.init(gThisInst, 100, 0, 400, 20, 5, 2, &(::parseCommand));  // the white interactive window
	tDisp.setText("Subj:", 0, 0);

	gScreen.init(gThisInst, 1920, 0, 1920, 1080, &(::updateGraphics)); // the black feedback window
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

	//******************************************************************//
	gTimer.init();
	gExp->control();
	return 0;
}


///////////////////////////////////////////////////////////////
/// MyExperiment Class: contains all the additional information on how that specific
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
		currentTrial->copyHaptics(); // Thread save copy
		if (gTimer[4] > UPDATE_TEXTDISP) {   //currently every 10ms
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
	int x; float dummy1;
	float arg[4];
	int bn;
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

	/// Print continusly state of the encodeers
	else if (arguments[0] == "zeroF") {
		tDisp.keyPressed = 0;
		tDisp.lock();
		double volts[2][5] = { {0,0,0,0,0},{0,0,0,0,0} };
		int n, j;
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

	else if (arguments[0] == "thres") {
		if (numArgs != 5) {
			tDisp.print("USAGE: thresh medianMT for seqLength of 2 3 4 14");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			timeThreshold[0] = (arg[0] * (timeThresPercent / 100));
			timeThresholdSuper[0] = (arg[0] * (superThresPercent / 100));

			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			timeThreshold[1] = (arg[1] * (timeThresPercent / 100));
			timeThresholdSuper[1] = (arg[1] * (superThresPercent / 100));

			sscanf(arguments[3].c_str(), "%f", &arg[2]);
			timeThreshold[2] = (arg[2] * (timeThresPercent / 100));
			timeThresholdSuper[2] = (arg[2] * (superThresPercent / 100));

			sscanf(arguments[4].c_str(), "%f", &arg[3]);
			timeThreshold[3] = (arg[3] * (timeThresPercent / 100));
			timeThresholdSuper[3] = (arg[3] * (superThresPercent / 100));

		}
	}
	// Added by Skim
	else if (arguments[0] == "krun" || arguments[0] == "KRUN") {
		if (arguments[1][0] != arguments[2][8]) { //file name shold be "ssh_sx_by.tgt", x and y seqType, blocknumber
			tDisp.print("Use the same block number shown in tgt file");
		}
		else if (numArgs != 3) {
			tDisp.print("USAGE: run blocknumber targetfile");
		}
		else {
			sscanf(arguments[1].c_str(), "%i", &bn);
			if (gExp->theBlock->init(bn, arguments[2])) {
				gExp->theBlock->state = START_BLOCK;
			}
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
	gNumErrors = 0;
	gNumPointsBlock = 0;
	sprintf(buffer, "%d", gNumPointsBlock);
	gs.line[2] = buffer;

}

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback() {
	b = b++;
	int i, j;

	int n[4] = { 0,0,0,0 };
	double nn[4] = { 0,0,0,0 };
	double MTarray[4][200];
	double medianMT[4] = { 0,0,0,0 };
	MyTrial* tpnr;  //MyTrial object --> inherits class Trial in Experiment.h

	for (i = 0; i < 4; i++) {
		medianMTarray[i][0] = 10000; // Initialise the MT for the 0th block to be 10000 (same as default)

	}
	ERarray[0] = 0; // Initialise the ER for the 0th block to be 0

	for (i = 0; i < trialNum; i++) {
		tpnr = (MyTrial*)trialVec.at(i);
		if (tpnr->seqLength == 2) {
			j = 0;
		}
		else if (tpnr->seqLength == 3) {
			j = 1;
		}
		else if (tpnr->seqLength == 4) {
			j = 2;
		}
		else if (tpnr->seqLength == 14) {  // edited by SKim
			j = 3;

		}

		if (tpnr->isError == 0) {
			MTarray[j][n[j]] = tpnr->MT; //remember the RT from the correct trials and add them
			n[j]++; // remember number of trials
		}
		nn[j]++;
	}
	ERarray[b] = 100 * ((double)gNumErrors) / (double)(trialNum); // error rate

	for (j = 0; j < 4; j++) {
		if (n[j] > 0) { // if more than one correct trials for seqlength j
			medianMTarray[j][b] = median(MTarray[j], n[j]);

			if ((ERarray[b] < ERthreshold) && (ERarray[b - 1] > ERthreshold)) { // if ER on previous block > 15%
				if (medianMTarray[j][b] < (timeThreshold[j] / (timeThresPercent / 100))) { // if MT faster than MT expected by threshold
					timeThreshold[j] = medianMTarray[j][b] * (timeThresPercent / 100);
					timeThresholdSuper[j] = medianMTarray[j][b] * (superThresPercent / 100);
				}
			}
			else if ((ERarray[b] < ERthreshold) && (ERarray[b - 1] < ERthreshold)) { // if ER on previous block <15%
				//if (medianMTarray[b]<medianMTarray[b-1]) { // adjust only if MT of current block faster
				if (medianMTarray[j][b] < (timeThreshold[j] / (timeThresPercent / 100))) { // adjust only if MT of current block faster than MT expected by threshold
					timeThreshold[j] = medianMTarray[j][b] * (timeThresPercent / 100);
					timeThresholdSuper[j] = medianMTarray[j][b] * (superThresPercent / 100);
				}
			}
		}
		else {

			medianMTarray[j][b] = 0;

		}
	}


	// print FEEDBACK on the screen
	sprintf(buffer, "Error rate: %2.0f%%", ERarray[b]);
	gs.line[0] = buffer;
	gs.lineColor[0] = 1;


	//sprintf(buffer,"MTs: %2.0fs , %2.0fs , %2.0fs, %2.0fs , %2.0fs , %2.0fs",medianMTarray[0][b], medianMTarray[1][b], medianMTarray[2][b], medianMTarray[3][b], medianMTarray[4][b], medianMTarray[5][b]);
	sprintf(buffer, "timeThresholdSuper: %2.0fs , %2.0fs , %2.0fs, %2.0fs", timeThresholdSuper[0], timeThresholdSuper[1], timeThresholdSuper[2], timeThresholdSuper[3]);
	gs.line[1] = buffer;
	gs.lineColor[1] = 1;

	gNumPoints += gNumPointsBlock;
	sprintf(buffer, "Point you've got: %d   Total points: %d", gNumPointsBlock, gNumPoints);
	gs.line[2] = buffer;
	gs.lineColor[2] = 1;
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
	hand = 2; // Read right box
	isError = 0; // init error flag
	isComplete = 0; // init if seq was produced incomplete but correct so fare
	seqCounter = 0; // init the sequence index variable
	numNewpress = 0;
	DigPressed = 0;
	MT = 0; // init total movement time, SKim edited
	RT = 0; // Added by SKim, reaction time
	points = 0;
	int released = 0;
	for (int i = 0; i < MAX_PRESS; i++) {    // MAX_PRESS = 14 defined in header
		response[i] = 0; // respose, pressTime and releaseTime
		pressTime[i] = 0; // are arrays of length 14
		releaseTime[i] = 0;
		fGiven[i] = 0;
	}


}

///////////////////////////////////////////////////////////////
// Read   // This is where the Target files are read
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	(in) >> seqType >> feedback;
	for (int i = 0; i < MAX_PRESS; i++) {   // MAX_PRESS = 14--> read presses
		(in) >> press[i];
	}
	// (in) >> hand >> cueS >> cueC >> cueP >> iti >> sounds >> Horizon >> StimTimeLim;
	(in) >> cueP >> iti >> Horizon >> StimTimeLim >> PrepTime;

	// do other job
	string zero("0");
	gTrial++;
	cTrial = gTrial;
	complete = 0;
	seqLength = cueP.find(zero); // get seqLength
	// chunkLength = cueC.length(); // get chunkLength
	if (seqLength < 0) { seqLength = cueP.length(); }
}

///////////////////////////////////////////////////////////////
// Write  // Neda - Eye data to be added
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	out << seqType << "\t"
		<< Horizon << "\t"
		<< feedback << "\t"
		<< PrepTime << "\t"
		;
	for (int i = 0; i < MAX_PRESS; i++) {
		out << press[i] << "\t";
	}
	out << cueP << "\t"
		<< complete << "\t"
		<< iti << "\t"
		// << sounds << "\t"
		<< MT << "\t"
		<< RT << "\t"  // added by SKim
		<< isError << "\t";
	for (int i = 0; i < MAX_PRESS; i++) {
		out << response[i] << "\t";
	}
	for (int i = 0; i < MAX_PRESS; i++) {
		out << pressTime[i] << "\t";
	}

	out << timeThreshold[0] << "\t"
		<< timeThreshold[1] << "\t"
		<< timeThreshold[2] << "\t"
		<< timeThreshold[3] << "\t"
		<< points << "\t"
		<< fGain[0] << "\t"
		<< fGain[1] << "\t"
		<< fGain[2] << "\t"
		<< fGain[3] << "\t"
		<< fGain[4] << "\t"
		<< StimTimeLim << "\t" << endl;

}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) {
	char header[200];
	out << "seqType" << "\t"
		<< "Horizon" << "\t"
		<< "FT" << "\t"
		<< "PrepTime" << "\t";
	for (int i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "press%d", i);
		out << header << "\t";
	}
	out << "cueP" << "\t"
		<< "complete" << "\t"
		<< "iti" << "\t"
		// << "sounds" << "\t"
		<< "MT" << "\t"
		<< "RT" << "\t"   // added by SKim
		<< "isError" << "\t";
	for (int i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "response%d", i);
		out << header << "\t";
	}
	for (int i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "pressTime%d", i);
		out << header << "\t";
	}
	out << "timeThreshold2" << "\t"
		<< "timeThreshold3" << "\t"
		<< "timeThreshold4" << "\t"
		<< "timeThreshold14" << "\t"
		<< "points" << "\t"
		<< "Gain1" << "\t"
		<< "Gain2" << "\t"
		<< "Gain3" << "\t"
		<< "Gain4" << "\t"
		<< "Gain5" << "\t"
		<< "StimTimeLim" << "\t" << endl;
	//_____________________end

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

	sprintf(buffer, "State : %d   Trial: %d", state, gExp->theBlock->trialNum);
	tDisp.setText(buffer, 2, 0);

	sprintf(buffer, "Press:  %d %d %d %d %d", finger[0], finger[1], finger[2], finger[3], finger[4]);
	tDisp.setText(buffer, 3, 0);

	sprintf(buffer, "sequence Counter: %d ", seqCounter);
	tDisp.setText(buffer, 4, 0);

	sprintf(buffer, "numNewpress: %d ", numNewpress);
	tDisp.setText(buffer, 5, 0);

	sprintf(buffer, "released: %d", released);
	tDisp.setText(buffer, 6, 0);

	sprintf(buffer, "gNumPointsBlock: %d", gNumPointsBlock);
	tDisp.setText(buffer, 7, 0);

	sprintf(buffer, "Horizon: %d", Horizon);
	tDisp.setText(buffer, 8, 0);

	//sprintf(buffer,"thresholds : %2.0f %2.0f %2.0f Super: %2.0f %2.0f %2.0f",timeThreshold[0],timeThreshold[1],timeThreshold[2],timeThresholdSuper[0],timeThresholdSuper[1],timeThresholdSuper[2]);
	//tDisp.setText(buffer,5,0);

	sprintf(buffer, "trial : %d seqType : %d state : %d", cTrial, seqType, state);
	tDisp.setText(buffer, 9, 0);


}

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD
///////////////////////////////////////////////////////////////
#define FINGWIDTH 2
#define RECWIDTH 1.4
#define FORCESCALE 2
#define BASELINE -10  // SKim

void MyTrial::updateGraphics(int what) {
	int i;
	double height;
	// Finger forces
	if (gs.showLines == 1) {
		gScreen.setColor(Screen::white); // defines the color of force lines
		for (i = 0; i < 5; i++) {
			//reads the forces and determins how high the small bars should jump up
			height = gBox[hand - 1].getForce(i) * FORCESCALE * fGain[i] + BASELINE;
			//draws the smaller line for individual finger forces
			gScreen.drawLine(i * FINGWIDTH - 5.0, height, i * FINGWIDTH - 3.0, height);
		}
		gScreen.drawLine(-5, BASELINE, 5, BASELINE); // the lower line
		gScreen.drawLine(-5, preTH * FORCESCALE + BASELINE, 5, preTH * FORCESCALE + BASELINE);
		gScreen.drawLine(-5, relTH * FORCESCALE + BASELINE, 5, relTH * FORCESCALE + BASELINE); // changed by SKim

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
	//if (state == WAIT_ALLRELEASE) {
	//	gScreen.setColor(2);
	//	gScreen.printChar('+', 0, -6, 2*SIZE_CUE);
	//	
	//}
	if (state == WAIT_ALLRELEASE || state == WAIT_GOCUE || state == WAIT_PRESS) {
		if (state == WAIT_ALLRELEASE || state == WAIT_GOCUE) {
			gScreen.setColor(2);  // Red signal, wait for "GO" signal and all fingers are released
			gScreen.printChar('+', 0, -7, SIZE_CUE);
		}
		else {
			if (gTimer[2] < 1000) {
				gScreen.setColor(3); // Green signal
			}
			else {
				gScreen.setColor(0); // "GO" signal invisible
			}
			gScreen.printChar('+', 0, -7, SIZE_CUE);
		}

		if (state == WAIT_GOCUE || state == WAIT_PRESS) {
			// Draw horizon SKim
			gScreen.setColor(1);

			if (seqType == 1) {  // Visual, vertical
				gHorizon.position = Vector2D(0, 3.5 + Horizon);
				gHorizon.size = Vector2D(10, 16 - 2 * Horizon);
				gHorizon.draw();
				gScreen.drawLine(-5, -5, -5, 11);
				gScreen.drawLine(-3, -5, -3, 11);
				gScreen.drawLine(-1, -5, -1, 11);
				gScreen.drawLine(1, -5, 1, 11);
				gScreen.drawLine(3, -5, 3, 11);
				gScreen.drawLine(5, -5, 5, 11);

				for (i = 0; i < min(Horizon, seqLength - seqCounter); i++) {
					if (gs.cuePress[i] > 0) {
						double xPos = gs.cuePress[i + seqCounter] - '1';
						gTarget.position = Vector2D(-4.0 + 2.0 * xPos, -4.0 + i * 2);
						gTarget.size = Vector2D(2.0, 2.0);
						gTarget.draw();
					}
				}
			}
			else if (seqType == 3) { // Visual horizontal
				gHorizon.position = Vector2D(Horizon, 3);
				gHorizon.size = Vector2D(16 - 2 * Horizon, 10);
				gHorizon.draw();
				gScreen.drawLine(-8, -2, 8, -2);
				gScreen.drawLine(-8, 0, 8, 0);
				gScreen.drawLine(-8, 2, 8, 2);
				gScreen.drawLine(-8, 4, 8, 4);
				gScreen.drawLine(-8, 6, 8, 6);
				gScreen.drawLine(-8, 8, 8, 8);

				for (i = 0; i < min(Horizon, seqLength - seqCounter); i++) {
					if (gs.cuePress[i] > 0) {
						double yPos = gs.cuePress[i + seqCounter] - '1';
						gTarget.position = Vector2D(-7.0 + i * 2, -1.0 + 2.0 * yPos);
						gTarget.size = Vector2D(2.0, 2.0);
						gTarget.draw();
					}
				}
			}

			else if (seqType == 2) { // Vertical, Numbers
				gHorizon.position = Vector2D(0, 3.5 + Horizon);
				gHorizon.size = Vector2D(10, 16 - 2 * Horizon);
				gHorizon.draw();
				for (i = 0; i < min(Horizon, seqLength - seqCounter); i++) {  // Edited by SKim
					if (gs.cuePress[i] > 0) {
						//						gScreen.printChar(gs.cuePress[i], (i - 4) * WIDTH_CHAR_CUE, CUE_PRESS, SIZE_CUE);
						gScreen.printChar(gs.cuePress[i + seqCounter], 0, -4.0 + i * 2, SIZE_CUE);
						// the number 6.5 is usually the seqLength/2 so that the sequence in centered
					}
				}
			}

			else {  // seqType==4, Horizontal, Numbers
				gHorizon.position = Vector2D(Horizon, 3);
				gHorizon.size = Vector2D(16 - 2 * Horizon, 10);
				gHorizon.draw();
				for (i = 0; i < min(Horizon, seqLength - seqCounter); i++) {  // Edited by SKim
					if (gs.cuePress[i] > 0) {
						//						gScreen.printChar(gs.cuePress[i], (i - 4) * WIDTH_CHAR_CUE, CUE_PRESS, SIZE_CUE);
						gScreen.printChar(gs.cuePress[i + seqCounter], -7.0 + i * 2, 3.0, SIZE_CUE);
						// the number 6.5 is usually the seqLength/2 so that the sequence in centered
					}
				}


			}


		}
	}
}



	//else {
	//	for (i = 0; i < seqLength; i++) {  // Edited by SKim
	//		if (gs.cuePress[i] > 0) {
	//			gScreen.setColor(1);
	//			gScreen.printChar(gs.cuePress[i], (i - 4) * WIDTH_CHAR_CUE, CUE_PRESS, SIZE_CUE);
	//			// the number 6.5 is usually the seqLength/2 so that the sequence in centered
	//		}
	//	}
	//}




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
	double critTime;
	int numNewpress = 0; // is there a new press?
	int pressedFinger = 0;
	released = 0;

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


	////__________________________________Neda
	//if (calib_mode) {
	// state = CALIB;
	//}
	////__________________________________end

	switch (state) {  // this state is before you enter the "run X *.tgt" command
	case WAIT_TRIAL: //0 as apears in mov
		gs.clearCues();

		break;
	case START_TRIAL:   // 1  This state is right after you've entered
		//the "run X *.tgt" command, and at the begginign
		//of each trail i that block. basically sets up
		//recording and clears screen for new trial
		gTimer.reset(1); // time for whole trial
		gTimer.reset(2); // time for events in the trial


		for (i = 0; i < NUMDISPLAYLINES; i++) {
			gs.line[i] = "";
		} // clear screen
		//__________________________________Neda
		// here I add the WAIT_FOR_EYE state which is supposed to wait
		//for the eye to fixate at the begginig of the cue

		state = START_FIX;
		break;
	case START_FIX: //2 as appears in mov

		if (released == 5) {
			dataman.startRecording();
			gTimer.reset(1); // time for whole trial
			gTimer.reset(2); // time for events in the trial
			int T1 = time(0);
			int w = 0;
			state = WAIT_ALLRELEASE;
		}
		break;

	case WAIT_ALLRELEASE: //3 as appears in mov

		if (gTimer[1] > 20000) {
			gs.clearCues();
			state = WAIT_FEEDBACK;
		}
		if (released == 5 && gTimer[2] > 2000) { //gTimer[2] > 1500 makes sure that the cross is being shown for 3 secs
			gTimer.reset(2);
			gs.clearCues();
			for (i = 0; i < seqLength; i++) {
				gs.cuePress[i] = cueP.at(i);
			}
			state = WAIT_GOCUE;
		}
		break;
	case WAIT_GOCUE:
		// check for time out
		if (gTimer[1] > 20000) {
			gs.clearCues();
			state = WAIT_FEEDBACK;
		}
		if (released == 5 && gTimer[2] > PrepTime) { // Wait for PrepTime, preplanning
			gTimer.reset(2);
//			gs.clearCues();
			state = WAIT_PRESS;
		}
		break;

	case WAIT_PRESS: //5 as appears in mov, Targets are shown here for preplanning
		// check for time out
		if (gTimer[1] > 20000) {
			gs.clearCues();
			state = WAIT_FEEDBACK;
		}

		if (StimTimeLim) {
			if (gTimer[2] > StimTimeLim) {
				for (i = 0; i < NUMDISPLAYLINES; i++) {
					gs.clearCues();
				}
			}
		}


		// Wait for the next keypress
		//*************************Feedback loop was here
		// Check if sequence is finished
		
		if (numNewpress > 0 && seqCounter < seqLength) {
			response[seqCounter] = pressedFinger;
			pressTime[seqCounter] = gTimer[1];
			if (seqCounter == 0) {
				RT = gTimer[2];  // Reaction time for the first press, SKedited
			}
			if (response[seqCounter] == press[seqCounter]) { // if press is correct
				// PLAY SOUND
//				channel = Mix_PlayChannel(-1, wavTask[0], 0); // SDL
				PlaySound("wav/chimes.wav", NULL, SND_ASYNC | SND_FILENAME);
			}
			else if (response[seqCounter] != press[seqCounter]) {   // press is wrong
				isError = 1;
				// PLAY SOUND
				PlaySound("wav/chord.wav", NULL, SND_ASYNC | SND_FILENAME);
//				channel = Mix_PlayChannel(-1, wavTask[1], 0); // SDL
			}

			seqCounter++;
			if (DigPressed < seqLength - Horizon + 1) {
				DigPressed++;
			}
		}

		if (seqCounter == seqLength && released == 5) {
			state = WAIT_END_RELEASE;
		}


		/*if (Horizon < seqLength -1){
		if (DigPressed < seqLength - Horizon){

		for (i= DigPressed; i<DigPressed + Horizon + 1; i++) {
		gs.cuePress[i] = cueP.at(i);
		}

		DigPressed += 1;
		} else if ((DigPressed == seqLength - Horizon)){
		for (i= DigPressed; i<seqLength; i++) {
		gs.cuePress[i] = cueP.at(i);
		}
		}
		}*/
		break;
	case WAIT_END_RELEASE: //6 as appears in mov

		if (released == 5) {

			MT = gTimer[1] - pressTime[0]; // Calculate total reaction time starting from the first press

			if (isError > 0) {
				gNumErrors++;
			}
			else {
				switch (seqLength) {
				case 2:
					j = 0;
					break;
				case 3:
					j = 1;
					break;
				case 4:
					j = 2;
					break;
				case 14:
					j = 3;
					break;
				default:
					j = 3;
					break;
				}

				critTime = MT;

				if (critTime < timeThresholdSuper[j]) {
					points = 3;
					gNumPointsBlock += 3;
					// PLAY SOUNDS
	//				channel = Mix_PlayChannel(-1, wavTask[2], 0); // SDL
				}
				else if (critTime < timeThreshold[j]) {
					points = 2;  // SKim
					gNumPointsBlock += 2; // SKim
					// PLAY SOUND
					//channel = Mix_PlayChannel(-1, wavTask[3], 0); // SDL
				}
				else {
					points = 1;
					gNumPointsBlock += 1;
					// PLAY SOUND
					// channel = Mix_PlayChannel(-1, wavTask[1], 0); // SDL
				}

			}
			gTimer.reset(2);
			gs.clearCues();
			// commented by SKim
			//if (seqLength == 2) {
			// gs.cuePress[7] = '+';
			//}
			//else if (seqLength == 3) {
			// gs.cuePress[8] = '+';
			//}
			//else if (seqLength == 4) {
			// gs.cuePress[8] = '+';
			//}
			//else {
			// gs.cuePress[13] = '+';
			//}


			gTimer.reset(2);
			state = WAIT_FEEDBACK;
		}
		else {
			state = WAIT_PRESS;
		}

		break;

	case WAIT_FEEDBACK:  //7 as appears in mov

		//do iti
		if (gTimer[2] > 1000) { // after 1 sec, providing feedback 
			dataman.stopRecording();
			int w = seqLength - 1;
			gs.clearCues();
			sprintf(buffer, "+%d", points);
			gs.lineColor[2] = 1;
			gs.line[2] = buffer; // displays the text
			state = END_FIX;
		}
		break;
	case END_FIX: //8
		if (gTimer[2] > FEEDBACKTIME) {
			// reset clear the screen

			for (i = 0; i < 2; i++) { gs.boxColor[i] = 0; }
			gTimer.reset(2);
			gs.clearCues();
			state = WAIT_ITI;

		}
		break;
	case WAIT_ITI:  //9 as appears in mov
		if (gTimer[2] > 2000) { // ITI fixed as 2 sec
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
	state = s;                              //culumn 1 of the .mov file
	time = gTimer[1];  //culumn 2 of the .mov file
	timeReal = gTimer.getRealtime();        //culumn 3 of the .mov file

	for (i = 0; i < 5; i++) {
		force_left[i] = gBox[0].getForce(i);//culumn 4-8 of the .mov file
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
	for (i = 0; i < 14; i++) {//(i=0;i<3;i++){
		lineXpos[i + 8] = 0;//i*1.4-1.4;
		lineYpos[i + 8] = CUE_PRESS;//2.3;
		lineColor[i + 8] = 1; // white
		size[i + 8] = 7;  // font size
	}

	clearCues();


	boxOn = false;
	showLines = true;
}

void GraphicState::clearCues(void) {
	int i;

	for (i = 0; i < 14; i++) {
		cuePress[i] = 0;
	}
}

void GraphicState::reset(void) {
	for (int i = 0; i < NUMDISPLAYLINES; i++) {
		line[i] = "";
	}
}






