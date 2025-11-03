///////////////////////////////////////////////////////////////
/// 
///	
///	learning of finger sequences
/// 
/// 
/// Tobias Wiestler, 2010 
/// 
/// Hardly changed at all by George Prichard, 2012. Thanks Tobi!
///////////////////////////////////////////////////////////////

#include "MDI0.h" 
#include "StimulatorBox.h" 
///////////////////////////////////////////////////////////////
/// Global variables 
///////////////////////////////////////////////////////////////
S626sManager s626;				///< Hardware Manager 
TextDisplay tDisp;				///< Text Display
Screen gScreen;					///< Screen 
TRCounter gCounter;				///< TR Counter 
StimulatorBox gBox;			///< Stimulator Box

//StimulatorBox gBox[2];		///< Stimulator Box 
Timer gTimer(UPDATERATE);		///< Timer from S626 board experiments 
HapticState hs;					///< This is the haptic State as d by the interrupt 
///< For Thread safety this SHOULD NOT be assessed While 
///< the interrupt is running. Use Thread-safe copy to 
///< Get the current haptic state for graphical display 
GraphicState gs;

char buffer[300];					///< String buffer 
HINSTANCE gThisInst;					///< Instance of Windows application 
Experiment* gExp;					///< Pointer to myExperiment 
Trial* currentTrial;					///< Pointer to current Trial 
bool gKeyPressed;					///< Key pressed? 
char gKey;						///< Which key?
int isPractice = 0;					///< Should we show the lines to help practice
int gNumErrors = 0;					///< How many erros did you make during a block

double timeThresholds[2] = { 0.8, 1.2 };	///< percentage when super fast and when late trial

int digitCounter = 0;

int gNumPointsBlock = 0;
int gNumPoints = 0;
int ghardPress = 0;
int glatePress = 0;
double avrgMT[2] = { 0, 0 };
double stMT = 5000;
//double seqMT[2][17] = { {stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT, stMT},
//						{stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT, stMT} };
//double seqGood[2][17] = { {0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0},
//						{0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0} };
//double seqForce[2][17] = { {0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0},
//						{0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0} };
//double seqFingerForce[][5] = { {0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},
//							{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0} };

#define TRTIME 2660//2700
char counterSignal = '5';		///< ToDo: AVOID THAT What char is used to count the TR
int sliceNumber = 32;			///< How many slices do we have

#define FEEDBACKTIME 800 
string TEXT[11] = { "*","1","2","3","4","5","6", "7", "8", "9", "+" };
//string FINGERSOUND[6] = { "A.wav", "C.wav", "D.wav", "E.wav", "G.wav" };
//int STIM_INTENSITY[5] = { 5, 5, 5 ,5, 5 };
//#define resTH 3     // in NEWTONS 0.6 * 4.9276 //0.5 * 4.9276
//#define relTH 2.5   // 0.45 * 4.9276 
//#define maxTH 20    //  5.0 * 4.9276 //5* 4.9276 //1.8 * 4.9276  (1.8 is too low--SWM) 2.8
double thresh = 3;//THRESHOLD[5] = { resTH, resTH, resTH, resTH, resTH }; //, {relTH, relTH, relTH, relTH, relTH}, {maxTH, maxTH, maxTH, maxTH, maxTH} };

///////////////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the task and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	gThisInst = hThisInst;
	gExp = new MyExperiment("ModifiedDigitInterference", "MDI0", "C:/data/ModifiedDigitInterference/MDI0/");
	gExp->redirectIOToConsole();

	tDisp.init(gThisInst, 0, 0, 550, 26, 8, 2, &(::parseCommand));
	tDisp.setText("Subj:", 0, 0);

	gScreen.init(gThisInst, 1920, 0, 1680, 1024, &(::updateGraphics));
	//gScreen.setScale(Vector2D(0.02,0.02));
	//gScreen.setScale(Vector2D(1.1*0.02,1.1*0.02));	 // In cm 
	//gScreen.setCenter(Vector2D(0,2));	 // In cm 
	gScreen.setCenter(Vector2D(2, 3));	 // In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE)); // cm/pixel 

	// initalize s626cards 
	s626.init("c:/robotcode/calib/s626_single.txt");
	if (s626.getErrorState() == 0) {
		cout << "Hello" << endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); //1	5			// initialize at 200 Hz update rate 
	}

	gTimer.init(1, 4, 0);					/// < On Cntr_1A , Cntr_1B 
	// initialize stimulation box
	//gBox[0].init(BOX_LEFT, "c:/robotcode/calib/Flatbox1_lowforce_LEFT_02-Dec-2021.txt");
	gBox.init(BOX_RIGHT, "c:/robotcode/calib/Flatbox1_lowforce_LEFT_02-Dec-2021.txt");
	gBox.filterconst = 0.8;
	//gBox[1].filterconst = 0.8;

	// initalize serial counter 
	gCounter.initSerial("COM1", 9600, counterSignal, sliceNumber); //serial 9600 19200
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
	do {
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		theBlock->control();
		currentTrial->copyHaptics();		// Thread save copy 
		if (gTimer[3] > UPDATE_TEXTDISP) {
			currentTrial->updateTextDisplay();
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			gTimer.reset(3);
		};
		InvalidateRect(gScreen.windowHnd, NULL, TRUE);
		UpdateWindow(gScreen.windowHnd);
	} while (msg.message != WM_QUIT);
}


///////////////////////////////////////////////////////////////
// Parse additional commands 
///////////////////////////////////////////////////////////////
bool MyExperiment::parseCommand(string arguments[], int numArgs) {
	int i, b;
	float arg[4];
	MSG msg;

	/// Recenter 


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

			sprintf(buffer, "Force : %2.2f %2.2f %2.2f %2.2f %2.2f", gBox.getForce(0),
				gBox.getForce(1), gBox.getForce(2), gBox.getForce(3), gBox.getForce(4));
			tDisp.setText(buffer, 5, 0);
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10);
		}
		tDisp.unlock();
	}

	/// Print continusly state of the encodeers 
	else if (arguments[0] == "zeroF") {
		tDisp.keyPressed = 0;
		tDisp.lock();
		double volts[5] = {0,0,0,0,0};
		int n, j;
		for (n = 0; n < 100; n++) {
			for (j = 0; j < 5; j++) {
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

	/// Set TR Counter to simulated or non-simulated 
	else if (arguments[0] == "TR") {
		if (numArgs != 2) {
			tDisp.print("USAGE: TR delay [ms]");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			if (arg[0] >= 0) {
				gCounter.simulate(arg[0]);
			}
			else {
				gCounter.simulate(0);
			}
		}
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
			//gScreen.setScale(Vector2D(0.02*arg[0],0.02*arg[1]));			
			gScreen.setCenter(Vector2D(2 * arg[0], 3 * arg[1]));
			//gScreen.setCenter(Vector2D(0.02*arg[0],0.02*arg[1]));			
		}
	}

	/// Show the force lines 
	else if (arguments[0] == "showlines") {
		if (numArgs != 2) {
			tDisp.print("USAGE: showlines 0/1");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			isPractice = arg[0];
		}
	}
	/// set the movement time threshold
	else if (arguments[0] == "tth") {
		if (numArgs != 2) {
			tDisp.print("USAGE: need to super fast and late percent values (tth 0.8 1.2) ");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			timeThresholds[1] = arg[0];
			timeThresholds[2] = arg[1];
		}
	}
	else if (arguments[0] == "stmt") {
		if (numArgs != 2) {
			tDisp.print("USAGE: stmt 0/.../10000 in ms");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			stMT = arg[0];
			//for (i = 0; i < 17; i++) { //loop over all possible sequences and 
			//	seqMT[0][i] = stMT;
			//	seqMT[1][i] = stMT;
			//}
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
	//gs.boxOn = true;
	gCounter.reset();
	gCounter.start();
	gNumErrors = 0;
	gNumPointsBlock = 0;
	ghardPress = 0;
	glatePress = 0;
}

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state 
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback() {
	//gs.boxOn = false;
	int i, j;
	double n[2] = { 0, 0 };
	double nn[2] = { 0, 0 };


	MyTrial* tpnr;
	avrgMT[0] = 0;
	avrgMT[1] = 0;
	//reset the sequence time 
	//for (i = 1; i < 17; i++) {
	//	for (j = 0; j < 2; j++) {
	//		seqMT[j][i] = 0;
	//		seqGood[j][i] = 0;
	//		seqForce[j][i] = 0;
	//	}
	//}

	//for (i = 0; i < trialNum; i++) {
	//	tpnr = (MyTrial*)trialVec.at(i);
	//	if (tpnr->announce == 0 & tpnr->lastTrial == 0) {
	//		if (tpnr->errorFlag == 0 & tpnr->incomplete == 0) {
	//			avrgMT[tpnr->hand] += tpnr->MT; //remember the RT from the trials and add them
	//			n[tpnr->hand]++; // remember number of trials
	//			//sequence movement time
	//			//cout<<tpnr ->startTR<<tpnr->MT<<endl;

	//			seqMT[tpnr->hand][tpnr->seqType] += tpnr->MT; //get the movement times for the sequence
	//			seqGood[tpnr->hand][tpnr->seqType]++; //remeber how often the seq was produced correct
	//			seqForce[tpnr->hand][tpnr->seqType] += tpnr->Force;
	//		}
	//		nn[tpnr->hand]++; //count task trials
	//	}
	//}
	//if (n[0] > 0)
	//	avrgMT[0] /= (n[0]);

	//if (n[1] > 0)
	//	avrgMT[1] /= (n[1]);

	//for (i = 0; i < 17; i++) { //loop over all possible sequences and 
	//	for (j = 0; j < 2; j++) {
	//		if (seqGood[j][i] > 0) {
	//			seqMT[j][i] /= (seqGood[j][i]);
	//			seqForce[j][i] /= (seqGood[j][i]);
	//		}
	//		else {
	//			seqMT[j][i] = stMT;
	//			seqForce[j][i] = 0;
	//		}
	//	}
	//}


	// print FEEDBACK on the screen 
	for (i = 0; i < 11; i++) {
		gs.line[i] = "";
	}		// reset clear the screen
	//cout<< "gNumErrors: " <<gNumErrors<<" count task trials: "<<nn<<endl;
	sprintf(buffer, "error rate: %2.0fpercent", (100.0 / (nn[0] + nn[1]) * gNumErrors));
	gs.line[11] = buffer;
	sprintf(buffer, "Average movement time: L %2.2fs  R %2.2fs", avrgMT[0] / 1000, avrgMT[1] / 1000);
	gs.line[12] = buffer;
	gCounter.stop();

	gNumPoints += gNumPointsBlock;
	sprintf(buffer, "Number points: %d   Total: %d", gNumPointsBlock, gNumPoints);
	gs.line[13] = buffer;

	//ToDo more feedback force MT exclude errors
}

///////////////////////////////////////////////////////////////
///	My Trial class contains the main info of how a trial in this experiment is run 
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Constructor 
///////////////////////////////////////////////////////////////
MyTrial::MyTrial() {
	//state = WAIT_TRIAL;
	//INIT TRIAL VARIABLE

	//errorFlag = 0;						// init error flag
	//lateFlag = 0;
	//hardPress = 0;						// init hard press flag
	//incomplete = 0;						// init if seq was produced incomplete but correct so fare
	//seqCounter = 0;						// init the sequence index variable
	//inactiveFinger = 0;					// init the inactive Finger counter
	//MT = 0;								// init total reaction time
	//Force = 0;
	//pointState = 0;
	//superFast = 0;						// init super fast flag
	//allPressed = 0;						// are all fingers on the board?
	//for (int i = 0; i < 5; i++) {
	//	releaseState[i] = 1;				// init the release state of the fingerpress 
	//	RT[i] = 0;						//		& reaction time	
	//	response[i] = 0;					//		& finger response
	//	pressT[i] = 0;					//		& duration of finger press
	//	hardpressKnown[i] = 0;			//		& hardpress knowlege	
	//
}

///////////////////////////////////////////////////////////////
// Read
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	(in) >> startTR
		>> startTime
		>> planTime
		>> execTime
        >> feedbackTime
		>> iti
	>> sequence;
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	out << startTR << "\t"
		<< startTRReal << "\t"
		<< startTimeReal << "\t"
		<< planTime << "\t"
		<< execTime 
		<< endl;
		/*<< lastTrial << "\t"
		<< startTime << "\t"
		<< seqType << "\t"
		<< announce << "\t"
		<< feedback << "\t"
		<< complete << "\t"
		<< iti << "\t"
		<< response[0] << "\t"
		<< RT[0] << "\t"
		<< pressT[0] << "\t"
		<< response[1] << "\t"
		<< RT[1] << "\t"
		<< pressT[1] << "\t"
		<< response[2] << "\t"
		<< RT[2] << "\t"
		<< pressT[2] << "\t"
		<< response[3] << "\t"
		<< RT[3] << "\t"
		<< pressT[3] << "\t"
		<< response[4] << "\t"
		<< RT[4] << "\t"
		<< pressT[4] << "\t"
		<< MT << "\t"
		<< errorFlag << "\t"
		<< hardPress << "\t"
		<< lateFlag << "\t"
		<< incomplete << "\t"
		<< pointState << "\t"
		<< hand << endl;*/
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) {
	out << "startTR" << "\t"
		<< "lastTrial" << "\t"
		<< "startTime" << "\t"
		<< "seqType" << "\t"
		<< "announce" << "\t"
		<< "feedback" << "\t"
		<< "complete" << "\t"
		<< "iti" << "\t"
		<< "resp1" << "\t"
		<< "RT1" << "\t"
		<< "pressT1" << "\t"
		<< "resp2" << "\t"
		<< "RT2" << "\t"
		<< "pressT2" << "\t"
		<< "resp3" << "\t"
		<< "RT3" << "\t"
		<< "pressT3" << "\t"
		<< "resp4" << "\t"
		<< "RT4" << "\t"
		<< "pressT4" << "\t"
		<< "resp5" << "\t"
		<< "RT5" << "\t"
		<< "pressT5" << "\t"
		<< "MT" << "\t"
		<< "seqError" << "\t"
		<< "hardPress" << "\t"
		<< "latePress" << "\t"
		<< "incompletePress" << "\t"
		<< "trialPoints" << "\t"
		<< "hand" << endl;

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
	sprintf(buffer, "TR : %d time: %2.2f slice:%d", gCounter.readTR(), gCounter.readTime(), gCounter.readSlice());
	tDisp.setText(buffer, 2, 0);

	//sprintf(buffer,"TIME : %1.4f:", gScreen.lastCycle);
	//tDisp.setText(buffer,4,0);
	sprintf(buffer, "State : %d  State time: %2.1f", state, gTimer[1]);
	tDisp.setText(buffer, 4, 0);

	sprintf(buffer, "read : %2.1f   readReal : %2.1f", gTimer[0], gTimer.readReal(1));
	tDisp.setText(buffer, 5, 0);

	//sprintf(buffer,"volts: %d %d %d %d %d",releaseState[0],releaseState[1],releaseState[2],releaseState[3],releaseState[4]); 
	//tDisp.setText(buffer,5,0);

	sprintf(buffer, "Force RIGHT:  %2.2f %2.2f %2.2f %2.2f %2.2f", gBox.getForce(0), gBox.getForce(1), gBox.getForce(2), gBox.getForce(3), gBox.getForce(4));
	tDisp.setText(buffer, 8, 0);

}

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD 
///////////////////////////////////////////////////////////////
#define FINGWIDTH 1
#define RECWIDTH 1.4
void MyTrial::updateGraphics(int what) {
	int i, j;

	if (gs.showSequence) {
		gScreen.setColor(Screen::white);
		gScreen.print(sequence, 0, 0, 5);
	}

	Vector2D recSize, recPos;
	recSize = Vector2D(1.3, 1.3);


	if (gs.showDiagnostics) {
		string stateString;
		switch (state)
		{
		case WAIT_TR:
			stateString = "Wait TR";
			break;
		case START_TRIAL:
			stateString = "Start Trial";
			break;
		case WAIT_PLAN:
			stateString = "Wait Plan";
			break;
		case WAIT_RESPONSE:
			stateString = "Wait Resp";
			break;
		case WAIT_FEEDBACK:
			stateString = "Wait Feedback";
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
	/// Update clocks and manipulandum 
	gTimer.countup();
	gTimer.countupReal();
	s626.updateAD(0);

	gCounter.update();
	/////if we need to check for the TR counter signal:
	//{ 
	//		buffer[0]=gCounter.readTR()%9+90;
	//		buffer[1]='0';
	//		gSerial.Write(buffer,1); 
	//	};
	gBox.update();
	//gBox[1].update(); 
	/// Call the Trial for control 
	currentTrial->control();

	/// record the data at record frequency 
	if (dataman.isRecording() && gTimer[4] >= RECORDRATE) {
		gTimer.reset(4);
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
	int i;
	Vector2D recSize;
	Vector2D recPos;
	int goalResponse;

	switch (state) {
	case START_TRIAL: //0
		gTimer.reset(1);
		gTimer.reset(2);
		gs.showDiagnostics = 1;
		dataman.clear();
		dataman.startRecording();
		for (i = 0; i < 11; i++) { gs.line[i] = ""; }			// clear screen
		gs.lineColor[7] = 1;					// WHITE
		state = WAIT_TR;
		break;

	case WAIT_TR: //1		
		/// Wait for TR counter to be at the right place & reset the clocks
		if (gCounter.readTR() == 0) {
			gTimer.reset(0);
		}
		if (gCounter.readTR() > 0 && gCounter.readTotTime() >= startTime) {
			startTimeReal = gCounter.readTotTime();
			startTRReal = gCounter.readTR(); // number of TR arrived so far

			
			gTimer.reset(1);					//time for whole trial
			gTimer.reset(2);					//time for events in the trial
			gBox.boardOn = 1;
			state = WAIT_PLAN;
		}

		break;

	case WAIT_PLAN: //2
		digitCounter = -1;
		// put here what happens during WAIT_PLAN
		//gs.line[0] = sequence;
		if (gTimer[1] > planTime) {
			state = WAIT_RESPONSE;
			gTimer.reset(1);					//time for whole trial
			gTimer.reset(2);					//time for events in the trial
		}
		break;

	case WAIT_RESPONSE: //3
		gs.showSequence = 1;
		for (i = 0; i < 5; i++) { // check all finger 
			if (gBox.getForceFilt(i) > thresh && releaseState[i]) { // check for initial press
				digitCounter = digitCounter + 1;
				releaseState[i] = 0; // set the finger to unreleased
				//response[seqCounter] = i + 1; // record finger that was pressed 
				//RT[seqCounter] = gTimer[2]; // record the reaction time 

				//if (sequence[digitCounter] == i) {
				//	break;// correct digit
				//}
				//else {
				//	break;// wrong digit
				//}
				//< Check if the single pressed finger (the only one with releaseState==0) is released 
			}
			else if (gBox.getForceFilt(i) <= thresh && !releaseState[i]) { // check for release of the press
				releaseState[i] = 1;
			}
		}

	if (gTimer[2] > execTime) { // Too late! only executed if complete is set => FOR SCANNING SESSION
		
		state = WAIT_FEEDBACK;

		gTimer.reset(2);
	} 
	break;

	case WAIT_FEEDBACK:  //4
		//do iti 
		if (gTimer[2] > FEEDBACKTIME) {
			gTimer.reset(2);
			state = WAIT_ITI;
		}
		break;

	case WAIT_ITI:  //5
		if (gTimer[2] > iti) {
			dataman.stopRecording();
			state = END_TRIAL;
		}

		break;

	case END_TRIAL: //6
		break;
	}
}


/////////////////////////////////////////////////////////////////////////////////////
/// Data Record: creator records the current data from the device 
/////////////////////////////////////////////////////////////////////////////////////
DataRecord::DataRecord(int s) {
	int i;
	state = s;
	TR = gCounter.readTR();
	currentSlice = gCounter.readSlice();
	timeReal = gCounter.readTime();
	time = gTimer[1];

	for (i = 0; i < 5; i++) {
		force_right[i] = gBox.getForce(i);
	}
}




/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream& out) {
	out << state << "\t" << TR << "\t" << currentSlice << "\t" << timeReal << "\t" << time << "\t"
		<< force_left[0] << " \t" << force_left[1] << "\t" << force_left[2] << " \t" << force_left[3] << "\t" << force_left[4] << " \t"
		<< force_right[0] << " \t" << force_right[1] << "\t" << force_right[2] << " \t" << force_right[3] << "\t" << force_right[4] << " \t"
		<< endl;
}

/////////////////////////////////////////////////////////////////////////////////////
///	Graphic State
/// Collection of current variables relating to what's on the screen 
/// contains 4 lines for display 
/// 
/////////////////////////////////////////////////////////////////////////////////////
GraphicState::GraphicState() {
	for (int i = 0; i < 5; i++) {		//SEQUENCE LETTER for training
		lineXpos[i] = i * 1.4 - 2.8;
		lineYpos[i] = 4;
		lineColor[i] = 1;			// white 
		size[i] = 9;
	}

	for (int i = 0; i < 5; i++) {		//SEQUENCE LETTER for announcement
		lineXpos[i + 5] = i * 1.4 - 2.8;
		lineYpos[i + 5] = 2.3;
		lineColor[i + 5] = 1;			// white 
		size[i + 5] = 9;
	}

	lineXpos[10] = 0;
	lineYpos[10] = 4.5;			// feedback 	
	lineColor[10] = 1;				// white 
	size[10] = 5;
	lineXpos[11] = 0;
	lineYpos[11] = 3.5;				// feedback 	
	lineColor[11] = 1;				// white 
	size[11] = 5;

	lineXpos[12] = 0;
	lineYpos[12] = 0.5;			// block points	
	lineColor[12] = 1;				// white 
	size[12] = 5;
	lineXpos[13] = 0;
	lineYpos[13] = -0.5;				// total points 	
	lineColor[13] = 1;				// white 
	size[13] = 5;
	//boxOn = false;
}


void GraphicState::reset(void) {
	for (int i = 0; i < NUMDISPLAYLINES; i++) {
		line[i] = "";
	}
}
