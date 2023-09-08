///////////////////////////////////////////////////////////////
/// Working memory + Force/Speed experiment - Cortico-cerebellar
/// Ladan Shahshahani
///////////////////////////////q
////////////////////////////////

#include "CerebellumWorkingMemory.h" 
#include "StimulatorBox.h"
#include <ctime> 
#include <sstream> 

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
int gNumErrors = 0;					///< How many erros did you make during a block
int gEarly;
int gNumErrors2;
int finger[5];						///< State of each finger
double fingerf[5];                  ///< force by each finger 
//double PKarray_FS[3][3] = {0};       ///< array containing the peak forces grouped by trial type (for FS task)
//double MTarray_FS[3][3] = {0};       ///< array containing the Movement times grouped by trial type (for FS task)

//double MTarray_WM[3][2] = {0};

//double CRarray_FS[3][3] = {0};		///< array containing correct trials grouped by trial type (for FS task)
//double CRarray_WM[3][2] = {0};      ///< array containing correct trials grouped by trial type (for WM task)

//double ERarray_FS[3][3] = {0};		///< array containing correct trials grouped by trial type (for FS task)
//double ERarray_WM[3][2] = {0};       ///< array containing correct trials grouped by trial type (for WM task)

double meanPKarray_FS[3][3][100];   ///<  
double meanCRarray_FS[3][3][100];
double meanERarray_FS[100];
double medianMTarray_FS[3][3][100];

double meanERarray_WM[100];
double medianMTarray_WM[100];
//double ERarray[100];				///< Initialise ER array across blocks
int b = 0;							///< Pointer for which block is running
int gNumPointsBlock = 0;
int gNumPoints = 0;
int gTrial = 0;
int it = 0; // will be used in wm experiment for presenting digits
int ii;
int is; // counter used for peakForce
float sum; //sum of peak forces in a trial
float gavg; // average of peak forces in a trial;


double gforce_levels[3] = { 2, 5, 10 };
int gspeed_levels[3] = { 6, 10, 18 };
int gtask; // specifies the task that is running. BE CAREFUL WITH THIS
const char* force_str[3] = { "LOW", "MEDIUM", "HIGH" };
const char* speed_str[3] = { "SLOW", "MEDIUM", "FAST" };

double force_lb;    // lower bound of the force area
double force_hb;    // higher bound of the force area
double force_width; // force area width
int num_press;      // number of presses needed to be made in the trial
int SpeedL;         // speed level used in the trial in ms.


double timeTrial = 0;
int showLine = 1;         // flag showing the timed presses are starting
int showLine3 = 0;        // Default color of Force area is dark grey (showLine3 will be used as an index for myColor)
int showline4[MAX_PRESS]; // used in graphic state to set the colors for press squares in FS task
int pressedButton;// green
double responseArray[6] = { 1,1,1,1,1,1 };
double responseArray2[2] = { 1, 1 };
int gindx[2][5] = { {1, 2, 3, 4, 5}, {6, 7, 8, 9, 10} }; // global index for the pressed finger
int s;

/////////////////////////////
// Imaging-related stuff  //
/////////////////////////////
double TRTIME = 1000;			// was #define
char			counterSignal = '5';		///< ToDo: AVOID THAT What char is used to count the TR
int		        sliceNumber = 42;		///< How many slices do we have
///////////////////////////////////////////////

char TEXT[5] = { '1','2','3','4','5' };
#define RECPOS 5
#define CUE_SEQ 10
#define CUE_PRESS 5.25
#define SIZE_CUE 13
#define WIDTH_CHAR_CUE 1.5

// forceThresholds 
//#define STARTTH 0.4	// Threshold for start 0.4
#define preTH 1.0       // Press threshold 1.5
#define relTH 0.5       // Release threshold 1.2
#define maxTH 4         // max threshold  4
#define FORCEG 0.2      // force gain

// Graphics
#define FINGWIDTH 1
#define RECWIDTH 1.8
#define FORCESCALE 0.5
#define BASELINE_WM -3.5
#define BASELINE_FS 2
#define SQWIDTH 0.6 // width of the squares for press presentation
#define SQGAP 0.5 // the gap between press squares
#define CROSSWIDTH 0.1					// Fixation cross height 
#define CROSSHEIGHT 1					// Fixation cross width 

// Visualization colors
/*Color_t myColor[9]={{0,0,0}, // Black 0
{255,255,255},		// White 1
{200,0,0},			// Red  2
{0,200,0},			// Green 3
{130,130,130}, 		// gray 4
{30,30,30},			// dark gray 5
{34,102,34},		// yellow 6
{20,20,255},		// Blue 7
{255,160,122}};     // Salmon 8*/

Color_t myColor[3] = { {130,130,130}, {0,200,0}, {200,0,0} }; 		// gray 0, green 1, red 2

// timings	 
//#define FEEDBACKTIME 350
//#define ANNOUNCE_TIME 2000	// Time before go cue - for fMRI
//#define ITI_MET_TIME 350	    // between presses - for fMRI
#define TRIAL_MAX_TIME 6000    // maximum allowed time
#define COMPLETE_TIME 6000     // complete time for last trial 
#define TOO_SLOW_TIME 6000	    // The sequence was performed too slow
#define DIG_TIME 1000           // time in ms during which in WM task, a digit remains on the screen
#define COMP_TRIAL_TIME 6000    // whole trial time used in FS experiment


double THRESHOLD[3][5] = { {preTH, preTH, preTH, preTH, preTH}, {relTH, relTH, relTH, relTH, relTH}, {maxTH, maxTH, maxTH, maxTH, maxTH} };
double fGain[5] = { 1.0,1.0,1.0,1.0,1.0 };
///////////////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the robot and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	gThisInst = hThisInst;
	gExp = new MyExperiment("CerebellumWorkingMemory", "cwm", "C:/data/CerebellumWorkingMemory/cwm/");
	gExp->redirectIOToConsole();

	tDisp.init(gThisInst, 0, 0, 400, 20, 9, 2, &(::parseCommand));
	tDisp.setText("Subj:", 0, 0);

	//gScreen.init(gThisInst,1024,0,1024,768,&(::updateGraphics));	//1440
	//gScreen.init(gThisInst,1440,0,1280,1024,&(::updateGraphics));	//1440 
	//gScreen.init(gThisInst,1680,0,1280,1024,&(::updateGraphics));	//1440 
	gScreen.init(gThisInst, -1280, 0, 1280, 1024, &(::updateGraphics));
	gScreen.setCenter(Vector2D(0, 2));    // In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE)); // cm/pixel 

	// initialize s626cards 
	s626.init("c:/robotcode/calib/s626_single.txt");
	if (s626.getErrorState() == 0) {
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); //1	5			// initialize at 200 Hz update rate 
	}

	//// high force
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/flatbox_hforce_forcesensor_calib_LEFT_02-Mar-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/flatbox_hforce_forcesensor_calib_RIGHT_07-Feb-2017.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/Flatbox4_highforceNew_RIGHT_12-Jun-2019.txt");
	//gBox[1].init(BOX_RIGHT,"c:/robot/calib/right_highForce_flatBox.txt");
	//right_highForce_flatBox.txt
	///// low force
	//gBox[0].init(BOX_LEFT,"c:/robot/calib/LEFT_lowForce_FlatBox2_24-Jan-2018.txt");
	////gBox[0].init(BOX_LEFT,"c:/robot/calib/flatbox2_lowforce_LEFT_03-Mar-2017.txt");
	gBox[1].init(BOX_RIGHT, "c:/robotcode/calib/flatbox2_lowforce_RIGHT_06-Jul-2017.txt");
	//// high force 2 (new one)
	////gBox[0].init(BOX_LEFT,"c:/robot/calib/flatbox2_highforce2_LEFT_27-May-2018.txt");
	////gBox[1].init(BOX_RIGHT,"c:/robot/calib/flatbox2_highforce2_RIG HT_27-May-2018.txt");

	gBox[0].filterconst = 0.8;
	gBox[1].filterconst = 0.8;

	// initialize Timer
	gTimer.init();
	// initialize TR counter 
	gCounter.init3();
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

/////////////////////////////////////
///////////////////////////////////
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

	int i, b;
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
			//sprintf(buffer,"Force : %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[0].getForce(4),
			//	gBox[0].getForce(3),gBox[0].getForce(2),gBox[0].getForce(1),gBox[0].getForce(0));
			// RIGHT
			sprintf(buffer, "Force : %2.2f %2.2f %2.2f %2.2f %2.2f", gBox[1].getForce(0),
				gBox[1].getForce(1), gBox[1].getForce(2), gBox[1].getForce(3), gBox[1].getForce(4));

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


	// set the speed levels
	else if (arguments[0] == "SpeedLs") {
		if (numArgs == 1) {
			tDisp.print("USAGE: enter speed as number of presses: low med high");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			sscanf(arguments[3].c_str(), "%f", &arg[2]);
			gspeed_levels[0] = arg[0];
			gspeed_levels[1] = arg[1];
			gspeed_levels[2] = arg[2];
		}
	}

	// set the force levels 
	else if (arguments[0] == "ForceLs") {
		if (numArgs == 1) {
			tDisp.print("USAGE: enter force level: low med high");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[0]);
			sscanf(arguments[2].c_str(), "%f", &arg[1]);
			sscanf(arguments[3].c_str(), "%f", &arg[2]);

			gforce_levels[0] = arg[0];
			gforce_levels[1] = arg[1];
			gforce_levels[2] = arg[2];
		}
	}

	// set the task id 
	else if (arguments[0] == "task") {
		if (numArgs == 1) {
			tDisp.print("USAGE: task id. 1 WM, 2 FS");
		}
		else {
			sscanf(arguments[1].c_str(), "%f", &arg[1]);
			gtask = arg[1];

		}
	}

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
			sscanf(arguments[1].c_str(), "%d", &arg[0]);
			gs.showLines = (bool)arg[0];
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
	for (int i = 0; i < NUMDISPLAYLINES_ROWS; i++) { gs.line[i] = ""; }
	gs.boxOn = true;
	//gs.tableOn=false; 
	gCounter.reset();
	gCounter.start();
	gNumPointsBlock = 0;
	gNumErrors = 0;
	gs.clearCues();

	// clear showLines
	showLine = 1;
	showLine3 = 0;
	for (int j = 0; j < MAX_PRESS; j++) {
		showline4[j] = 0;
	}

}

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state 
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback() {

	gCounter.stop();
	int i;
	int ii; // force level/memory load
	int jj; // speed level/recall direction
	int kk; // trial type in working memory task
	int iii;
	int jjj;
	int si;
	int n[2] = { 0,0 }; // number of correct trials for FS experiment [0] and WM experiment [1].
	//int nn_fs[3][3]={0}; // number of total trials for each condition type
	//int nn_wm[3][2]={0};
	int nn_fs_g[3][3] = { 0 }; // number of trials per condition for force speed task
	int nn_fs_t = 0; // total number of trials for force speed task!
	int nn_wm_exec = 0; // one for the memory encoding trials and one for the execution trial
	int nn_wm_t = 0;
	double MTarray_FS[3][3][200]; // array with the MTs for each trial, grouped by condition type (Force-Speed combinations)
	double PKarray_FS[3][3][200]; // array with average forces for each trial grouped by condition type
	double ERarray_FS[200]; // array counting the number of error trials


	// the last dim of the following arrays represents the trial type in WM task
	double MTarray_WM[200]; // array with the MTs for each trial, grouped by condition type (Force-Speed combinations)
	double ERarray_WM[200]; // array counting the number of error trials

	//double medianMT[2]={0,0};
	MyTrial* tpnr;

	//medianMTarray[0]=10000;		// Initialise the MT for the 0th block to be 10000 (same as default)
	//ERarray[0]=0;			// Initialise the ER for the 0th block to be 0 


	for (i = 0; i < trialNum; i++) {
		tpnr = (MyTrial*)trialVec.at(i);
		//kk = tpnr->wmType;


		// grouping and counting trials based on task and conditions
		if (tpnr->trialType == 1) { // Force speed task
			ii = tpnr->Force; // which force condition
			jj = tpnr->Speed; // which speed condition
			//cout<<"nn_fs_g["<<ii<<"]"<<"["<<jj<<"]"<<" = "<<nn_fs_g[ii][jj]<<"\n";
			MTarray_FS[ii][jj][nn_fs_g[ii][jj]] = tpnr->MT; // get the MT
			PKarray_FS[ii][jj][nn_fs_g[ii][jj]] = tpnr->gavg; // get the average force for the current trial
			ERarray_FS[i] = tpnr->isError; // was it an error trial?
			nn_fs_g[ii][jj]++;
			nn_fs_t++;
		}
		else {
			// for the working memory task, memory encoding trial
			// the only thing that is important is whether or not they pressed early during memory encoding trial
			/// set the working memory load condition
			if (tpnr->wmLoad == 2) {
				ii = 0;
			}
			else if (tpnr->wmLoad == 4) {
				ii = 1;
			}
			else if (tpnr->wmLoad == 6) {
				ii = 2;
			};
			jj = tpnr->seqType;
			kk = tpnr->exec;

			if (tpnr->exec == 1) {
				MTarray_WM[nn_wm_exec] = tpnr->MT;
				ERarray_WM[nn_wm_exec] = tpnr->isError;
				nn_wm_exec++;
			}
			//ERarray_WM[i] = tpnr->isError;
			//nn_wm_t++;
		}
	}


	//--------------------------------------------------------------------------------------
	// calculate median MT, average peak force, and error rate for force/speed task
	// the code goes here:
	/// only prints out for Force speed task
	if (gtask == 2) {
		for (iii = 0; iii < 3; iii++) {
			for (jjj = 0; jjj < 3; jjj++) {
				medianMTarray_FS[iii][jjj][b] = median(MTarray_FS[iii][jjj], nn_fs_g[iii][jjj]);

				// calculating the average of all the elements in array
				int n, i2;
				float sum = 0; // sum of all the elements
				float avg; // the average
				// calculate sum
				for (i2 = 0; i2 < nn_fs_g[iii][jjj]; i2++) {
					sum = sum + PKarray_FS[iii][jjj][i2];
				}
				meanPKarray_FS[iii][jjj][b] = sum / nn_fs_g[iii][jjj];

				//meanPKarray_FS[iii][jjj][b] = average(PKarray_FS[iii][jjj], nn_fs_g[iii][jjj]);
				//meanPKarray_FS[iii][jjj][b]=0;
				//cout<<iii<<":"<<jjj<<"\n";
				//cout<<"median is: "<<medianMTarray_FS[iii][jjj][b]<<"\n";
				//cout<<"mean is: "<<meanPKarray_FS[iii][jjj][b]<<"\n";
			}
		}
		// calculate the total error rate
		// calculating the average of all the elements in array
		// calculating the average of all the elements in array
		int n, i2;
		float sum = 0; // sum of all the elements
		float avg; // the average
		// calculate sum
		for (i2 = 0; i2 < nn_fs_g[iii][jjj]; i2++) {
			sum = sum + ERarray_FS[i2];
		}
		meanERarray_FS[b] = sum / nn_fs_g[iii][jjj] * 100;
		//meanERarray_FS[b] = average(ERarray_FS, nn_fs_t)*100;
		//meanERarray_FS[b] = 0;
		cout << "rate is: " << meanERarray_FS[b] << "\n";


		// print feedback to the screen
		gNumPoints += gNumPointsBlock;
		sprintf(buffer, "Point you've got: %d   Total points: %d", gNumPointsBlock, gNumPoints);
		gs.line[10] = buffer;
		gs.lineColor[10] = 1;

		sprintf(buffer, "Error rate: %3.1f", meanERarray_FS[b]);
		gs.line[11] = buffer;
		gs.lineColor[11] = 1;

		// the final table with parameters grouped by trialType
		/*sprintf(buffer, "condition");
		gs.line[12]=buffer;
		gs.lineColor[12]=1;

		sprintf(buffer, "MT");
		gs.line[13]=buffer;
		gs.lineColor[13]=1;

		sprintf(buffer, "PF");
		gs.line[14]=buffer;
		gs.lineColor[14]=1;*/


		//
		cout << "LOW, SLOW:\n";
		cout << "MT: " << medianMTarray_FS[0][0][b] << "\n";
		cout << "PF: " << meanPKarray_FS[0][0][b] << "\n\n";
		/*sprintf(buffer, "LOW, SLOW");
		gs.line[15]=buffer;
		gs.lineColor[15]=1;

		sprintf(buffer, "%f", medianMTarray_FS[0][0][b]);
		gs.line[16]=buffer;
		gs.lineColor[16]=1;

		sprintf(buffer, "%f", meanPKarray_FS[0][0][b]);
		gs.line[17]=buffer;
		gs.lineColor[17] = 1;*/

		//
		cout << "LOW, MED:\n";
		cout << "MT: " << medianMTarray_FS[0][1][b] << "\n";
		cout << "PF: " << meanPKarray_FS[0][1][b] << "\n\n";
		/*sprintf(buffer, "LOW, MED");
		gs.line[18]=buffer;
		gs.lineColor[18]=1;

		sprintf(buffer, "%f", medianMTarray_FS[0][1][b]);
		gs.line[19]=buffer;
		gs.lineColor[19]=1;

		sprintf(buffer, "%f", meanPKarray_FS[0][1][b]);
		gs.line[20]=buffer;
		gs.lineColor[20] = 1;*/

		//
		cout << "LOW, FAST:\n";
		cout << "MT: " << medianMTarray_FS[0][2][b] << "\n";
		cout << "PF: " << meanPKarray_FS[0][2][b] << "\n\n";
		/*sprintf(buffer, "LOW, FAST");
		gs.line[21]=buffer;
		gs.lineColor[21]=1;

		sprintf(buffer, "%f", medianMTarray_FS[0][2][b]);
		gs.line[22]=buffer;
		gs.lineColor[22]=1;

		sprintf(buffer, "%f", meanPKarray_FS[0][2][b]);
		gs.line[23]=buffer;
		gs.lineColor[23] = 1;*/

		//
		cout << "MED, SLOW:\n";
		cout << "MT: " << medianMTarray_FS[1][0][b] << "\n";
		cout << "PF: " << meanPKarray_FS[1][0][b] << "\n\n";
		/*sprintf(buffer, "MED, SLOW");
		gs.line[24]=buffer;
		gs.lineColor[24]=1;

		sprintf(buffer, "%f", medianMTarray_FS[1][0][b]);
		gs.line[25]=buffer;
		gs.lineColor[25]=1;

		sprintf(buffer, "%f", meanPKarray_FS[1][0][b]);
		gs.line[26]=buffer;
		gs.lineColor[26] = 1;*/

		//
		cout << "HIGH, SLOW:\n";
		cout << "MT: " << medianMTarray_FS[2][0][b] << "\n";
		cout << "PF: " << meanPKarray_FS[2][0][b] << "\n\n";
		/*sprintf(buffer, "HIGH, SLOW");
		gs.line[27]=buffer;
		gs.lineColor[27]=1;

		sprintf(buffer, "%f", medianMTarray_FS[2][0][b]);
		gs.line[28]=buffer;
		gs.lineColor[28]=1;

		sprintf(buffer, "%f", meanPKarray_FS[2][0][b]);
		gs.line[29]=buffer;
		gs.lineColor[29] = 1;*/

	}

	//---------------------------------------------------------------------------------------
	// calculate median MT and error rate for woeking memory task
	// the code goes here:
	if (gtask == 1) {
		medianMTarray_WM[b] = median(MTarray_WM, nn_wm_exec);

		// calculating the average of all the elements in array
		int n, i2;
		float sum = 0; // sum of all the elements
		float avg; // the average
		// calculate sum
		for (i2 = 0; i2 < nn_wm_exec; i2++) {
			sum = sum + ERarray_WM[i2];
		}
		meanERarray_WM[b] = sum / nn_wm_exec * 100;


		//meanERarray_WM[b] = average(ERarray_WM, nn_wm_exec)*100;
		//meanERarray_WM[b] = 0;

		cout << "median is: " << medianMTarray_WM[b] << "\n";
		cout << "rate is: " << meanERarray_WM[b] << "\n";


		// print feedback to the screen
		gNumPoints += gNumPointsBlock;
		sprintf(buffer, "Point you've got: %d   Total points: %d", gNumPointsBlock, gNumPoints);
		gs.line[10] = buffer;
		gs.lineColor[10] = 1;

		sprintf(buffer, "Error rate: %3.1f MT: %f", meanERarray_WM[b], medianMTarray_WM[b]);
		gs.line[11] = buffer;
		gs.lineColor[11] = 1;
	}
	//---------------------------------------------------------------------------------------
	b++;
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
	hand = 0;							    // Read right box	 
	isError = 0;							// init error flag	
	gNumErrors2 = 0;

	isComplete = 0;						// init if seq was produced incomplete but correct so fare
	seqCounter = 0;						// init the sequence index variable
	seqCounterR = 5;					// this will be used for backward sequences (just for display)
	RT = 0;								// init total reaction time
	points = 0;							// init points gained
	released = 0;						// init whether fingers released
	showLine3 = 0;



	for (int i = 0; i < MAX_PRESS; i++) {
		response[i] = 0;					//		& finger response
		//respForce[i] = 0; 			// force by pressed finger
		pressTime[i] = 0;					// when is the pressed made?		
		releaseTime[i] = 0;               // when is the finger released? 
		showline4[i] = 0;
	}
}

///////////////////////////////////////////////////////////////
// Read
///////////////////////////////////////////////////////////////
void MyTrial::read(istream& in) {
	// read from .tgt file
	(in) >> startTime >> endTime >> seqNumb >> trialType >> seqType >> wmType;
	//for (int i=0;i<MAX_PRESS;i++) {
	//	(in)>>press[i];
	//}

	(in) >> hand >> cueD >> cueDm >> iti >> isScan >> wmLoad >> exec >> lastTrial >> announceTime >> feedbackTime >> ScanSess >> Force >> Speed;
	// do other job
	gTrial++;
	cTrial = gTrial;
	complete = 0;
	seqLength = cueD.length(); // get seqLength	 	
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream& out) {
	int i;
	out << startTime << "\t" << endTime << "\t" << startTimeReal << "\t"
		<< startTR << "\t" << startTRTime << "\t"
		<< seqNumb << "\t" << seqType << "\t" << wmType << "\t";

	//for (int i=0;i<MAX_PRESS;i++){
	//	out<<press[i]<<"\t";
	//}
	out << hand << "\t"
		<< cueD << "\t"
		<< wmLoad << "\t"
		<< exec << "\t"
		<< complete << "\t"
		<< iti << "\t"
		<< RT << "\t"
		<< gNumErrors2 << "\t"
		<< isError << "\t"
		<< isScan << "\t"
		<< announceTime << "\t" // announceTime is time before go cue!
		<< feedbackTime << "\t"
		<< ScanSess << "\t"
		<< ForceL << "\t"
		<< AnumPress << "\t"
		<< num_press << "\t";

	for (i = 0; i < MAX_PRESS; i++) {
		out << response[i] << "\t";
	}

	for (i = 0; i < MAX_PRESS; i++) {
		out << pressTime[i] << "\t";
	}

	out << points << "\t" << timeTrial << "\t" << endl;
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream& out) {
	char header[200];
	int i;
	out << "startTime" << "\t" << "endTime" << "\t" << "startTimeReal" << "\t"
		<< "startTR" << "\t" << "startTRTime" << "\t" << "seqNumb" << "\t"
		<< "seqType" << "\t" << "wmType" << "\t";
	//for (int i=0;i<MAX_PRESS;i++){
	//	sprintf(header,"press%d",i);
	//	out<<header<<"\t";
	//}
	out << "hand" << "\t"
		<< "cueD" << "\t"
		<< "wmLoad" << "\t"
		<< "exec" << "\t"
		<< "complete" << "\t"
		<< "iti" << "\t"
		<< "RT" << "\t"
		<< "gNumErrors2" << "\t"
		<< "isError" << "\t"
		<< "isScan" << "\t"
		<< "announceTime" << "\t"
		<< "feedbackTime" << "\t"
		<< "ScanSess" << "\t"
		<< "ForceL" << "\t"
		<< "AnumPress" << "\t"
		<< "num_press" << "\t";

	for (i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "response%d", i);
		out << header << "\t";
	}
	for (i = 0; i < MAX_PRESS; i++) {
		sprintf(header, "pressTime%d", i);
		out << header << "\t";
	}
	out << "points" << "\t" << "timeTrial" << "\t" << endl;

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

	if (wmType == 0) { // setting force/speed levels for force/speed experiment
		ForceL = gforce_levels[Force];
		num_press = gspeed_levels[Speed];
		SpeedL = COMP_TRIAL_TIME / num_press;

		force_lb = ForceL * (1 - FORCEG);
		force_hb = ForceL * (1 + FORCEG);
		force_width = force_hb - force_lb;

	}
	else if (wmType == 1) { // setting force/speed levels for the wm experiment
		ForceL = 1;
		SpeedL = 1000;
	}
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

	sprintf(buffer, "Task: %d", gtask);
	tDisp.setText(buffer, 1, 0);

	sprintf(buffer, "TR : %d time: %2.2f slice:%d", gCounter.readTR(), gCounter.readTotTime(), gCounter.readSlice());
	tDisp.setText(buffer, 2, 0);

	//sprintf(buffer,"Force LH:    %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[0].getForce(4),gBox[0].getForce(3),gBox[0].getForce(2),gBox[0].getForce(1),gBox[0].getForce(0));
	//tDisp.setText(buffer,3,0);

	sprintf(buffer, "Force RH:    %2.2f %2.2f %2.2f %2.2f %2.2f", gBox[1].getForce(0), gBox[1].getForce(1), gBox[1].getForce(2), gBox[1].getForce(3), gBox[1].getForce(4));
	tDisp.setText(buffer, 3, 0);

	sprintf(buffer, "Force Levels: %.3f    %.3f    %.3f", gforce_levels[0], gforce_levels[1], gforce_levels[2]);
	tDisp.setText(buffer, 4, 0);

	sprintf(buffer, "Force LB: %.3f    Force HB: %.3f    AVG PEAK: %.3f", force_lb, force_hb, gavg);
	tDisp.setText(buffer, 5, 0);

	sprintf(buffer, "Speed levels: %d    %d    %d", gspeed_levels[0], gspeed_levels[1], gspeed_levels[2]);
	tDisp.setText(buffer, 7, 0);

	sprintf(buffer, "trial : %d, state : %d", gExp->theBlock->trialNum + 1, state);
	tDisp.setText(buffer, 8, 0);

	sprintf(buffer, "trial time: %d ", timeTrial);
	tDisp.setText(buffer, 9, 0);

	sprintf(buffer, "gTimers: %2.0f %2.0f", gTimer[1], gTimer[2]);
	tDisp.setText(buffer, 10, 0);

}

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD 
///////////////////////////////////////////////////////////////
void MyTrial::updateGraphics(int what) {
	Vector2D crossSize(1.5, 1.5);
	Vector2D crossPos(0, 2);
	gScreen.setColor(Screen::white);

	int i, j;
	double start_pos; // is used to find the starting position of the centre of the first press square bar and speed bar
	double stop_pos;  // is used to find the ending position of the speed bar

	//------------------------------------------------------------
	// finger forces
	if (gs.showLines && state != 2) {

		// finger forces in state = 0 (similar for both tasks)
		for (i = 0; i < 5; i++) {
			j = 4 - i;
			gScreen.setColor(Screen::white);
			if (state == 0) {
				//gScreen.drawLine(j*FINGWIDTH-6.5,gBox[0].getForce(i)*2-3,j*FINGWIDTH-5.7,gBox[0].getForce(i)*2-3); // left hand	
				gScreen.drawLine(i * FINGWIDTH - 2.27, gBox[1].getForce(i) * 2 + BASELINE_WM, i * FINGWIDTH - 1.47, gBox[1].getForce(i) * 2 + BASELINE_WM); // right hand	
			}// if state = 0
			// for every other state except for state = 2 (WAIT_TR)
			else if (state > 2 && state < 13) {
				// for force speed task
				if (trialType == 1) {
					showLine3 = 0;
					gScreen.drawLine(5, BASELINE_FS, -5, BASELINE_FS); // a horizontal line at the lower level
					// show finger force lines
					for (i = 0; i < 5; i++) {
						j = 4 - i;// for the left hand
						gScreen.setColor(Screen::white);
						// set the area color to green once the force enters the force area
						if (gBox[1].getForce(i) * FORCESCALE + BASELINE_FS >= force_lb * FORCESCALE + BASELINE_FS) {
							//gScreen.setColor(myColor[0]);
							//gScreen.drawLine(i*FINGWIDTH-2.27,gBox[1].getForce(i)*FORCESCALE+BASELINE,i*FINGWIDTH-1.47,gBox[1].getForce(i)*FORCESCALE+BASELINE);
							showLine3 = 1;
						} // if > force_lb 
					}//for i

				}//if trialType == 1
				else if (trialType == 2 || trialType == 3) {
					//gScreen.drawLine(5,BASELINE_WM,-5,BASELINE_WM); // a horizontal line at the lower level
					// show finger force lines
					for (i = 0; i < 5; i++) {
						j = 4 - i;// for the left hand
						gScreen.setColor(Screen::white);
						gScreen.drawLine(i * FINGWIDTH - 2.27, gBox[1].getForce(i) * 2 + BASELINE_WM, i * FINGWIDTH - 1.47, gBox[1].getForce(i) * 2 + BASELINE_WM); // right hand

					}//for i
				}// else if trialType == 2 || trialType ==3 
			} // else if state
		} //for i

	} // if gs.showLines
	//------------------------------------------------------------
	// other letters and digits
	//************************************************************
	/// for foce speed
	// draw the digits inside the boxes
	if (wmType == 0 && state != 2) {
		// show
		for (i = 0; i < NUMDISPLAYLINES_ROWS; i++) {
			if (!gs.line[i].empty()) {
				gScreen.setColor(gs.lineColor[i]);
				gScreen.print(gs.line[i].c_str(), gs.lineXpos[i], gs.lineYpos[i], gs.size[i] * 1);
			}// if gs not empty 
		}// forr i

		// Present sequence
		// Sequence Cue
		gScreen.setColor(1);		// White
		// only two digits are presented
		for (i = 0; i < 2; i++) {
			if (gs.cuePress[i] > 0) {
				gScreen.setColor(responseArray2[i]);
				gScreen.printChar(gs.cuePress[i], (-0.65 + ((i) * 1.3) + (-2 + 2 * i + 1) * 0.1625 * (i + 1)) * WIDTH_CHAR_CUE, CUE_PRESS + 1 - 9, SIZE_CUE);
			} // if gs.
		}; // for i
	} // if wmType

	//************************************************************
	/// for working memory
	//************************************************************
	// for working memory
	// show the digits insied the box
	if (wmType == 1 && state != 2) {
		// show
		for (i = 0; i < NUMDISPLAYLINES_ROWS; i++) {
			if (!gs.line[i].empty()) {
				gScreen.setColor(gs.lineColor[i]);
				gScreen.print(gs.line[i].c_str(), gs.lineXpos[i], gs.lineYpos[i], gs.size[i] * 1);
			}
		};

		for (i = 0; i < seqLength; i++) {
			//
			if (gs.cuePress[i] > 0) {
				gScreen.setColor(responseArray[i]);
				gScreen.printChar(gs.cuePress[i], (i - 2.5) * WIDTH_CHAR_CUE, CUE_PRESS + 1, SIZE_CUE);
			}
		};
	}
	//------------------------------------------------------------
	//************************************************************

	//----------------------------------------------------
	// Setting the boxes!
	//****************************************************
	// boxes for force speed task
	if (wmType == 0 && state != 2) {
		if (((gs.boxOn) && (lastTrial == 0)) || ((gs.boxOn) && (lastTrial == 1) && state < 13 && state >2)) {


			// the force area
			// Force area for force speed task
			if (trialType == 1 && state != 2 && state != 0) {

				// setting up the force area
				//gScreen.setColor(myColor[showLine3]);
				if (showLine3 == 0) {
					gScreen.setColor(Screen::grey);
				}
				else if (showLine3 == 1) {
					gScreen.setColor(Screen::green);
				}
				else if (showLine3 == 2) {
					gScreen.setColor(Screen::red);
				} // if showline3
				gScreen.drawBox(10, FORCESCALE * (force_width), 0, FORCESCALE * ForceL + BASELINE_FS); // a box showing force area
				gScreen.setColor(Screen::forestgreen);
				gScreen.drawLine(-5, FORCESCALE * ForceL + BASELINE_FS, 5, FORCESCALE * ForceL + BASELINE_FS); // drwas a black line at the force level
				gScreen.setColor(Screen::white);

				// show the finger forces AGAIN!
				// finger forces
				// showing finger forces
				for (i = 0; i < 5; i++) {
					j = 4 - i;
					if (state == 0) {
						//gScreen.drawLine(j*FINGWIDTH-6.5,gBox[0].getForce(i)*2-3,j*FINGWIDTH-5.7,gBox[0].getForce(i)*2-3); // left hand	
						gScreen.drawLine(i * FINGWIDTH - 2.27, gBox[1].getForce(i) * FORCESCALE + BASELINE_FS, i * FINGWIDTH - 1.47, gBox[1].getForce(i) * FORCESCALE + BASELINE_FS); // right hand	
					}// if state = 0	

					// state numbers need to change:
					// States that the finger forces are to be shown are: for force speed the finger forces will be multiplied by a scalar (only display)
					if (hand == 1) {
						//gScreen.drawLine(j*FINGWIDTH-1.5,gBox[0].getForce(i)*FORCESCALE+BASELINE,j*FINGWIDTH-2.3,gBox[0].getForce(i)*FORCESCALE+BASELINE); 	
					}
					else if (hand == 2) {
						gScreen.drawLine(i * FINGWIDTH - 2.27, gBox[1].getForce(i) * FORCESCALE + BASELINE_FS, i * FINGWIDTH - 1.47, gBox[1].getForce(i) * FORCESCALE + BASELINE_FS);
					} // else if hand
				} // for i
			} // if trialType == 1
			//
			Vector2D recSize, recPos;
			recSize = Vector2D(1.3, 1.3);
			recSize = Vector2D(2.6, 2.6);
			//recPos= Vector2D(-1.3,2.95*2);
			recPos = Vector2D(-1.3, -2);
			gScreen.setColor(Screen::white);
			gScreen.drawRect(recSize, recPos);

			recSize = Vector2D(2.6, 2.6);
			//recPos= Vector2D(1.4,2.95*2);
			recPos = Vector2D(1.4, -2);
			gScreen.setColor(Screen::white);
			gScreen.drawRect(recSize, recPos);

			// the boxes turn green as a go cue in state 4 (FS_WAIT_PRESS)
			if (state == 4 && gTimer[2] > announceTime) {
				Vector2D recSize, recPos;
				recSize = Vector2D(1.3, 1.3);
				recSize = Vector2D(2.6, 2.6);
				//recPos= Vector2D(-1.3,2.95*2);
				recPos = Vector2D(-1.3, -2);
				gScreen.setColor(Screen::green);
				gScreen.drawRect(recSize, recPos);

				recSize = Vector2D(2.6, 2.6);
				//recPos= Vector2D(1.4,2.95*2);
				recPos = Vector2D(1.4, -2);
				gScreen.setColor(Screen::green);
				gScreen.drawRect(recSize, recPos);

			}; // if state == 4
			// draw press squares
			int N = COMP_TRIAL_TIME / SpeedL; //number of presses or the number of squares
			recSize = Vector2D(SQWIDTH, SQWIDTH); // SQWIDTH is set at 0.6

			for (int k = 0; k < N; k++) {

				start_pos = SQGAP + k - (N / 2); // SQGAP is set at 0.5


				recPos = Vector2D(start_pos, -2 + 2);
				//gScreen.setColor(myColor[showline4[k]]); // the color of press square changes based on whether the correct finger was pressed or not
				if (showline4[k] == 0) {
					gScreen.setColor(Screen::grey);
				}
				else if (showline4[k] == 1) {
					gScreen.setColor(Screen::green);
				}
				else if (showline4[k] == 2) {
					gScreen.setColor(Screen::red);
				}
				gScreen.drawBox(recSize, recPos);

			} // for k
			stop_pos = (SQGAP + (N - 1) - (N / 2));

			// show the speed bar!
			/// the empty speed bar is shown at all times
			/// the filling speed bar is only shown at FS_WAIT_PRESS state
			//// draw the empty speed bar
			gScreen.setColor(Screen::white);
			gScreen.drawLine(-start_pos - SQWIDTH / 2, 0.6, stop_pos + SQWIDTH / 2, 0.6);
			if ((gTimer[2] > announceTime)) {//if ((gTimer[2]>announceTime +1000)){

				if (showLine == 1) {
					if (state == 4) {
						// draw the filled portion of the speed bar only during FS_WAIT_PRESS
						gScreen.setColor(Screen::salmon);
						gScreen.drawLine(-start_pos - SQWIDTH / 2, 0.6, -start_pos - SQWIDTH / 2 + ((gTimer[2] - announceTime) * (stop_pos + start_pos + SQWIDTH) / COMP_TRIAL_TIME), 0.6);
						//gScreen.drawLine(-start_pos-SQWIDTH/2,0.6,-start_pos-SQWIDTH/2+((gTimer[2]-announceTime-1000)*(stop_pos +start_pos + SQWIDTH)/COMP_TRIAL_TIME),0.6);
					} // if state == 4 (press)
					gScreen.setColor(Screen::white);
				} // if showLine
			} // if gTimer[2]>announceTime
		} // if boxOn
		else if ((gs.boxOn) && (lastTrial == 1) && gTimer[2] > TRIAL_MAX_TIME + COMPLETE_TIME) {
			gs.reset();
		};
	} // if wmType == 0 && state !=2
	else if (wmType == 0 && state == 2) {
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// set the graphic state for WAIT_TR
		if (state == 2) {
			// only comes here when its WAIT_TR state (state == 2)
			gScreen.setColor(Screen::white);
			// fixation cross for force speed task
			if (trialType == 1 && gs.crossOn) {
				// draw the cross 
				gScreen.drawBox(CROSSWIDTH, CROSSHEIGHT, 0, BASELINE_FS - 0.8);
				gScreen.drawBox(CROSSHEIGHT, CROSSWIDTH, 0, BASELINE_FS - 0.8);

			}// if trialType == 1 && crossOn
		}// if state == 2
	}; // else if wmType == 0 and state ==2
	//******************************************************************
	// setting the boxes for working memory task
	if (wmType == 1 && state != 2) {
		if (((gs.boxOn) && (lastTrial == 0)) || ((gs.boxOn) && (lastTrial == 1) && state < 13 && state >2)) {
			//
			Vector2D recSize, recPos;
			recSize = Vector2D(1.3, 1.3);
			// setting the graphic state for memory encoding trials
			if (trialType == 2 && state != 10) {
				// draw a white rectangle around the digits they need to memorize
				if (wmLoad == 2) {
					recSize = Vector2D(2.8, 2.1);
					recPos = Vector2D(0, 3.45 * 2);
					gScreen.setColor(Screen::white);
					gScreen.drawRect(recSize, recPos);
				}
				else if (wmLoad == 4) {
					recSize = Vector2D(5.8, 2.1);
					recPos = Vector2D(0, 3.45 * 2);
					gScreen.setColor(Screen::white);
					gScreen.drawRect(recSize, recPos);
				}
				else if (wmLoad == 6) {
					recSize = Vector2D(9, 2.1);
					recPos = Vector2D(0, 3.45 * 2);
					gScreen.setColor(Screen::white);
					gScreen.drawRect(recSize, recPos);
				}; // if wmLoad
				// the red rectangle
				recSize = Vector2D(1.5 * 8, 2.6);
				recPos = Vector2D(0, 3.45 * 2);
				gScreen.setColor(Screen::red);
				gScreen.drawRect(recSize, recPos);

				// draw boxes signalling the recall direction
				/// this is stupid I know! I just want to have the 
				/// graphic state of the memory encoding trial separate from the execution trial
				recSize = Vector2D(2.6, 2.6);
				recPos = Vector2D(-7.39, 3.45 * 2);
				gScreen.drawRect(recSize, recPos);
				if (seqType == 1) {
					recPos = Vector2D(-7.39, 3.45 * 2);
					gScreen.setColor(Screen::yellow);
					gScreen.drawBox(recSize, recPos);
				} // if seqType == 1
				//backwards also on the right
				else if (seqType == 0) {
					recPos = Vector2D(-7.39, 3.45 * 2);
					gScreen.setColor(Screen::blue);
					gScreen.drawBox(recSize, recPos);
				}; // else if seqType == 0
				// draw a red Box if a press is made during memory encoding trials
				if (gEarly == 1) {
					// the red rectangle
					recSize = Vector2D(1.5 * 8, 2.6);
					recPos = Vector2D(0, 3.45 * 2);
					gScreen.setColor(Screen::red);
					gScreen.drawBox(recSize, recPos);
				};

			} // if trialType == 2 and state !=10 //Vector2D recSize, recPos;//recSize= Vector2D(1.3,1.3);
			// setting the graphic state for the execution trials
			else if (trialType == 3 && state >= 10 || state == 7) {
				// the above state are chosen so that during imaging
				// the WAIT_TR that comes before the execution trial does not show the cross
				/// show the green box
				recSize = Vector2D(1.5 * 8, 2.6);
				recPos = Vector2D(0, 3.45 * 2);
				gScreen.setColor(Screen::green);
				gScreen.drawRect(recSize, recPos);

				// show the box signaling the recall direction
				// signal forward or backwards for wm experiment
				/// forwards on the right
				// A rectangle is created to the left of the sequence
				recSize = Vector2D(2.6, 2.6);
				recPos = Vector2D(-7.39, 3.45 * 2);
				gScreen.drawRect(recSize, recPos);
				if (seqType == 1) {
					recPos = Vector2D(-7.39, 3.45 * 2);
					gScreen.setColor(Screen::yellow);
					gScreen.drawBox(recSize, recPos);
				}
				//backwards also on the right
				else if (seqType == 0) {
					recPos = Vector2D(-7.39, 3.45 * 2);
					gScreen.setColor(Screen::blue);
					gScreen.drawBox(recSize, recPos);
				};
			}; // else if trialType == 3
			gScreen.setColor(Screen::white);


		}// if boxOn
		else if ((gs.boxOn) && (lastTrial == 1) && gTimer[2] > TRIAL_MAX_TIME + COMPLETE_TIME) {
			gs.reset();
		};
	} // if wmType == 1 && state !=2
	else if (wmType == 1 && state == 2) {
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++
		// set the graphic state for WAIT_TR
		if (state == 2) {
			// only comes here when its WAIT_TR state (state == 2)
			gScreen.setColor(Screen::white);
			// fixation cross for force speed task
			if (trialType == 2) {
				// drawing the fixation cross for working memory task
				/// only before memory encoding trials
				// draw the cross 
				gScreen.drawBox(CROSSWIDTH, CROSSHEIGHT, 0, 3.45 * 2);
				gScreen.drawBox(CROSSHEIGHT, CROSSWIDTH, 0, 3.45 * 2);
			}
			else if (trialType == 3) {
				// During scanning of working memory task
				/// if the trial is an execution trial, 
				/// the box from the previous trial (a memory encoding trial should remain on the screen)
				//// draw a red box
				//// draw digits from the end of the memory encoding trial
				//// draw a box signalling recall direction
				//// draw a rectangle around the digits they need to memorize
				//// show the finger forces and the baseline line???????????????????????????????????????????


				//// define the size of the red box and draw
				Vector2D recSize, recPos;
				recSize = Vector2D(1.3, 1.3);

				recSize = Vector2D(1.5 * 8, 2.6);
				recPos = Vector2D(0, 3.45 * 2);
				gScreen.setColor(Screen::red);
				gScreen.drawRect(recSize, recPos);

				//// draw the box signalling the recall direction
				///// the box is drawn to the left of the sequence
				recSize = Vector2D(2.6, 2.6);
				recPos = Vector2D(-7.39, 3.45 * 2);
				gScreen.setColor(Screen::red);
				gScreen.drawRect(recSize, recPos);
				if (seqType == 1) {
					recPos = Vector2D(-7.39, 3.45 * 2);
					gScreen.setColor(Screen::yellow);
					gScreen.drawBox(recSize, recPos);
				}
				else if (seqType == 0) {
					recPos = Vector2D(-7.39, 3.45 * 2);
					gScreen.setColor(Screen::blue);
					gScreen.drawBox(recSize, recPos);
				};

				//// draw rectangle around the digits they need to memorize
				if (wmLoad == 2) {
					recSize = Vector2D(2.8, 2.1);
					recPos = Vector2D(0, 3.45 * 2);
					gScreen.setColor(Screen::white);
					gScreen.drawRect(recSize, recPos);
				}
				else if (wmLoad == 4) {
					recSize = Vector2D(5.8, 2.1);
					recPos = Vector2D(0, 3.45 * 2);
					gScreen.setColor(Screen::white);
					gScreen.drawRect(recSize, recPos);
				}
				else if (wmLoad == 6) {
					recSize = Vector2D(9, 2.1);
					recPos = Vector2D(0, 3.45 * 2);
					gScreen.setColor(Screen::white);
					gScreen.drawRect(recSize, recPos);
				};

				//// draw digits
				for (i = 0; i < seqLength; i++) {
					gScreen.setColor(responseArray[i]);
					gScreen.printChar(gs.cuePress[i], (i - 2.5) * WIDTH_CHAR_CUE, CUE_PRESS + 1, SIZE_CUE);
				}// for i

			}// if else if trialType

		}// if state == 2
	}; // else if wmType == 0 and state ==2	

} // void function
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
	//cout<<"Im here "<<dataman.isRecording()<<"\n";
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
	int i, f, j;
	Vector2D recSize;
	Vector2D recPos;

	//check finger presses
	double force;
	double critTime;
	int numNewpress = 0;			// is there a new press? 
	int pressedFinger = 0;
	released = 0;

	// variables that will be used to separate the digits of cueM and put them in a vector
	int ii;
	int iii;
	int iwm; // index representing WM load 
	int pair_repeat;
	string pair;
	string cueM; // the final seq of numbers as an integer

	if (wmType == 0) { // force/speed
		//pair_repeat = num_press/2; // number of times the pair of digits should be repeated
		pair_repeat = MAX_PRESS / 2; // number of times the pair of digits should be repeated
		pair = cueDm; // the pair of digits read from target file
	};
	// display time during the memory phase
	double dTime;

	// Tasks can be determined based on either wmType or trialType
	// 1: force speed task
	// 2: working memory task: memory encoding phase
	// 3: working memory task: execution phase
	// 4: rest. This is only if I decide to have rest as a separate trial.
	if (wmType == 1) {
		// WM task:
		for (f = 0; f < 5; f++) {
			force = gBox[hand - 1].getForce(f) * fGain[f];

			if (finger[f] == 0 && force > THRESHOLD[0][f]) {
				// Press threshold comparison
				numNewpress++;
				pressedFinger = f + 1;
				finger[f] = 1;
				released = 0;
			}
			else if (force < THRESHOLD[1][f]) {
				// Release threshold comparison
				finger[f] = 0;
				released++;
			}
		}
	}
	else if (wmType == 0) {
		// Force speed task:
		for (f = 0; f < 5; f++) {
			force = gBox[1].getForce(f) * fGain[f];
			if (peakForcef[f] < force) {
				peakForcef[f] = force;
			}
			if (finger[f] == 0 && force >= (ForceL * (1 - FORCEG))) {

				// Press threshold comparison
				numNewpress++;
				pressedFinger = f + 1;
				finger[f] = 1;
				fingerf[f] = force;
				released = 0;
			}

			else if ((force) < (ForceL * (1 - FORCEG))) {
				// Release threshold comparison
				finger[f] = 0;
				fingerf[f] = 0;
				released++;

			}
		}

	}
	switch (state) {
	case WAIT_TRIAL: //1: when program hasn't started, program waits here

		// the cross
		gs.reset();

		for (i = 0; i < NUMDISPLAYLINES_ROWS; i++) {
			if (!gs.line[i].empty()) {
				gs.lineColor[i] = 0;
				gs.line[i] = "";
			}
		}
		gs.clearCues();
		gs.crossOn = true;

		timeTrial = 0;
		break;
	case START_TRIAL: //2: when program started, this is run once for every trial
		// initialize the variables and measures for a trial
		gNumErrors2 = 0; // number of errors made in a trial
		gEarly = 0;
		gTimer.reset(1);
		gTimer.reset(2);
		if (wmType == 0) {
			gs.clearCues();
		}
		else if (wmType == 1) {
			if (trialType == 2) {
				gs.clearCues();

			}
		};
		gavg = 0; // reset the average peak force in the trial
		for (i = 0; i < MAX_PRESS; i++) {
			response[i] = 0;
			pressTime[i] = 0;
			releaseTime[i] = 0;
			showline4[i] = 0; // default color of the squares is set to grey
			// reset peak force
			peakForce[i] = 0;
		}


		//------------------------------------------------------------------------------
		// create the whole sequence using the pair (in cueDm) and the number of presses
		if (wmType == 0) {//force speed
			for (ii = 0; ii < pair_repeat; ii++) {
				cueM = cueM + pair;
			}
		}
		else if (wmType == 1) {//wmType
			cueM = cueDm;
		}

		// set up the sequence
		for (iii = 0; iii < MAX_PRESS; iii++) {
			int ia = cueM[iii] - '0'; // converting ascii to number 
			PP.push_back(ia);
			//cout<<PP.at(iii)<<endl;
		}
		//-------------------------------------------------------------------------------

		it = 0;      // reset it counter for trial (used in WM experiment)

		dataman.clear();			// clear screen

		if (isScan > 0) {
			state = WAIT_TR;
		}
		else if (wmType == 0) { // FS task
			state = FS_WAIT_ANNOUNCE;
		}
		else if (wmType == 1) { // WM task
			if (trialType == 2) {
				// memory encoding trial
				// goes on to annnounce the trial
				// during trial announcement, the box around memory digit, 
				// the box signaling recall direction and the big box are shown
				state = WM_WAIT_ANNOUNCE;

			}
			else if (trialType == 3) {
				// execution trial
				// if it's an execution trial, then it goes right straight to the WM_WAIT_PRESS
				state = WM_WAIT_PRESS;
			}
			//state = WM_WAIT_ANNOUNCE;

		}
		gTimer.reset(1);					//time for whole trial
		gTimer.reset(2);
		break;

	case WAIT_TR: //3

		if ((gCounter.readTR() > 0 && gCounter.readTotTime() >= startTime)) {

			startTimeReal = gCounter.readTotTime();
			startTR = gCounter.readTR();
			startTRTime = gCounter.readTime();



			if (wmType == 0) {

				// then it is force speed experiment,
				// go to WAIT_ANNONCE routine for the force speed experiment
				state = FS_WAIT_ANNOUNCE;
				// the cross
				gs.crossOn = true;
			}
			else if (wmType == 1) {
				// then it is WM experiment, 
				//go to WAIT_ANNOUNCE routine for wm experiment
				if (trialType == 3) {
					gs.crossOn = true;
					// goes right straight to press when it's an execution trial
					state = WM_WAIT_PRESS;
					//gs.boxOn = false;
				}
				else if (trialType == 2) {
					gs.crossOn = false;
					// goes right straight to anoouncing the trial when it's memory 
					state = WM_WAIT_ANNOUNCE;
					//gs.boxOn = true;
				}

			}
		}
		break;
	case FS_WAIT_ANNOUNCE: //4: WAIT_ANNONCE for force speed experiment

		//gNumErrors2 = 0;
		dataman.startRecording();
		gTimer.reset(1);					//time for whole trial
		gTimer.reset(2);					//time for events in the trial
		//showLine=1;
		// Announce the sequence and the speed. 
		// The force level threshold is also shown using a rectangle that
		// defines the area that the force line need to get into
		j = 0;
		for (i = 0; i < 2; i++) {
			gs.cuePress[i + j] = cueD.at(i);
		}

		//sprintf(buffer, speed_str[Speed]); // printing out the speed condition: slow, medium, fast
		//gs.line[9] = buffer;

		state = FS_WAIT_PRESS;

		break;
	case FS_WAIT_PRESS: // 5: WAIT_PRESS for FS task
		showLine = 1;

		if (gTimer[2] > announceTime && gTimer[2] < COMP_TRIAL_TIME + announceTime) {//if (gTimer[2]>announceTime && gTimer[2]<COMP_TRIAL_TIME + announceTime+1000) {

			sprintf(buffer, ""); // clear the announcement 
			gs.line[9] = buffer;


			peakForce[seqCounter] = peakForcef[pressedFinger - 1];
			//if (numNewpress>0 && seqCounter <= (COMP_TRIAL_TIME/SpeedL)) {
			if (numNewpress > 0) {

				response[seqCounter] = pressedFinger;

				pressTime[seqCounter] = gTimer[1];
				//cout<<" Detected peak force is: "<<peakForce[seqCounter]<<"\n";

				if (response[seqCounter] == PP.at(seqCounter)) {
					//if (resp.at(seqCounter)==PP.at(seqCounter)){

					showline4[seqCounter] = 1;// square turns green
					pressedButton++;
					isError = 0;
					gNumErrors2 = gNumErrors2 + 0;

				}
				else if (response[seqCounter] != PP.at(seqCounter)) {

					showline4[seqCounter] = 2;// square turns red
					pressedButton++;
					isError = 1;
					gNumErrors2 = gNumErrors2 + 1;

				};

				seqCounter++;

			}
		};

		if (gTimer[2] >= COMP_TRIAL_TIME + announceTime) {//if (gTimer[2]>=COMP_TRIAL_TIME + announceTime+1000){
			state = FS_WAIT_RELEASE; // when all the necessary pressses are done, it goes to WAIT_RELEASE
		};
		break;


	case FS_WAIT_RELEASE: //6:WAIT_RELEASE for fs experiment
		//showLine = 0;

		AnumPress = seqCounter;
		MT = pressTime[seqCounter - 1] - pressTime[0]; // movement time:how long it took to do the number of presses required
		RT = pressTime[0];

		// get the MT and add it to the MT array for the run
		//MTarray_FS[Force][Speed] = MTarray_FS[Force][Speed] + MT;

		int is;
		float sum;
		//float avg;

		// Calculate the average of peakForce
		sum = 0;
		for (is = 0; is < seqCounter; is++) {
			sum = sum + peakForce[is];
		}
		gavg = sum / seqCounter;

		// get the average peak force and add it to the peak force array for the run
		//PKarray_FS[Force][Speed] = PKarray_FS[Force][Speed] + gavg;

		if (seqCounter < num_press) {
			//
			isError = 1;
			//ERarray_FS[Force][Speed] = ERarray_FS[Force][Speed] + 1;
			points = 0;
			//gNumErrors++;
			sprintf(buffer, "TOO SLOW! +%d", points);
			gs.line[9] = buffer;
			//sprintf(buffer,"+%d",points);
			//gs.lineColor[1] = 1;
			//gs.line[1] = buffer;
		}
		else if (seqCounter > num_press && gNumErrors2 == 0) {
			isError = 1;
			//ERarray_FS[Force][Speed] = ERarray_FS[Force][Speed]+ 1;
			points = 0;
			//gNumErrors++;
			sprintf(buffer, "TOO FAST! +%d", points);
			gs.line[9] = buffer;
			//sprintf(buffer,"+%d",points);
			//gs.lineColor[1] = 1;
			//gs.line[1] = buffer;

		}
		else if (seqCounter > num_press && gNumErrors2 > 0) {
			isError = 1;
			//ERarray_FS[Force][Speed] = ERarray_FS[Force][Speed]+ 1;
			points = 0;
			//gNumErrors++;
			sprintf(buffer, "TOO FAST! +%d", points);
			gs.line[9] = buffer;
			//sprintf(buffer,"+%d",points);
			//gs.lineColor[1] = 1;
			//gs.line[1] = buffer;
		}
		else if (seqCounter == num_press && MT <= COMP_TRIAL_TIME / 2) {
			isError = 1;
			//ERarray_FS[Force][Speed] = ERarray_FS[Force][Speed]+ 1;
			points = 0;
			//gNumErrors++;
			sprintf(buffer, "TOO FAST! +%d", points);
			gs.line[9] = buffer;
			//sprintf(buffer,"+%d",points);
			//gs.lineColor[1] = 1;
			//gs.line[1] = buffer;

		}
		else if (seqCounter == num_press && gNumErrors2 > 0) {
			isError = 1;
			//ERarray_FS[Force][Speed] = ERarray_FS[Force][Speed]+ 1;
			//gNumErrors++;
			points = 0;
			sprintf(buffer, "+%d", points);
			gs.lineColor[9] = 1;
			gs.line[9] = buffer;
		}
		else if (seqCounter == num_press && gNumErrors2 == 0) {
			isError = 0;
			//ERarray_FS[Force][Speed] = ERarray_FS[Force][Speed]+ 0;
			//gNumErrors = 0;
			points = 4;
			sprintf(buffer, "+%d", points);
			gs.lineColor[9] = 1;
			gs.line[9] = buffer;
		}
		gTimer.reset(2);
		state = FS_WAIT_FEEDBACK;
		break;
	case FS_WAIT_FEEDBACK: //7: WAIT_FEEDBACK for fs experiment
		//keep feedback on screen for specified time		
		pressedButton = 0;
		//showLine = 1;
		gNumErrors += isError;

		// reset clear the screen
		if (gTimer[2] > feedbackTime) {

			gNumPointsBlock += points;
			//gNumErrors = gNumErrors+isError;

			state = WAIT_ITI;
			// clear all the lines that have been used to present feedback
			gs.lineColor[1] = 1;
			gs.line[1] = buffer;
			gs.line[1] = "";
			gs.line[0] = "";
			gs.line[9] = "";
		}
		break;
	case WAIT_ITI:  //7
		showLine = 0;

		// do the iti
		if (
			//(isScan==0)&&(gTimer[2]>(iti+FEEDBACKTIME))
			(isScan == 0) && (gTimer[2] > (iti + feedbackTime))
			||
			(isScan > 0) && (lastTrial == 0) // for scanning session, iti is dealt with manually setting startSlice/startTR
			||
			(isScan > 0) && (lastTrial == 1) && (gTimer[2] > COMPLETE_TIME) // but for last trial wait until gTimer[2] passes complete time
			)

		{
			//showCross = 1;

			dataman.stopRecording();
			timeTrial = gTimer[1];

			state = END_TRIAL;
		}
		if (lastTrial == 1) {
			gs.clearCues();
		}
		break;

	case WM_WAIT_ANNOUNCE: // 8: WAIT_ANNONCE for WM experiment
		//cout<<"Im here \n";
		gEarly = 0;
		gNumErrors2 = 0;
		dataman.startRecording();
		gTimer.reset(1);					//time for whole trial
		//gTimer.reset(2);					//time for events in the trial	
		// if it's in the memory phase, then it does not need to go to WAIT_PRESS state and if the participant pressed any key
		// then it needs to issue a warning saying "too early"
		if (gTimer[2] > announceTime) {
			gTimer.reset(2);
			if (trialType == 2) {
				state = WM_WAIT_DIGIT;
			}
			else if (trialType == 3) {
				state = WM_WAIT_PRESS;
			};
		}
		break;

	case WM_WAIT_DIGIT: //9
		// routine for showing digits sequentially
		// comes here only if it is memory encoding phase
		// Announce the sequence

		if (numNewpress > 0) {
			//gEarly = 1; // this flag is set to 1 if they press early
			// during the memory phase, it comes here.
			// if the participant presses any key during this phase, he will receive
			// a feedback saying too early and loses 1 point. 
			// otherwise it goes to the next trial, which will be the execution phase.
			gEarly = 1;
			points = 0;
			isError = 1;
			sprintf(buffer, "TOO EARLY");
			gs.line[0] = buffer;
			sprintf(buffer, "-1");
			gs.line[1] = buffer;
			state = WM_WAIT_RELEASE;
			//state = WM_WAIT_FEEDBACK;
		}
		else {

			if (gTimer[2] < DIG_TIME && it <= seqLength) {
				gs.cuePress[it] = cueDm.at(it);
			}
			else if (gTimer[2] >= DIG_TIME && it <= seqLength) {
				gs.cuePress[it] = cueD.at(it); // cueD is the sequence as it will be shown on the screen
				it++;
				//cout<<"it "<<it<<"\n";
				gTimer.reset(2);

			};

			if (it == seqLength) {
				state = WM_WAIT_FEEDBACK;
			};
		}

		break;

	case WM_WAIT_PRESS: // 10 waits for presses during execution phase of the wm task
		// WM task:
		dataman.startRecording();

		// show the press cue
		for (i = 0; i < seqLength; i++) {
			gs.cuePress[i] = cueD.at(i);
		}

		// Checks each finger and see if it was pressed correctly
		if (gEarly == 1) {
			//cout<<"here\n";
			state = WM_WAIT_RELEASE;
		}
		else {
			//cout<<"should not be here\n";
			if (exec == 1) {
				// Execution phase
				if (numNewpress > 0 && seqCounter < seqLength) {
					response[seqCounter] = pressedFinger;
					pressTime[seqCounter] = gTimer[1];

					// for the forward sequences and backward sequences 
					if (seqType == 0) {
						// backwards
						seqCounterRi = seqCounterR - seqCounter;
						//if (response[seqCounter]==press[seqCounter]){
						if (response[seqCounter] == PP.at(seqCounterRi)) {
							pressedButton++;
							//isError = 0;
							gNumErrors2 = gNumErrors2 + 0;
							responseArray[seqCounterRi] = 3; // green	

						}
						else {
							pressedButton++;
							//isError = 1; 
							gNumErrors2 = gNumErrors2 + 1;
							responseArray[seqCounterRi] = 2; // red								
						};
					}
					else if (seqType == 1) {
						// forwards
						//if (response[seqCounter]==press[seqCounter]){
						if (response[seqCounter] == PP.at(seqCounter)) {
							pressedButton++;
							//isError = 0;
							gNumErrors2 = gNumErrors2 + 0;
							responseArray[seqCounter] = 3; // green
						}
						else {
							pressedButton++;
							//isError = 1;
							gNumErrors2 = gNumErrors2 + 1;
							responseArray[seqCounter] = 2; // red 
						};
					}
					seqCounter++;

				} // IF(numNewpress>0)

				if ((seqCounter < seqLength && gTimer[2]>(TRIAL_MAX_TIME + announceTime))) {
					isError = 1;

					sprintf(buffer, "TOO SLOW");
					gs.line[1] = buffer;
					gTimer.reset(2);
					state = WM_WAIT_FEEDBACK;
				}
				else if (seqCounter == seqLength && released == 5 && gTimer[2] > TRIAL_MAX_TIME + announceTime) {
					state = WM_WAIT_RELEASE;
				}
				break;
			};
		}


	case WM_WAIT_RELEASE: // 11 waits for releases during execution phase of the wm task
		//isError = 0;
		AnumPress = seqCounter;

		// set the index corresponding to working memory load

		if (wmLoad == 2) {
			iwm = 0;
		}
		else if (wmLoad == 4) {
			iwm = 1;
		}
		else if (wmLoad == 6) {
			iwm = 2;
		}

		if (gEarly == 1) {
			gTimer.reset(2);
			isError = 1;
			gNumErrors2 = 6;
			points = -1;
			sprintf(buffer, "TOO EARLY");
			gs.line[0] = buffer;
			sprintf(buffer, "-1");
			gs.line[1] = buffer;
			state = WM_WAIT_FEEDBACK;
		}
		else {

			// working memory task
			// check if done & present feedback
			if (released == 5) {
				//MT = gTimer[1]-pressTime[0]; // movement time 
				MT = pressTime[seqCounter - 1] - pressTime[0]; // movement time:how long it took to do the number of presses required
				//cout<< "MT: "<<MT<<"\n";
				RT = pressTime[0]; // RT is the time of the first press

				//MTarray_WM[iwm][seqType] = MTarray_WM[iwm][seqType] + MT;


				critTime = timeTrial;

				if (critTime < TOO_SLOW_TIME && (gNumErrors2 == 0)) {
					points = 4;
					//gNumErrors = 0;
					isError = 0;
					//ERarray_WM[iwm][seqType] = ERarray_WM[iwm][seqType] + 0;
				}
				else if (critTime < TOO_SLOW_TIME && (gNumErrors2 == 1)) {
					points = 3;
					//ERarray_WM[iwm][seqType] = ERarray_WM[iwm][seqType] + 1;
					isError = 1;
					//gNumErrors++;
					//gNumErrors = gNumErrors + 1;
				}
				else if (critTime < TOO_SLOW_TIME && (gNumErrors2 == 2)) {
					points = 2;
					//ERarray_WM[iwm][seqType] = ERarray_WM[iwm][seqType] + 1;
					isError = 1;
					//gNumErrors++;
					//gNumErrors = gNumErrors + 1;
				}
				else if (critTime < TOO_SLOW_TIME && (gNumErrors2 == 3)) {
					points = 1;
					isError = 1;
					//ERarray_WM[iwm][seqType] = ERarray_WM[iwm][seqType] + 1;
					//gNumErrors++;
					//gNumErrors = gNumErrors + 1;
				}
				else if (critTime < TOO_SLOW_TIME && (gNumErrors2 > 3)) {
					isError = 1;
					points = 0;
					//ERarray_WM[iwm][seqType] = ERarray_WM[iwm][seqType] + 1;
					//gNumErrors++;
					//gNumErrors = gNumErrors + 1;
				} //else if (critTime>TOO_SLOW_TIME){

				if ((gTimer[2] > (TOO_SLOW_TIME + announceTime) && seqCounter < seqLength)) {
					isError = 1;
					//cout<<"here2\n";
					//ERarray_WM[iwm][seqType] = ERarray_WM[iwm][seqType] + 1;
					//gNumErrors++;
					//gNumErrors = gNumErrors + 1;
					sprintf(buffer, "TOO SLOW");
					gs.line[1] = buffer;
				}
				else {
					sprintf(buffer, "+%d", points);
					gs.lineColor[1] = 1;
					gs.line[1] = buffer;
					//	gs.clearCues(); 
				}

				gTimer.reset(2);
				state = WM_WAIT_FEEDBACK;
			}
		}
		break;


	case WM_WAIT_FEEDBACK: // 12 shows feedback: does not come here after memory trials

		if (numNewpress > 0) {
			gEarly = 1;
			isError = 1;
			points = -1;
			sprintf(buffer, "TOO EARLY");
			gs.line[0] = buffer;
			sprintf(buffer, "-1");
			gs.line[1] = buffer;

		};
		//keep feedback on screen for specified time
		it = 0;
		pressedButton = 0;
		//gEarly = 0;

		for (i = 0; i < MAX_PRESS; i++) {
			responseArray[i] = 1;
		}


		// reset clear the screen
		if (gTimer[2] >= feedbackTime) {
			gNumPointsBlock += points;
			gNumErrors = gNumErrors + isError;

			state = WAIT_ITI;
			gs.lineColor[1] = 1;
			gs.line[1] = buffer;
			gs.line[1] = "";
			gs.line[0] = "";

		}
		//


		break;

	case END_TRIAL: //13
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
		<< force_left[0] << " \t" << force_left[1] << "\t" << force_left[2] << " \t" << force_left[3] << "\t" << force_left[4] << " \t"
		<< force_right[0] << " \t" << force_right[1] << "\t" << force_right[2] << " \t" << force_right[3] << "\t" << force_right[4] << " \t" << endl;
}
/////////////////////////////////////////////////////////////////////////////////////
///	Graphic State
/// Collection of current variables relating to what's on the screen 
/// contains 20 lines for display 
/// 
/////////////////////////////////////////////////////////////////////////////////////
GraphicState::GraphicState() {
	int i;
	//int ic;
	int ir;

	//-------------------------------------------------------------------
		// define text lines for screen
		// the first 10 lines are reserved for presenting feedbacks during run
	lineXpos[0] = 0;
	lineYpos[0] = 10;
	lineColor[0] = 1;			// white 
	size[0] = 5;
	// rest of the lines identical but y position changes
	for (i = 1; i < 10; i++)
	{
		lineXpos[i] = lineXpos[0];
		lineYpos[i] = 10 - i;
		lineColor[i] = lineColor[0];
		size[i] = size[0];
	}
	//
//-------------------------------------------------------------------
	// The rest of the lines will be used to present feedback at the end of the run
	// points in block
	///points
	lineXpos[10] = 0;
	lineYpos[10] = 10;
	lineColor[10] = 1;
	size[10] = 7;

	// error rate
	lineXpos[11] = 0;
	lineYpos[11] = 9;
	lineColor[11] = 1;
	size[11] = 7;

	// starting the table
	// For Force Speed task the feedback is givern on a candition-type basis
	/// write the column name: Cond
	lineXpos[12] = -6;
	lineYpos[12] = 7;
	lineColor[12] = 1;
	size[12] = 5;

	/// write the column name: MT (movement time)
	lineXpos[13] = 0;
	lineYpos[13] = 7;
	lineColor[13] = 1;
	size[13] = 5;

	/// write the column name: PF (peak force)
	lineXpos[14] = 6;
	lineYpos[14] = 7;
	lineColor[14] = 1;
	size[14] = 5;

	// others
	lineXpos[15] = -6;
	lineYpos[15] = 6;
	lineColor[15] = 1;
	size[15] = 5;

	/// write the column name: MT (movement time)
	lineXpos[16] = 0;
	lineYpos[16] = 6;
	lineColor[16] = 1;
	size[16] = 5;

	/// write the column name: PF (peak force)
	lineXpos[17] = 6;
	lineYpos[17] = 6;
	lineColor[17] = 1;
	size[17] = 5;

	//
	lineXpos[18] = -6;
	lineYpos[18] = 5;
	lineColor[18] = 1;
	size[18] = 5;

	/// write the column name: MT (movement time)
	lineXpos[19] = 0;
	lineYpos[19] = 5;
	lineColor[19] = 1;
	size[19] = 5;

	/// write the column name: PF (peak force)
	lineXpos[20] = 6;
	lineYpos[20] = 5;
	lineColor[20] = 1;
	size[20] = 5;


	//
	lineXpos[21] = -6;
	lineYpos[21] = 4;
	lineColor[21] = 1;
	size[21] = 5;

	/// write the column name: MT (movement time)
	lineXpos[22] = 0;
	lineYpos[22] = 4;
	lineColor[22] = 1;
	size[22] = 5;

	/// write the column name: PF (peak force)
	lineXpos[23] = 6;
	lineYpos[23] = 4;
	lineColor[23] = 1;
	size[23] = 5;

	//
	lineXpos[24] = -6;
	lineYpos[24] = 3;
	lineColor[24] = 1;
	size[24] = 5;

	/// write the column name: MT (movement time)
	lineXpos[25] = 0;
	lineYpos[25] = 3;
	lineColor[25] = 1;
	size[25] = 5;

	/// write the column name: PF (peak force)
	lineXpos[26] = 6;
	lineYpos[26] = 3;
	lineColor[26] = 1;
	size[26] = 5;

	//
	lineXpos[27] = -6;
	lineYpos[27] = 2;
	lineColor[27] = 1;
	size[27] = 5;

	/// write the column name: MT (movement time)
	lineXpos[28] = 0;
	lineYpos[28] = 2;
	lineColor[28] = 1;
	size[27] = 5;

	/// write the column name: PF (peak force)
	lineXpos[29] = 6;
	lineYpos[29] = 2;
	lineColor[29] = 1;
	size[29] = 5;



	//clearCues();
	boxOn = false;
	tableOn = false;
	showLines = true;
}


void GraphicState::clearCues(void) {
	int i;
	cueSeq[0] = 0;

	for (i = 0; i < MAX_PRESS; i++) {
		cuePress[i] = 0;
	}
}

void GraphicState::reset(void) {
	for (int i = 0; i < NUMDISPLAYLINES_ROWS; i++) {
		line[i] = "";
	}
}
