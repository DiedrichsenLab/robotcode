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

#include "SequenceLearning5.h" 
#include "StimulatorBox.h" 
///////////////////////////////////////////////////////////////
/// Global variables 
///////////////////////////////////////////////////////////////
S626sManager s626;				///< Hardware Manager 
TextDisplay tDisp;				///< Text Display
Screen gScreen;					///< Screen 
TRCounter gCounter;				///< TR Counter 
StimulatorBox gBox[2];			///< Stimulator Box

//StimulatorBox gBox[2];		///< Stimulator Box 
Timer gTimer(UPDATERATE);		///< Timer from S626 board experiments 
HapticState hs;					///< This is the haptic State as d by the interrupt 
///< For Thread safety this SHOULD NOT be assessed While 
///< the interrupt is running. Use Thread-safe copy to 
///< Get the current haptic state for graphical display 
GraphicState gs; 

char buffer[300];					///< String buffer 
HINSTANCE gThisInst;					///< Instance of Windows application 
Experiment *gExp;					///< Pointer to myExperiment 
Trial *currentTrial;					///< Pointer to current Trial 
bool gKeyPressed;					///< Key pressed? 
char gKey;						///< Which key?
int isPractice=0;					///< Should we show the lines to help practice
int gNumErrors=0;					///< How many erros did you make during a block

double timeThresholds[2]= {0.8, 1.2};	///< percentage when super fast and when late trial

int gNumPointsBlock= 0;
int gNumPoints= 0;
int ghardPress=0;	
int glatePress=0;
double avrgMT[2]={0, 0}; 
double stMT=5000;		
double seqMT[2][17]=	{{stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT, stMT},
						{stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT, stMT}};
double seqGood[2][17]=	{{0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0}, 
						{0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0}};
double seqForce[2][17]=	{{0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0},
						{0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0}};
double seqFingerForce[][5]={{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},
							{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

#define TRTIME 2660//2700
char counterSignal= '5';		///< ToDo: AVOID THAT What char is used to count the TR
int sliceNumber= 32;			///< How many slices do we have

#define FEEDBACKTIME 800 
string TEXT[11]={"*","1","2","3","4","5","6", "7", "8", "9", "+"}; 
string FINGERSOUND[6]= {"A.wav", "C.wav", "D.wav", "E.wav", "G.wav"};
int STIM_INTENSITY[5]= {5, 5, 5 ,5, 5};
#define resTH 3     // in NEWTONS 0.6 * 4.9276 //0.5 * 4.9276
#define relTH 2.5   // 0.45 * 4.9276 
#define maxTH 20    //  5.0 * 4.9276 //5* 4.9276 //1.8 * 4.9276  (1.8 is too low--SWM) 2.8
double THRESHOLD[3][5]= {{resTH, resTH, resTH, resTH, resTH}, {relTH, relTH, relTH, relTH, relTH}, {maxTH, maxTH, maxTH, maxTH, maxTH}};
int SEQ[][5]={{0,0,0,0,0},
{5,3,4,2,1}, // 1
{5,3,1,2,4}, // 2
{5,2,1,3,4}, // 3
{5,1,2,4,3}, // 4
{4,5,3,1,2}, // 5
{4,5,1,3,2}, // 6
{4,3,5,1,2}, // 7
{4,2,5,3,1}, // 8 
{4,1,3,5,2}, // 9
{4,1,2,5,3}, // 10
{3,5,4,2,1}, // 11
{3,5,2,1,4}, // 12
{3,5,1,4,2}, // 13
{3,2,5,1,4}, // 14
{3,1,5,2,4}, // 15
{3,1,2,5,4}, // 16
{3,1,4,2,5}, // 17
{2,4,3,5,1}, // 18
{2,3,5,4,1}, // 19
{2,5,3,1,4}, // 20
{2,5,4,1,3}, // 21
{2,1,3,5,4}, // 22
{1,4,3,5,2}, // 23
{1,4,2,5,3}, // 24
{1,3,2,4,5}, // 25
{1,3,5,4,2}, // 26
{1,2,4,3,5}, // 27
{1,5,3,2,4}, // 28
{1,5,4,2,3}, // 29
// The Inversed Sequences
{1,3,2,4,5}, // 1' - 30
{1,3,5,4,2}, // 2'
{1,4,5,3,2}, // 3'
{1,5,4,2,3}, // 4'
{2,1,3,5,4}, // 5'
{2,1,5,3,4}, // 6' - 35
{2,3,1,5,4}, // 7'
{2,4,1,3,5}, // 8'
{2,5,3,1,4}, // 9'
{2,5,4,1,3}, // 10'
{3,1,2,4,5}, // 11' - 40
{3,1,4,5,2}, // 12'
{3,1,5,2,4}, // 13'
{3,4,1,5,2}, // 14'
{3,5,1,4,2}, // 15'
{3,5,4,1,2}, // 16' - 45
{3,5,2,4,1}, // 17'
{4,2,3,1,5}, // 18'
{4,3,1,2,5}, // 19'
{4,1,3,5,2}, // 20'
{4,1,2,5,3}, // 21' - 50
{4,5,3,1,2}, // 22'
{5,2,3,1,4}, // 23'
{5,2,4,1,3}, // 24'
{5,3,4,2,1}, // 25'
{5,3,1,2,4}, // 26' - 55
{5,4,2,3,1}, // 27'
{5,1,3,4,2}, // 28'
{5,1,2,4,3}, // 29'
// Explain
{1,2,3,4,5}, // 59
{5,4,3,2,1}, // 60
{1,2,3,5,4}, // 61
{4,5,3,2,1}, // 62
};

///////////////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the robot and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
				   LPSTR kposzArgs, int nWinMode)
{
	gThisInst=hThisInst; 
    gExp = new MyExperiment("SequenceLearning","sl5","C:/data/SequenceLearning/sl5/data/");
	gExp->redirectIOToConsole(); 
	
	tDisp.init(gThisInst,0,0,550,26,8,2,&(::parseCommand));	
	tDisp.setText("Subj:",0,0);
	
	gScreen.init(gThisInst,1280,0,1280,1024,&(::updateGraphics));
	//gScreen.setScale(Vector2D(0.02,0.02));
	//gScreen.setScale(Vector2D(1.1*0.02,1.1*0.02));	 // In cm 
	//gScreen.setCenter(Vector2D(0,2));	 // In cm 
	gScreen.setCenter(Vector2D(2,3));	 // In cm //0,2
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE)); // cm/pixel 

	// initalize s626cards 
	s626.init("c:/robot/calib/s626_single.txt"); 
	if(s626.getErrorState()==0){
		cout<<"Hello"<<endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); //1	5			// initialize at 200 Hz update rate 
	} 
	
	gTimer.init(1,4,0);					/// < On Cntr_1A , Cntr_1B 
	// initialize stimulation box
	gBox[0].init(BOX_LEFT,"c:/robot/calib/left_highForce_flatBox.txt");
	gBox[1].init(BOX_RIGHT,"c:/robot/calib/right_highForce_flatBox.txt");
	gBox[0].filterconst=0.8; 
	gBox[1].filterconst=0.8; 

	// initalize serial counter 
	gCounter.initSerial("COM1",9600, counterSignal, sliceNumber); //serial 9600 19200
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
MyExperiment::MyExperiment(string name,string code,string dDir): Experiment(name,code,dDir) { 
	theBlock = new MyBlock();
	theTrial = new MyTrial();
	currentTrial = theTrial;
}

////////////////////////////////////////////////////////////////////////
// MyExperiment: control 
////////////////////////////////////////////////////////////////////////
void MyExperiment::control(void){ 
	MSG msg;
	do { 
		if (PeekMessage(&msg,NULL,0,0,PM_REMOVE)){ 
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		} 
		theBlock->control();
		currentTrial->copyHaptics();		// Thread save copy 
		if (gTimer[3]> UPDATE_TEXTDISP) { 
			currentTrial->updateTextDisplay();	
			InvalidateRect(tDisp.windowHnd,NULL,TRUE);
			UpdateWindow(tDisp.windowHnd);
			gTimer.reset(3);
		}; 
		InvalidateRect(gScreen.windowHnd,NULL,TRUE);
		UpdateWindow(gScreen.windowHnd);
	} while (msg.message!=WM_QUIT);
} 


///////////////////////////////////////////////////////////////
// Parse additional commands 
///////////////////////////////////////////////////////////////
bool MyExperiment::parseCommand(string arguments[],int numArgs){ 
	int i,b; 
	float arg[4];
	MSG msg;
	
	/// Recenter 
	
	/// Valves Command: set voltage channels directly 
	if (arguments[0]=="on") { 
		if (numArgs!=5) { 
			tDisp.print("USAGE: on board channel volts seq");
		} else {
			sscanf(arguments[1].c_str(),"%f",&arg[0]);
			b=arg[0];
			sscanf(arguments[2].c_str(),"%f",&arg[1]);
			i=arg[1];
			sscanf(arguments[3].c_str(),"%f",&arg[2]);
			sscanf(arguments[4].c_str(),"%f",&arg[3]);
			
			if (i>=NUMSTIMULATORS) { 
				tDisp.print("USAGE: on channel volts");
			} else { 
				gBox[b].vibrVolts=arg[2];  
				gBox[b].vibrSeq=arg[3];  
				gBox[b].on(i); 
				gBox[b].boardOn=1; 
				gTimer.reset(2);
			} 
		} 
	} 
	/// Valves Command: set voltage channels directly 
	else if (arguments[0]=="off") { 
		for (b=0;b<2;b++) { 
			for (i=0;i<NUMSTIMULATORS;i++) { 
				gBox[b].off(i); 
			} 
			gBox[b].boardOn=0; 
		} 
	} 
	
	/// Print continusly state of the encodeers 
	else if (arguments[0]=="state") { 
		tDisp.keyPressed=0;
		tDisp.lock();
		
		while (!tDisp.keyPressed){ 
			if (PeekMessage(&msg,NULL,0,0,PM_REMOVE)){ 
				TranslateMessage(&msg);
				DispatchMessage(&msg);
				
			} 
			s626.updateAD(0); 
			for (b=0;b<2;b++) { 
				gBox[b].update(); 
				
				sprintf(buffer,"Force : %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[b].getForce(0),
					gBox[b].getForce(1),gBox[b].getForce(2),gBox[b].getForce(3),gBox[b].getForce(4));
			} 
			tDisp.setText(buffer,5,0);
			InvalidateRect(tDisp.windowHnd,NULL,TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10); 
		} 
		tDisp.unlock();
	} 
	
	/// Print continusly state of the encodeers 
	else if (arguments[0]=="zeroF") { 
		tDisp.keyPressed=0;
		tDisp.lock();
		double volts[2][5]={{0,0,0,0,0},{0,0,0,0,0}}; 
		int n,j; 
		for (n=0;n<100;n++) { 				
			for (b=0;b<2;b++) { 
				for (j=0;j<5;j++) { 
					volts[b][j]+=gBox[b].getVolts(j); 
				} 
			} 
			InvalidateRect(tDisp.windowHnd,NULL,TRUE);
			UpdateWindow(tDisp.windowHnd);
			Sleep(10); 
		} 
		cout<<endl;
		for (b=0;b<2;b++) { 
			for (j=0;j<5;j++) { 
				volts[b][j]/=100; 
				cout<<volts[b][j]<< "  "<<endl; 
			} 
			gBox[b].zeroForce(volts[b]); 
		} 
		tDisp.unlock();
	} 
	
	/// Set TR Counter to simulated or non-simulated 
	else if (arguments[0]=="TR") { 
		if (numArgs!=2) { 
			tDisp.print("USAGE: TR delay [ms]");
		} else {
			sscanf(arguments[1].c_str(),"%f",&arg[0]);
			if (arg[0]>=0) { 
				gCounter.simulate(arg[0]);
			} else { 
				gCounter.simulate(0); 
			} 
		} 
	} 
	
	/// Flip display left-right or up-down 
	else if (arguments[0]=="flip") { 
		if (numArgs!=3) { 
			tDisp.print("USAGE: flip horz_sign vert_sign"); 
		} else { 
			sscanf(arguments[1].c_str(),"%f",&arg[0]);
			sscanf(arguments[2].c_str(),"%f",&arg[1]);
			gScreen.setScale(Vector2D(SCR_SCALE*arg[0],SCR_SCALE*arg[1]));
			//gScreen.setScale(Vector2D(0.02*arg[0],0.02*arg[1]));			
			gScreen.setCenter(Vector2D(2*arg[0],3*arg[1]));
			//gScreen.setCenter(Vector2D(0.02*arg[0],0.02*arg[1]));			
		}
	} 
	
	/// Show the force lines 
	else if (arguments[0]=="showlines") { 
		if (numArgs!=2) { 
			tDisp.print("USAGE: showlines 0/1"); 
		} else { 
			sscanf(arguments[1].c_str(),"%f",&arg[0]);
			isPractice=arg[0]; 
		}
	} 
	/// set the movement time threshold
	else if (arguments[0]=="tth") { 
		if (numArgs!=2) { 
			tDisp.print("USAGE: need to super fast and late percent values (tth 0.8 1.2) "); 
		} else { 
			sscanf(arguments[1].c_str(),"%f",&arg[0]);
			sscanf(arguments[2].c_str(),"%f",&arg[1]);
			timeThresholds[1]=arg[0]; 
			timeThresholds[2]=arg[1]; 
		}
	} 
	else if (arguments[0]=="stmt") { 
		if (numArgs!=2) { 
			tDisp.print("USAGE: stmt 0/.../10000 in ms"); 
		} else { 
			sscanf(arguments[1].c_str(),"%f",&arg[0]);
			stMT=arg[0]; 
			for (i=0;i<17;i++){ //loop over all possible sequences and 
				seqMT[0][i]=stMT; 
				seqMT[1][i]=stMT; 
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
void MyExperiment::onExit(){ 
	s626.stopInterrupt();
	tDisp.close();
	gScreen.close();
} 

///////////////////////////////////////////////////////////////
/// Constructor 
///////////////////////////////////////////////////////////////
MyBlock::MyBlock(){
	state=WAIT_BLOCK;
} 

///////////////////////////////////////////////////////////////
/// getTrial
///////////////////////////////////////////////////////////////
Trial* MyBlock::getTrial(){
	return new MyTrial();
} 

///////////////////////////////////////////////////////////////
/// Called at the start of the block: resets TR Counter 
///////////////////////////////////////////////////////////////
void MyBlock::start(){
	for (int i=0; i<NUMDISPLAYLINES; i++){gs.line[i]="";}
	gs.boxOn=true; 
	gCounter.reset();
	gCounter.start();
	gNumErrors=0; 
	gNumPointsBlock=0;
	ghardPress=0;
	glatePress=0;
} 

///////////////////////////////////////////////////////////////
/// giveFeedback and put it to the graphic state 
///////////////////////////////////////////////////////////////
void MyBlock::giveFeedback(){
	gs.boxOn=false; 
	int i, j;
	double n[2]={0, 0}; 
	double nn[2]={0, 0};
	
	
	MyTrial *tpnr;
	avrgMT[0]=0;
	avrgMT[1]=0;
	//reset the sequence time 
	for (i=1; i<17; i++) {
		for (j=0; j<2; j++) {
			seqMT[j][i]= 0;
			seqGood[j][i]=0;
			seqForce[j][i]=0;
		}
	} 
	//make the left right box black
	for (i=0; i<2; i++){gs.boxColor[i]=0;}

	for (i=0;i<trialNum;i++) { 
		tpnr=(MyTrial *)trialVec.at(i);
		if (tpnr ->announce==0 & tpnr ->lastTrial==0){
			if (tpnr->errorFlag==0 & tpnr->incomplete==0) {  
				avrgMT[tpnr->hand]+=tpnr->MT; //remember the RT from the trials and add them
				n[tpnr->hand]++; // remember number of trials
				//sequence movement time
				//cout<<tpnr ->startTR<<tpnr->MT<<endl;

				seqMT[tpnr->hand][tpnr->seqType]+= tpnr->MT; //get the movement times for the sequence
				seqGood[tpnr->hand][tpnr->seqType]++; //remeber how often the seq was produced correct
				seqForce[tpnr->hand][tpnr->seqType]+= tpnr->Force;
			}
			nn[tpnr->hand]++; //count task trials
		} 
	} 
	if (n[0]>0)  
		avrgMT[0]/=(n[0]);

	if (n[1]>0)  
		avrgMT[1]/=(n[1]);

	for (i=0;i<17;i++){ //loop over all possible sequences and 
		for (j=0;j<2;j++){
			if (seqGood[j][i]>0) {
				seqMT[j][i]/=(seqGood[j][i]);
				seqForce[j][i]/=(seqGood[j][i]);
			}
			else {
				seqMT[j][i]=stMT; 
				seqForce[j][i]=0;
			}
		}
	}
	 		

	// print FEEDBACK on the screen 
	for (i=0; i<11; i++){
		gs.line[i]="";
	}		// reset clear the screen
	//cout<< "gNumErrors: " <<gNumErrors<<" count task trials: "<<nn<<endl;
	sprintf(buffer,"error rate: %2.0fpercent",(100.0 /(nn[0]+nn[1]) * gNumErrors));
	gs.line[11]=buffer; 
	sprintf(buffer,"Average movement time: L %2.2fs  R %2.2fs",avrgMT[0]/1000, avrgMT[1]/1000);
	gs.line[12]=buffer; 
	gCounter.stop();
	
	gNumPoints+=gNumPointsBlock;
	sprintf(buffer,"Number points: %d   Total: %d",gNumPointsBlock,gNumPoints);
	gs.line[13]=buffer; 
	
	//ToDo more feedback force MT exclude errors
}

///////////////////////////////////////////////////////////////
///	My Trial class contains the main info of how a trial in this experiment is run 
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Constructor 
///////////////////////////////////////////////////////////////
MyTrial::MyTrial(){
	state=WAIT_TRIAL;
	//INIT TRIAL VARIABLE
			 
	errorFlag=0;						// init error flag
	lateFlag=0;
	hardPress=0;						// init hard press flag
	incomplete=0;						// init if seq was produced incomplete but correct so fare
	seqCounter= 0;						// init the sequence index variable
	inactiveFinger = 0;					// init the inactive Finger counter
	MT=0;								// init total reaction time
	Force=0;
	pointState=0;
	superFast=0;						// init super fast flag
	allPressed=0;						// are all fingers on the board?
	for (int i=0;i<5;i++){					
		releaseState[i] =1;				// init the release state of the fingerpress 
		RT[i]= 0;						//		& reaction time	
		response[i]= 0;					//		& finger response
		pressT[i]=0;					//		& duration of finger press
		hardpressKnown[i]=0;			//		& hardpress knowlege	
	} 
} 

///////////////////////////////////////////////////////////////
// Read
///////////////////////////////////////////////////////////////
void MyTrial::read(istream &in){
	(in)>>startTR>>lastTrial>>startTime>>seqType>>announce>>feedback>>complete>>iti>>sounds>>hand; //
}

///////////////////////////////////////////////////////////////
// Write
///////////////////////////////////////////////////////////////
void MyTrial::writeDat(ostream &out){
	out <<startTR<<"\t"
		<<lastTrial<<"\t"
		<<startTime <<"\t"
		<<seqType <<"\t"
		<<announce<<"\t"
		<<feedback<<"\t"
		<<complete<<"\t"
		<<iti<<"\t"
		<<response[0]<<"\t"
		<<RT[0]<<"\t"
		<<pressT[0]<<"\t"
		<<response[1]<<"\t"
		<<RT[1]<<"\t"
		<<pressT[1]<<"\t"
		<<response[2]<<"\t"
		<<RT[2]<<"\t"
		<<pressT[2]<<"\t"
		<<response[3]<<"\t"
		<<RT[3]<<"\t"
		<<pressT[3]<<"\t"
		<<response[4]<<"\t"
		<<RT[4]<<"\t"
		<<pressT[4]<<"\t"
		<<MT<<"\t"
		<<errorFlag<<"\t"
		<<hardPress<<"\t"
		<<lateFlag<<"\t"
		<<incomplete<<"\t"
		<<pointState<<"\t"
		<<sounds<<"\t"
		<<hand<<endl;
}

///////////////////////////////////////////////////////////////
// Header
///////////////////////////////////////////////////////////////
void MyTrial::writeHeader(ostream &out){
	out <<"startTR"<<"\t"
		<<"lastTrial"<<"\t"
		<<"startTime"<<"\t"
		<<"seqType"<<"\t"
		<<"announce"<<"\t"
		<<"feedback"<<"\t"
		<<"complete"<<"\t"
		<<"iti"<<"\t"
		<<"resp1"<<"\t"
		<<"RT1"<<"\t"
		<<"pressT1"<<"\t"
		<<"resp2"<<"\t"
		<<"RT2"<<"\t"
		<<"pressT2"<<"\t"
		<<"resp3"<<"\t"
		<<"RT3"<<"\t"
		<<"pressT3"<<"\t"
		<<"resp4"<<"\t"
		<<"RT4"<<"\t"
		<<"pressT4"<<"\t"
		<<"resp5"<<"\t"
		<<"RT5"<<"\t"
		<<"pressT5"<<"\t"
		<<"MT"<<"\t"
		<<"seqError"<<"\t"
		<<"hardPress"<<"\t"
		<<"latePress"<<"\t"
		<<"incompletePress"<<"\t"
		<<"trialPoints"<<"\t"
		<<"sounds"<<"\t"
		<<"hand"<<endl;
	
}

///////////////////////////////////////////////////////////////
// Save: Save movement data
///////////////////////////////////////////////////////////////
void MyTrial::writeMov(ostream &out){ 
	dataman.save(out);
} 

///////////////////////////////////////////////////////////////
// Start Trial 
///////////////////////////////////////////////////////////////
void MyTrial::start(){ 
	dataman.clear();
	state=START_TRIAL;
} 

///////////////////////////////////////////////////////////////
// End the trial 
///////////////////////////////////////////////////////////////
void MyTrial::end(){ 
	state=END_TRIAL;
	dataman.stopRecording();
	gs.reset();
} 

///////////////////////////////////////////////////////////////
// isFinished
///////////////////////////////////////////////////////////////
bool MyTrial::isFinished(){ 
	return(state==END_TRIAL?TRUE:FALSE);
} 


///////////////////////////////////////////////////////////////
// copyHaptics: makes a thread safe copy of haptic state 
///////////////////////////////////////////////////////////////
void MyTrial::copyHaptics(){ 
	S626_InterruptEnable( 0, false);
	S626_InterruptEnable( 0, true);
} 

///////////////////////////////////////////////////////////////
/// updateTextDisp: called from TextDisplay 
///////////////////////////////////////////////////////////////
void MyTrial::updateTextDisplay(){ 
	sprintf(buffer,"TR : %d time: %2.2f slice:%d",gCounter.readTR(),gCounter.readTime(),gCounter.readSlice());
	tDisp.setText(buffer,2,0);
	
	//sprintf(buffer,"TIME : %1.4f:", gScreen.lastCycle);
	//tDisp.setText(buffer,4,0);
	sprintf(buffer,"State : %d",state);
	tDisp.setText(buffer,4,0);	
	
	sprintf(buffer,"read : %2.1f   readReal : %2.1f",gTimer[0], gTimer.readReal(1));
	tDisp.setText(buffer,5,0);
	
	//sprintf(buffer,"volts: %d %d %d %d %d",releaseState[0],releaseState[1],releaseState[2],releaseState[3],releaseState[4]); 
	//tDisp.setText(buffer,5,0);
	
	sprintf(buffer,"Force LEFT:    %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[0].getForce(0),gBox[0].getForce(1),gBox[0].getForce(2),gBox[0].getForce(3),gBox[0].getForce(4));
	tDisp.setText(buffer,7,0);
	
	sprintf(buffer,"Force RIGHT:  %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[1].getForce(0),gBox[1].getForce(1),gBox[1].getForce(2),gBox[1].getForce(3),gBox[1].getForce(4));
	tDisp.setText(buffer,8,0);
	
	sprintf(buffer,"trial MT : %2.0f ",MT);
	tDisp.setText(buffer,9,0);
	
	
	sprintf(buffer,"total Points=> %2i      Points=> %2i" ,gNumPoints, gNumPointsBlock);
	tDisp.setText(buffer,11,0);
	
	sprintf(buffer,"Errors => %2i      Hard presses=> %2i      Late presses=> %2i" ,gNumErrors,ghardPress, glatePress);
	tDisp.setText(buffer,12,0); 
	
	sprintf(buffer,"avMT L: %2.2f  R: %2.2f ",avrgMT[0] ,avrgMT[1]);
	tDisp.setText(buffer,14,0);
	
	sprintf(buffer,"seq:    ||1     |2     |3     |4     |5     |6     |7     |8     |9     |10   |11   |12 " );
	tDisp.setText(buffer,15,0);
	sprintf(buffer,"MT L:   ||%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f " ,
		seqMT[0][1]/1000,seqMT[0][2]/1000,seqMT[0][3]/1000,seqMT[0][4]/1000,seqMT[0][5]/1000,seqMT[0][6]/1000,seqMT[0][7]/1000,seqMT[0][8]/1000,seqMT[0][9]/1000,seqMT[0][10]/1000,seqMT[0][11]/1000,seqMT[0][12]/1000);
	tDisp.setText(buffer,16,0);
	sprintf(buffer,"MT R:   ||%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  " ,
		seqMT[1][1]/1000,seqMT[1][2]/1000,seqMT[1][3]/1000,seqMT[1][4]/1000,seqMT[1][5]/1000,seqMT[1][6]/1000,seqMT[1][7]/1000,seqMT[1][8]/1000,seqMT[1][9]/1000,seqMT[1][10]/1000,seqMT[1][11]/1000,seqMT[1][12]/1000);
	tDisp.setText(buffer,17,0);
	sprintf(buffer,"num L: ||%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    " ,
		seqGood[0][1],seqGood[0][2],seqGood[0][3],seqGood[0][4],seqGood[0][5],seqGood[0][6],seqGood[0][7],seqGood[0][8],seqGood[0][9],seqGood[0][10],seqGood[0][11],seqGood[0][12]);
	tDisp.setText(buffer,18,0);
	sprintf(buffer,"num R: ||%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    " ,
		seqGood[1][1],seqGood[1][2],seqGood[1][3],seqGood[1][4],seqGood[1][5],seqGood[1][6],seqGood[1][7],seqGood[1][8],seqGood[1][9],seqGood[1][10],seqGood[1][11],seqGood[1][12]);	
	tDisp.setText(buffer,19,0);
	sprintf(buffer,"F L:      ||%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f " ,
		seqForce[0][1],seqForce[0][2],seqForce[0][3],seqForce[0][4],seqForce[0][5],seqForce[0][6],seqForce[0][7],seqForce[0][8],seqForce[0][9],seqForce[0][10],seqForce[0][11],seqForce[0][12]);
	tDisp.setText(buffer,20,0);
	sprintf(buffer,"F R:      ||%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  " ,
		seqForce[1][1],seqForce[1][2],seqForce[1][3],seqForce[1][4],seqForce[1][5],seqForce[1][6],seqForce[1][7],seqForce[1][8],seqForce[1][9],seqForce[1][10],seqForce[1][11],seqForce[1][12]);
	tDisp.setText(buffer,21,0);
	
	//sprintf(buffer,"dt : %2.1f",gTimer.dt());
	//tDisp.setText(buffer,14,0);
} 

///////////////////////////////////////////////////////////////
/// updateGraphics: Call from ScreenHD 
///////////////////////////////////////////////////////////////
#define FINGWIDTH 1
#define RECWIDTH 1.4
void MyTrial::updateGraphics(int what){ 
	int i,j; 
	
	if (isPractice==1) {
		//ToDo please check!!  
		gScreen.setColor(Screen::white);

		for (i=0;i<5;i++) { //LEFT	
			j=4-i;
			gScreen.drawLine(j*FINGWIDTH-5,(gBox[0].getForce(i)*2-3)/4.9276,j*FINGWIDTH-4.2,(gBox[0].getForce(i)*2-3)/4.9276); 		
		} 
		for (i=0;i<5;i++) { //RIGHT
			gScreen.drawLine(i*FINGWIDTH+1,(gBox[1].getForce(i)*2-3)/4.9276,i*FINGWIDTH+1.8,(gBox[1].getForce(i)*2-3)/4.9276); 		
		} 
	} 
	
	gScreen.setColor(Screen::white);
	for (i=0;i<NUMDISPLAYLINES;i++) { 
		if (!gs.line[i].empty()){ 
			gScreen.setColor(gs.lineColor[i]);
			gScreen.print(gs.line[i].c_str(),gs.lineXpos[i],gs.lineYpos[i],gs.size[i]*1);
		} 
	} 
	
	Vector2D recSize, recPos;
	recSize= Vector2D(1.3,1.3);
	
	if (gs.boxOn){ 
		recSize= Vector2D(1.4*5,1.4);
		recPos= Vector2D(0,2.8);
		gScreen.setColor(Screen::white);
		gScreen.drawRect(recSize, recPos);

		//rect on the sides
			recSize= Vector2D(1.4,1.4);
		recPos= Vector2D(-4.2,2.8);
		gScreen.drawRect(recSize, recPos);
		//RIGHT
		recPos= Vector2D(4.2,2.8);
		gScreen.drawRect(recSize, recPos);
	} 
	// signal which hand to use
	//LEFT
	if (gs.boxColor[0]>0) { 
		recSize= Vector2D(1.35,1.35);
		recPos= Vector2D(-4.2,2.8);
		gScreen.setColor(gs.boxColor[0]);
		gScreen.drawBox(recSize, recPos);
	} 
	//RIGHT
	if (gs.boxColor[1]>0) { 
		recPos= Vector2D(4.2,2.8);
		gScreen.setColor(gs.boxColor[1]);
		gScreen.drawBox(recSize, recPos);
	} 
	
} 

//////////////////////////////////////////////////////////////////////
/// updateHaptics: called from Hardware interrupt to allow for regular update intervals 
//////////////////////////////////////////////////////////////////////
void MyTrial::updateHaptics(){ 
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
	gBox[0].update(); 
	gBox[1].update(); 
	//gBox[1].update(); 
	/// Call the Trial for control 
	currentTrial->control();	
	
	/// record the data at record frequency 
	if (dataman.isRecording() && gTimer[4]>=RECORDRATE) { 
		gTimer.reset(4); 
		bool x=dataman.record(DataRecord(state));
		if (!x){ 
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
	case WAIT_TRIAL: //0
		break;
	case START_TRIAL: //1 
		gTimer.reset(1); 
		gTimer.reset(2);
		for (i=0; i<5;i++){
			response[i]=0;
			RT[i]=0;
		}
		dataman.clear();
		for (i=0; i<11; i++){gs.line[i]="";}			// clear screen
		gs.line[7]=TEXT[0];					// present a star
		gs.lineColor[7]=1;					// WHITE
		state = WAIT_TR;
		break; 
		
	case WAIT_TR: //2				
		/// Wait for TR counter to be at the right place & reset the clocks
		if (gCounter.readTR()==0) { 
			gTimer.reset(0); 
		} 
		if ((gCounter.readTR()>startTR) || (gCounter.readTR()==startTR && gCounter.readTime()>=startTime)) { 		
			dataman.startRecording(); 
			gTimer.reset(1);					//time for whole trial
			gTimer.reset(2);					//time for events in the trial
			gBox[hand].boardOn = 1;
					
			state=WAIT_ANNOUNCE; 
		} 
		
		break; 
		
	case WAIT_ANNOUNCE:  //3
		//Left right signal => color of filled square grey
		if (hand==0){
			gs.boxColor[0]= 3;//5;
			gs.boxColor[1]= 2;//0;
		}
		else if (hand==1){
			gs.boxColor[0]= 2;//0;
			gs.boxColor[1]= 3;//5;
		}
		// anounce the sequence in white or blue. No time delay in that state
		if (announce){							
			for (i=0; i<5; i++) {
				gs.line[i+5]=TEXT[SEQ[seqType][i]]; //TEXT[6-SEQ[seqType][i]];
				gs.lineColor[i+5]=9;			// LIGHTBLUE
			}

		} else if (lastTrial==0){						
			gs.line[7]=TEXT[0];					// present a star
			gs.lineColor[7]=1;					// WHITE 
			sprintf(buffer,"%d", gNumPointsBlock); //show the points 
			gs.line[12]=buffer;
		}
		state =WAIT_ALLFINGERPRESS;

		if (complete>=0) { //we are on a scanning trail!! => present stars and jump directly to WAIT_RESPONSE
		 	if (feedback>=2 && !announce && lastTrial==0){ // show stars on the top if feedback is set to 2
				for (i=0;i<5;i++) {
					gs.line[i]=TEXT[0];				//TEXT[6-SEQ[seqType][i]];
					gs.lineColor[i]=1;				// WHITE 
				}
			}
			state =WAIT_RESPONSE;
		}
		//handle the last TR
		if (lastTrial){ // show only center stars 
			state =WAIT_ITI;
		}
		break; 

	case WAIT_ALLFINGERPRESS: //4
		allPressed=0;
		for (i=0;i<5;i++) {				// check all finger
			if (gBox[0].getForceFilt(i) > 0.005*4.9276) {	// check left fingers
				++allPressed;}
			if (gBox[1].getForceFilt(i) > 0.005*4.9276) {	// check right fingers
				++allPressed;}
		}
		if (allPressed==10) { //if all fingers are pressed give the go signal
			if (feedback==1 && !announce){ // show the finger sequenze on the top if feedback is set to 1
				for (i=0;i<5;i++) {
					gs.line[i]=TEXT[SEQ[seqType][i]]; //TEXT[6-SEQ[seqType][i]];
					gs.lineColor[i]=1;				// WHITE 
				}
			}else if (feedback>=2 && !announce && lastTrial==0){ // show stars on the top if feedback is set to 2
				for (i=0;i<5;i++) {
					gs.line[i]=TEXT[0];				//TEXT[6-SEQ[seqType][i]];
					gs.lineColor[i]=1;				// WHITE 
				}
			}
			state =WAIT_RESPONSE;
		}
		break;
	case WAIT_RESPONSE: //5
		// prepare for reaction & get the reactionTime & responseKey & provide feedback
		// check if the responses were correct 
		// Announce and time is out 
		if (announce && gTimer[2]> complete){		// check if trial is announce trial
			gTimer.reset(2); 
			state =WAIT_FEEDBACK;
		}
		
		/// Real trial: Check response 
		else if (!announce){						// check for finger presses
			inactiveFinger= releaseState[0]+releaseState[1]+releaseState[2]+releaseState[3]+releaseState[4];
			if (seqCounter<5){						// get the reactionTime & responseKey
				
				goalResponse=(SEQ[seqType][seqCounter] - 6+(hand*6)) *(2*hand-1); ///< Required Response in intrinsic coordinates 

				for (i=0;i<5;i++) {					// check all finger
					///< If no finger is pressed, check whether right or wrong is pressed 
				    if (gBox[hand].getForceFilt(i) > THRESHOLD[0][i] &&  inactiveFinger==5){ // check for initial press
						releaseState[i] =0;			// set the finger to unreleased
						response[seqCounter] = i+1; // record finger that was pressed 
						RT[seqCounter] = gTimer[2]; // record the reaction time 
						inactiveFinger=4;
						if (sounds){
							PlaySound(FINGERSOUND[i].c_str(),NULL,SND_ASYNC|SND_FILENAME);//
						}
						
						if (response[seqCounter] != goalResponse) {
							if (feedback) {			// if feedback => Red  set color of the SEQ and center array
								gs.lineColor[seqCounter]=2;
							}
							errorFlag++;			// set error flag
						}else if (response[seqCounter] == goalResponse) {
							if (feedback) {			// if feedback => Green 
								gs.lineColor[seqCounter]=3;
							}
						}
					//< Check if the single pressed finger (the only one with releaseState==0) is released 
					}else if (gBox[hand].getForceFilt(i) <= THRESHOLD[1][i] && !releaseState[i]){ // check for release of the press
						releaseState[i]= 1; 
						pressT[seqCounter]= gTimer[2]-RT[seqCounter];
						seqCounter++;
						for (int i=0;i<5;i++){					
							hardpressKnown[i]=0;			//reset hardpress knowlege for the next fingerpress
						} 
						
					}else if (gBox[hand].getForceFilt(i) >= THRESHOLD[2][i]& hardpressKnown[i]==0){ // force check! was the press to hard?
						if (response[seqCounter] == goalResponse) { 	// check if press was correct
							gs.lineColor[seqCounter]=7;	// yellow press to hard
							hardPress++;
							hardpressKnown[i]=1; //preventing to visit that else if again if the same finger is still pressed hard
						} 
					}
					if (gBox[hand].getForceFilt(i) >= seqFingerForce[seqType][seqCounter]) { // calc. max force for fingers
						seqFingerForce[seqType][seqCounter]= gBox[hand].getForceFilt(i); 
					}
				} //end for
			} // end if 

			if (complete>=0 && gTimer[2]>complete) { // Too late! only executed if complete is set => FOR SCANNING SESSION
				//no MT could be measured COUNT as ERROR!!!!
				if (errorFlag==0) {		//the seq that was typed was ok so fare but incomplete due to TR 
					glatePress++;		//set late flag to lable that 
					incomplete=1;
					gs.lineColor[7]=4;			// blue response was to slow
				}else{	//error occured
					if (gNumPointsBlock>0) { //substract a point if possible
						gNumPointsBlock-=1; 
					} 
					gs.lineColor[7]=2;		// red signal you have been to late to complete the seq. 
					pointState=-1;					
					gNumErrors++;			// the error counter is always updated => error can occure because of wrong presses or time threshold!
				}
				state=WAIT_FEEDBACK; 
				sprintf(buffer,"%d", gNumPointsBlock); //show the points 
				gs.line[12]=buffer;		

				gTimer.reset(2); 
			} // end if 

			if (seqCounter>=5) { // pressed 5 times
				MT= RT[4]+pressT[4]-RT[0]; //calculalte movement time 
				Force= (seqFingerForce[seqType][1]+seqFingerForce[seqType][2]+seqFingerForce[seqType][3]+seqFingerForce[seqType][4]+seqFingerForce[seqType][5])/5;

				if (errorFlag==0){ 
					if (hardPress>0) { 
						gs.lineColor[7]=7;			//  yellow press to hard
						pointState=0;
						ghardPress++;
					} else if (MT< seqMT[hand][seqType]*timeThresholds[0] & feedback==2){  // only during training super fast feedback
						superFast=1;
						gs.lineColor[7]=3;			// three stars  feedback for super fast			
						gs.line[7]="***"; 
						gNumPointsBlock+=3; 
						pointState=3;

					} else if (MT < seqMT[hand][seqType]*timeThresholds[1]) {
						gs.lineColor[7]=3;			// green feedback for correct press
						gNumPointsBlock+=1; 
						pointState=1;
					} else if (MT >= seqMT[hand][seqType]*timeThresholds[1] & feedback!=1) {
						gs.lineColor[7]=4;			// blue response was to slow
						glatePress++;
						lateFlag=1;
						pointState=0;
					} 
				} else {		// error occurred 
					gs.lineColor[7]=2;
					if (gNumPointsBlock>0) { 
						gNumPointsBlock-=1; 
					} 
					pointState=-1;					
					gNumErrors++;
					
				} 
				state=WAIT_FEEDBACK; 
				sprintf(buffer,"%d", gNumPointsBlock); //show the points 
				gs.line[12]=buffer;		
				gTimer.reset(2); 
			} // end if
		
		} // end else if 
		break;
			
	case WAIT_FEEDBACK:  //6
		//do iti 
		if (gTimer[2]>FEEDBACKTIME){
			for (i=0; i<11; i++){gs.line[i]="";}		// reset clear the screen
			for (i=0; i<2; i++){gs.boxColor[i]=0;}
			gTimer.reset(2); 
			state=WAIT_ITI; 
		} 
		break;
		
	case WAIT_ITI:  //7
		if (lastTrial){
			for (i=0; i<11; i++){gs.line[i]="";}		// reset clear the screen
			for (i=0; i<2; i++){gs.boxColor[i]=0;}
		}
		if (gTimer[2]>iti || complete>=0){
			dataman.stopRecording(); 
			gBox[hand].boardOn = 0;				// stop board 
			state=END_TRIAL; 
		} 
		
		break; 
		
	case END_TRIAL: //8
		break;
		}
}


/////////////////////////////////////////////////////////////////////////////////////
/// Data Record: creator records the current data from the device 
/////////////////////////////////////////////////////////////////////////////////////
DataRecord::DataRecord(int s) { 
	int i; 
	state=s;
	TR=gCounter.readTR(); 
	currentSlice= gCounter.readSlice();
	timeReal=gCounter.readTime(); 
	time=gTimer[1]; 
	
	for (i=0;i<5;i++) { 
		force_left[i]=gBox[0].getForce(i);
		force_right[i]=gBox[1].getForce(i);
	} 
} 




/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream &out) { 
	out<<state<<"\t"<<TR<<"\t"<<currentSlice<<"\t"<<timeReal<<"\t"<< time << "\t" 
		<< force_left[0]<< " \t"<<force_left[1]<< "\t" << force_left[2]<< " \t" <<force_left[3]<< "\t" << force_left[4] << " \t" 
		<< force_right[0]<< " \t"<<force_right[1]<< "\t" << force_right[2]<< " \t" <<force_right[3]<< "\t" << force_right[4] << " \t" 
		<<endl;
} 

/////////////////////////////////////////////////////////////////////////////////////
///	Graphic State
/// Collection of current variables relating to what's on the screen 
/// contains 4 lines for display 
/// 
/////////////////////////////////////////////////////////////////////////////////////
GraphicState::GraphicState() { 
	for (int i=0;i<5;i++){		//SEQUENCE LETTER for training
		lineXpos[i]= i*1.4-2.8;
		lineYpos[i]= 4;
		lineColor[i]=1;			// white 
		size[i]=9;
	}
	
	for (i=0;i<5;i++){		//SEQUENCE LETTER for announcement
		lineXpos[i+5]= i*1.4-2.8;
		lineYpos[i+5]= 2.3;
		lineColor[i+5]=1;			// white 
		size[i+5]=9;
	}
	
	lineXpos[10]=0;
	lineYpos[10]=4.5;			// feedback 	
	lineColor[10]=1;				// white 
	size[10]=5;
	lineXpos[11]=0;
	lineYpos[11]=3.5;				// feedback 	
	lineColor[11]=1;				// white 
	size[11]=5;
	
	lineXpos[12]=0;
	lineYpos[12]=0.5;			// block points	
	lineColor[12]=1;				// white 
	size[12]=5;
	lineXpos[13]= 0;
	lineYpos[13]=-0.5;				// total points 	
	lineColor[13]=1;				// white 
	size[13]=5;
	boxOn=false; 
} 


void GraphicState::reset(void) { 
	for (int i=0;i<NUMDISPLAYLINES;i++) { 
		line[i]="";
	} 
} 
