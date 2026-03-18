///////////////////////////////////////////////////////////////
/// 
///	
///	learning of finger sequences
/// 
/// 
/// Tobias Wiestler, 2010 
/// 
///////////////////////////////////////////////////////////////

#include "SequenceLearning3.h" 
#include "StimulatorBox.h" 


///////////////////////////////////////////////////////////////
/// Global variables 
///////////////////////////////////////////////////////////////
S626sManager s626;				///< Hardware Manager 
TextDisplay tDisp;				///< Text Display
Screen gScreen;					///< Screen 
TRCounter gCounter;				///< TR Counter 
StimulatorBox gBox[2];			///< Stimulator Box

//StimulatorBox gBox[2];			///< Stimulator Box 
Timer gTimer(UPDATERATE);		///< Timer from S626 board experiments 
HapticState hs;					///< This is the haptic State as d by the interrupt 
///< For Thread safety this SHOULD NOT be assessed While 
///< the interrupt is running. Use Thread-safe copy to 
///< Get the current haptic state for graphical display 
GraphicState gs; 

char buffer[300];						///< String buffer 
HINSTANCE gThisInst;					///< Instance of Windows application 
Experiment *gExp;						///< Pointer to myExperiment 
Trial *currentTrial;					///< Pointer to current Trial 
bool gKeyPressed;						///< Key pressed? 
char gKey;								///< Which key?
int isPractice=0;						///< Should we show the lines to help practice
int gNumErrors=0;						///< How many erros did you make during a block

double timeThresholds[2]= {0.8, 1.2};	///< percentage when super fast and when late trial


int gNumPointsBlock= 0;
int gNumPoints= 0;
int ghardPress=0;	
int glatePress=0;
double avrgMT=0;
double stMT=3600;		
double seqMT[19]= {stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT,stMT,stMT,stMT,  stMT,stMT,stMT};
double seqGood[19]= {0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0,0,0};
double seqForce[19]= {0,0,0,0,0  ,0,0,0,0,0 ,0,0,0,0,0 ,0,0,0,0};
double seqFingerForce[][5]={{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

#define TRTIME 2660//2700
char counterSignal= '5';		///< ToDo: AVOID THAT What char is used to count the TR
int sliceNumber= 32;			///< How many slices do we have

#define FEEDBACKTIME 800 
string TEXT[11]={"*","1","2","3","4","5","6", "7", "8", "9", "+"}; 
string FINGERSOUND[6]= {"A.wav", "C.wav", "D.wav", "E.wav", "G.wav"};
int STIM_INTENSITY[5]= {5, 5, 5 ,5, 5};
#define resTH 0.5
#define relTH 0.45
#define maxTH  5// //1.8
double THRESHOLD[3][5]= {{resTH, resTH, resTH, resTH, resTH}, {relTH, relTH, relTH, relTH, relTH}, {maxTH, maxTH, maxTH, maxTH, maxTH}};
//int SEQ[10][5]= {{0,0,0,0,0},{1,5,2,4,3},{3,4,1,5,2},{1,3,5,4,2},{5,4,1,3,2},{3,2,4,5,1},{4,2,3,5,1}, {3,5,2,1,4}, {2,1,5,3,4}, {4,5,3,1,2}};
int SEQ[][5]={{0,0,0,0,0},
{5,3,4,2,1}, //1
{5,2,1,3,4}, //2
{4,5,1,3,2}, //3
{4,1,3,5,2}, //4
{3,1,4,2,5}, //5
{2,3,5,4,1}, //6
{2,5,3,1,4}, //7
{1,4,2,5,3}, //8
{1,2,4,3,5}, //9
{1,5,4,2,3}, //10
{3,5,2,1,4}, //11
{3,2,5,1,4}, //12
{4,2,5,3,1},       //these are the explain sequences!!!!
{2,5,4,1,3},
{2,1,3,5,4}, //15  //these are seq for testing the passing for the scan with unkown seq
{4,5,3,1,2}, //16
{3,5,4,2,1}, //17
{1,3,5,4,2}	 //18
};

///////////////////////////////////////////////////////////////
/// Main Program: Start the experiment, initialize the robot and run it 
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
				   LPSTR kposzArgs, int nWinMode)
{
	gThisInst=hThisInst; 
    gExp = new MyExperiment("SequenceLearning","SL1");
	gExp->redirectIOToConsole(); 
	
	tDisp.init(gThisInst,0,0,550,26,8,2,&(::parseCommand));	
	tDisp.setText("Subj:",0,0);
	
	gScreen.init(gThisInst,1280,0,1280,1024,&(::updateGraphics));
	gScreen.setScale(Vector2D(0.02,0.02));
	//gScreen.setScale(Vector2D(1.1*0.02,1.1*0.02));	 // In cm 
	gScreen.setCenter(Vector2D(0,2));	 // In cm 
	//gScreen.setScale(Vector2D(-SCR_SCALE, SCR_SCALE)); // cm/pixel 

	// initalize s626cards 
	s626.init("c:/robot/calib/s626_single.txt"); 
	if(s626.getErrorState()==0){
		cout<<"Hello"<<endl;
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE); //1	5			// initialize at 200 Hz update rate 
	} 
	
	gTimer.init(1,4,0);					/// < On Cntr_1A , Cntr_1B 
	// initialize stimulation box
	gBox[0].init(BOX_LEFT);
	gBox[1].init(BOX_RIGHT);
	gBox[0].setScale(1);
	gBox[1].setScale(1);
	
	
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
MyExperiment::MyExperiment(string name,string code): Experiment(name,code) { 
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
				
				sprintf(buffer,"Volts : %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[b].getForce(0),
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
			gScreen.setScale(Vector2D(0.02*arg[0],0.02*arg[1]));			
			gScreen.setCenter(Vector2D(0.02*arg[0],0.02*arg[1]));			
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
			for (i=0;i<19;i++){ //loop over all possible sequences and 
				seqMT[i]=stMT; 
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
	int i;
	double n=0; 
	double nn=0;
	
	
	MyTrial *tpnr;
	avrgMT=0;
	//reset the sequence time 
	for (i=1; i<19; i++) {
		seqMT[i]= 0;
		seqGood[i]=0;
		seqForce[i]=0;
	} 
	
	for (i=0;i<trialNum;i++) { 
		tpnr=(MyTrial *)trialVec.at(i);
		if (tpnr ->announce==0 & tpnr ->lastTrial==0){
			if (tpnr->errorFlag==0 & tpnr->incomplete==0) {  
				avrgMT+=tpnr->MT; //remember the RT from the trials and add them
				n++; // remember number of trials
				//sequence movement time
				//cout<<tpnr ->startTR<<tpnr->MT<<endl;

				seqMT[tpnr->seqType]+= tpnr->MT; //get the mobvement times for the sequence
				seqGood[tpnr->seqType]++; //remeber how often the seq was produced correct
				seqForce[tpnr->seqType]+= tpnr->Force;
			}
			nn++; //count task trials
		} 
	} 
	if (n>0)  
		avrgMT/=(n);

	for (i=0;i<19;i++){ //loop over all possible sequences and 
		if (seqGood[i]>0) {
			seqMT[i]/=(seqGood[i]);
			seqForce[i]/=(seqGood[i]);
		}
		else {
			seqMT[i]=stMT; 
			seqForce[i]=0;
		}
	}
	 		

	// print FEEDBACK on the screen 
	for (i=0; i<11; i++){
		gs.line[i]="";
	}		// reset clear the screen
	//cout<< "gNumErrors: " <<gNumErrors<<" count task trials: "<<nn<<endl;
	sprintf(buffer,"error rate: %2.0fpercent",(100.0 /nn * gNumErrors));
	gs.line[16]=buffer; 
	sprintf(buffer,"Average movement time: %2.2fs",avrgMT/1000);
	gs.line[17]=buffer; 
	gCounter.stop();
	
	gNumPoints+=gNumPointsBlock;
	sprintf(buffer,"Number points: %d   Total: %d",gNumPointsBlock,gNumPoints);
	gs.line[18]=buffer; 
	
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
	announceCounter= 0;					// init the sequence index variable gor the anouncement of the sequence	
	inactiveFinger = 0;					// init the inactive Finger counter
	MT=0;								// init total reaction time
	Force=0;
	pointState=0;
	superFast=0;						// init super fast flag
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
	(in)>>startTR>>lastTrial>>startTime>>seqType>>announce>>feedback>>complete>>iti>>trialType>>sounds>>board>>day; //
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
		<<trialType<<"\t"
		<<sounds<<"\t"
		<<board<<"\t"
		<<day<<endl;
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
		<<"trialType"<<"\t"
		<<"sounds"<<"\t"
		<<"board"<<"\t"
		<<"day"<<endl;
	
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
	
	sprintf(buffer,"volts LEFT:    %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[0].getForce(0),gBox[0].getForce(1),gBox[0].getForce(2),gBox[0].getForce(3),gBox[0].getForce(4));
	tDisp.setText(buffer,7,0);
	
	sprintf(buffer,"volts RIGHT:  %2.2f %2.2f %2.2f %2.2f %2.2f",gBox[1].getForce(0),gBox[1].getForce(1),gBox[1].getForce(2),gBox[1].getForce(3),gBox[1].getForce(4));
	tDisp.setText(buffer,8,0);
	
	sprintf(buffer,"trial MT : %2.0f ",MT);
	tDisp.setText(buffer,9,0);
	
	
	sprintf(buffer,"total Points=> %2i      Points=> %2i" ,gNumPoints, gNumPointsBlock);
	tDisp.setText(buffer,11,0);
	
	sprintf(buffer,"Errors => %2i      Hard presses=> %2i      Late presses=> %2i" ,gNumErrors,ghardPress, glatePress);
	tDisp.setText(buffer,12,0); 
	
	sprintf(buffer,"avMT: %2.2f ",avrgMT);
	tDisp.setText(buffer,14,0);
	
	sprintf(buffer,"seq:  ||1     |2     |3     |4     |5     |6     |7     |8     |9     |10   |11   |12   |13   |14   |15 " );
	tDisp.setText(buffer,15,0);
	sprintf(buffer,"MT:   ||%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f " ,
		seqMT[1]/1000,seqMT[2]/1000,seqMT[3]/1000,seqMT[4]/1000,seqMT[5]/1000,seqMT[6]/1000,seqMT[7]/1000,seqMT[8]/1000,seqMT[9]/1000,seqMT[10]/1000,seqMT[11]/1000,seqMT[12]/1000,seqMT[13]/1000,seqMT[14]/1000,seqMT[15]/1000);
	tDisp.setText(buffer,16,0);
	sprintf(buffer,"num: ||%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f    |%2.0f " ,seqGood[1],seqGood[2],seqGood[3],seqGood[4],seqGood[5],seqGood[6],seqGood[7],seqGood[8],seqGood[9],seqGood[10],seqGood[11],seqGood[12],seqGood[13],seqGood[14],seqGood[15]);
	tDisp.setText(buffer,18,0);
	sprintf(buffer,"F:      ||%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f  |%2.1f " ,
		seqForce[1],seqForce[2],seqForce[3],seqForce[4],seqForce[5],seqForce[6],seqForce[7],seqForce[8],seqForce[9],seqForce[10],seqForce[11],seqForce[12],seqForce[13],seqForce[14],seqForce[15]);
	tDisp.setText(buffer,19,0);
	
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
		gScreen.setColor(Screen::white);
		for (i=0;i<5;i++) { 
			j=4-i;
			gScreen.drawLine(j*FINGWIDTH-2.5,gBox[0].getForce(i)*2-3,j*FINGWIDTH-1.7,gBox[0].getForce(i)*2-3); 		
		} 
	} 
	
	gScreen.setColor(Screen::white);
	for (i=0;i<NUMDISPLAYLINES;i++) { 
		if (!gs.line[i].empty()){ 
			gScreen.setColor(gs.lineColor[i]);
			gScreen.print(gs.line[i].c_str(),gs.lineXpos[i],gs.lineYpos[i],gs.size[i]*100);
			//cout<<gs.size[i]<<endl;
		} 
	} 
	
	Vector2D recSize, recPos;
	recSize= Vector2D(1.3,1.3);
	
	if (gs.boxOn){ 
		recSize= Vector2D(1.4*5,1.4);
		recPos= Vector2D(0,2.8);
		gScreen.setColor(Screen::white);
		gScreen.drawRect(recSize, recPos);
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
	
	/// record the data at update frequency 
	if (dataman.isRecording()) { 
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
//ToDo
//check which hand is perfoming
	int i;
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
		for (i=0; i<18; i++){gs.line[i]="";}			// clear screen
		gs.line[12]=TEXT[0];					// present a star
		gs.lineColor[12]=1;					// WHITE
		state = WAIT_TR;
		break; 
		
	case WAIT_TR: //2				
		/// Wait for TR counter to be at the right place & reset the clocks
		if (gCounter.readTR()==0) { 
			gTimer.reset(0); 
			//			gs.line[7]=TEXT[0];					// present a star
			//			gs.lineColor[7]=1;					// WHITE
		} 
		if ((gCounter.readTR()>startTR) || (gCounter.readTR()==startTR && gCounter.readTime()>=startTime)) { 		
			dataman.startRecording(); 
			//cout<<gCounter.readTR()-startTR<< " " << gCounter.readTime()-startTime<<endl;
			gTimer.reset(1);					//time for whole trial
			gTimer.reset(2);					//time for events in the trial
			gBox[board].boardOn = 1;
					
			state=WAIT_ANNOUNCE; 
		} 
		
		break; 
		
	case WAIT_ANNOUNCE:  //3
		// anounce the sequence in white or blue. No time delay in that state
		if (announce){	//present white stars on top
			for (i=0; i<10; i++){	
				gs.line[i]=TEXT[0];						
				gs.lineColor[i]=1;
			}	
		} else if (lastTrial==0){						
			gs.line[12]=TEXT[0];					// present a star
			gs.lineColor[12]=1;					// WHITE 
			sprintf(buffer,"%d", gNumPointsBlock); //show the points 
			gs.line[17]=buffer;
		}
		
		if (feedback==1 && !announce){ // show the here the first key that has to be pressed if feedback is set to 1
			//present white stars
			for (i=0; i<15; i++){	
				gs.line[i]=TEXT[0];						
				gs.lineColor[i]=1;
			}	
			//color the first key that has to be pressed
			gs.lineColor[5*board+SEQ[seqType][0]-1]=9; //LIGHTBLUE

		}else if (feedback>=2 && !announce && lastTrial==0){ // show stars on the top if feedback is set to 2
			for (i=0; i<15; i++){	
				gs.line[i]=TEXT[0];						
				gs.lineColor[i]=1;
			}	
		}
		//jump to WAIT_RESPONSE if it is not the last trial
		state =WAIT_RESPONSE;
		//handle the last TR
		if (lastTrial){ // show only center stars 
			state =WAIT_ITI;
		}
		break; 
		
	case WAIT_RESPONSE: //4
		// prepare for reaction & get the reactionTime & responseKey & provide feedback
		// check if the responses were correct 
		// Announce and time is out 
		if (announce && (gTimer[2]> (complete/5)*announceCounter) && announceCounter<5){		// 
			// first make all announcement stars white again!
			for (i=0; i<10; i++){gs.lineColor[i]=1;}			
			// present the positions of the sequence => if board==0 present sequence left => if board==1 present sequence right				
			gs.lineColor[5*board+SEQ[seqType][announceCounter]-1]=9; //LIGHTBLUE
			announceCounter=announceCounter++; //count up the announcement counter
	
		}else if (announce && gTimer[2]> complete){		// check if trial is announce trial & if anouncement time is up
			gTimer.reset(2); 
			state =WAIT_FEEDBACK;
			
			// real trial: wait for response 
		}else if (!announce){						// check for finger presses
			inactiveFinger= releaseState[0]+releaseState[1]+releaseState[2]+releaseState[3]+releaseState[4];
			if (seqCounter<5){						// get the reactionTime & responseKey
				for (i=0;i<5;i++) {					// check all finger
				    if (gBox[board].getForce(i) > THRESHOLD[0][i] &&  inactiveFinger==5){ // check for initial press
						releaseState[i] =0;			// set the finger to unreleased
						response[seqCounter] = i+1; // record finger that was pressed 
						RT[seqCounter] = gTimer[2]; // record the reaction time 
						inactiveFinger=4;
						if (sounds){
							PlaySound(FINGERSOUND[i].c_str(),NULL,SND_ASYNC|SND_FILENAME);//
						}
						if (response[seqCounter] != SEQ[seqType][seqCounter]){
							if (feedback) {			// if feedback => Red  set color of the SEQ and center array
								gs.lineColor[seqCounter+10]=2;
							}
							errorFlag++;			// set error flag
							if (feedback==1){ //color the next key that has to be pressed
								gs.lineColor[5*board+SEQ[seqType][seqCounter]-1]=1; //WHITE previous key
								gs.lineColor[5*board+SEQ[seqType][seqCounter+1]-1]=9; //LIGHTBLUE
							}
						}else if (response[seqCounter] == SEQ[seqType][seqCounter]){
							if (feedback) {			// if feedback => Green 
								gs.lineColor[seqCounter+10]=3;
							}
							if (feedback==1){ //color the next key that has to be pressed
								gs.lineColor[5*board+SEQ[seqType][seqCounter]-1]=1; //WHITE previous key
								gs.lineColor[5*board+SEQ[seqType][seqCounter+1]-1]=9; //LIGHTBLUE
							}
						}
					}else if (gBox[board].getForce(i) <= THRESHOLD[1][i] && !releaseState[i]){ // check for release of the press
						releaseState[i]= 1; 
						pressT[seqCounter]= gTimer[2]-RT[seqCounter];
						seqCounter++;
						for (int i=0;i<5;i++){					
							hardpressKnown[i]=0;			//reset hardpress knowlege for the next fingerpress
						} 
						
					}else if (gBox[board].getForce(i) >= THRESHOLD[2][i]& hardpressKnown[i]==0){ // force check! was the press to hard?
						if (response[seqCounter] == SEQ[seqType][seqCounter]) { 	// check if press was correct
							gs.lineColor[seqCounter+10]=7;	// yellow press to hard
							hardPress++;
							hardpressKnown[i]=1; //preventing to visit that else if again if the same finger is still pressed hard
						} 
					}
					if (gBox[board].getForce(i) >= seqFingerForce[seqType][seqCounter]) { // calc. max force for fingers
						seqFingerForce[seqType][seqCounter]= gBox[board].getForce(i); 
					}
				} //end for
			} // end if 

			if (gTimer[2]>complete && complete>=0) { // Too late! only executed if complete is set => FOR SCANNING SESSION
	//ToDo maybe delete the stars around the center star
				//no MT could be measured COUNT as ERROR!!!!
				if (errorFlag==0) {		//the seq that was typed was ok so fare but incomplete due to TR 
					glatePress++;		//set late flag to lable that 
					incomplete=1;
					gs.lineColor[12]=4;			// blue response was to slow
				}else{	//error occured
					if (gNumPointsBlock>0) { //substract a point if possible
						gNumPointsBlock-=1; 
					} 
					gs.lineColor[12]=2;		// red signal you have been to late to complete the seq. 
					pointState=-1;					
					gNumErrors++;			// the error counter is always updated => error can occure because of wrong presses or time threshold!
				}
				state=WAIT_FEEDBACK; 
				sprintf(buffer,"%d", gNumPointsBlock); //show the points 
				gs.line[17]=buffer;		

				gTimer.reset(2); 
			} // end if 

			if (seqCounter>=5) { // pressed 5 times
	//ToDo maybe delete the stars around the center star
				MT= RT[4]+pressT[4]-RT[0]; //calculalte movement time 
				Force= (seqFingerForce[seqType][1]+seqFingerForce[seqType][2]+seqFingerForce[seqType][3]+seqFingerForce[seqType][4]+seqFingerForce[seqType][5])/5;

				if (errorFlag==0){ 
					if (hardPress>0) { 
						gs.lineColor[12]=7;			//  yellow press to hard
						pointState=0;
						ghardPress++;
					} else if (MT< seqMT[seqType]*timeThresholds[0] & feedback==2){  // only during training super fast feedback
						superFast=1;
						gs.lineColor[12]=3;			// three stars  feedback for super fast			
						gs.line[12]="***"; 
						gNumPointsBlock+=3; 
						pointState=3;

					} else if (MT < seqMT[seqType]*timeThresholds[1]) {
						gs.lineColor[12]=3;			// green feedback for correct press
						gNumPointsBlock+=1; 
						pointState=1;
					} else if (MT >= seqMT[seqType]*timeThresholds[1]) {
						gs.lineColor[12]=4;			// blue response was to slow
						glatePress++;
						lateFlag=1;
						pointState=0;
					} 
				} else {		// error occurred 
					gs.lineColor[12]=2;
					if (gNumPointsBlock>0) { 
						gNumPointsBlock-=1; 
					} 
					pointState=-1;					
					gNumErrors++;
					
				} 
				state=WAIT_FEEDBACK; 
				sprintf(buffer,"%d", gNumPointsBlock); //show the points 
				gs.line[17]=buffer;		
				gTimer.reset(2); 
			} // end if
		
		} // end else if 
		break;
			
	case WAIT_FEEDBACK:  //5
		//do iti 
		if (gTimer[2]>FEEDBACKTIME){
			for (i=0; i<16; i++){gs.line[i]="";}		// reset clear the screen
			gTimer.reset(2); 
			state=WAIT_ITI; 
		} 
		break;
		
	case WAIT_ITI:  //6
		if (gTimer[2]>iti || complete>=0){
			dataman.stopRecording(); 
			gBox[board].boardOn = 0;				// stop board 
			state=END_TRIAL; 
		} 
		
		break; 
		
	case END_TRIAL: //7
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
		volts_left[i]=gBox[0].getForce(i);
		volts_right[i]=gBox[1].getForce(i);
	} 
} 




/////////////////////////////////////////////////////////////////////////////////////
// Writes out the data to the *.mov file 
/////////////////////////////////////////////////////////////////////////////////////
void DataRecord::write(ostream &out) { 
	out<<state<<"\t"<<TR<<"\t"<<currentSlice<<"\t"<<timeReal<<"\t"<< time << "\t" 
		<< volts_left[0]<< " \t"<<volts_left[1]<< "\t" << volts_left[2]<< " \t" <<volts_left[3]<< "\t" << volts_left[4] << " \t" 
		<< volts_right[0]<< " \t"<<volts_right[1]<< "\t" << volts_right[2]<< " \t" <<volts_right[3]<< "\t" << volts_right[4] << " \t" 
		<<endl;
} 

/////////////////////////////////////////////////////////////////////////////////////
///	Graphic State
/// Collection of current variables relating to what's on the screen 
/// contains 4 lines for display 
/// 
/////////////////////////////////////////////////////////////////////////////////////
GraphicState::GraphicState() { 
	for (int i=0;i<5;i++){		//LEFT keys positions
		lineXpos[i]= i*1.4-7.6;
		lineYpos[i]= 4;
		lineColor[i]=1;			// white 
		size[i]=9;
	}
	//define LEFT and RIGHT in two steps to be able to have more space between left and right
	for (i=0;i<5;i++){		//RIGHT keys position
		lineXpos[i+5]= i*1.4+2;
		lineYpos[i+5]= 4;
		lineColor[i+5]=1;			// white 
		size[i+5]=9;
	}

	for (i=0;i<5;i++){		//SEQUENCE LETTER for announcement
		lineXpos[i+10]= i*1.4-2.8;
		lineYpos[i+10]= 2.3;
		lineColor[i+10]=1;			// white 
		size[i+10]=9;
	}
	
	lineXpos[15]=0;
	lineYpos[15]=4.5;			// feedback 	
	lineColor[15]=1;				// white 
	size[15]=5;
	lineXpos[16]=0;
	lineYpos[16]=3.5;				// feedback 	
	lineColor[16]=1;				// white 
	size[16]=5;
	
	lineXpos[17]=0;
	lineYpos[17]=0.5;			// block points	
	lineColor[17]=1;				// white 
	size[17]=5;
	lineXpos[18]= 0;
	lineYpos[18]=-0.5;				// total points 	
	lineColor[18]=1;				// white 
	size[18]=5;
	boxOn=false; 
} 


void GraphicState::reset(void) { 
	for (int i=0;i<NUMDISPLAYLINES;i++) { 
		line[i]="";
	} 
} 
