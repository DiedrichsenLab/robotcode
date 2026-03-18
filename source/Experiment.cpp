 ///////////////////////////////////////////////////////////////////
/// 
/// Experiment.cpp
/// Experiment, Block and Trial class implementation 
/// 
/// Joern Diedrichsen 2006
/// j.diedrichsen@ucl.ac.uk
/////////////////////////////////////////////////////////////////////

#include "Experiment.h"
#include <fcntl.h>
#include <io.h>
#include <windows.h>

//*****************************************************************
///	Globals (most external - define in myExperiment.cpp **
//******************************************************************

extern HINSTANCE gThisInst;			// Instance of Windows application 
extern TextDisplay tDisp; 


char myBuffer[100];

extern Experiment* gExp;
extern Trial* currentTrial;
extern char gKey;
extern bool gKeyPressed;

//*******************************************************************************
/// C-style callbacks 
//*******************************************************************************/
////////////////////////////////////////////////////////////////////////
/// parseCommands check the input from the text display for known commands 
/// subj: specify subject name 
/// run: runs a block of trials
/// quit/exit: exits the program 
/// afterwards calls MyExperiment->parseCommands() for optional commands.
////////////////////////////////////////////////////////////////////////
void parseCommand(string line) { 
	
	/// Parse the input into command and parameters 
	string arguments[10];
	string::size_type pos; 
	int numArgs=0;
	int bn;
	bool isDone=FALSE;
	bool correctCommand; 
	while (numArgs<10 && !isDone && !line.empty()){ 
		pos = line.find(" ");
		if (pos==string::npos) { 
			arguments[numArgs++]=line;
			isDone=TRUE;
		} else if (pos==0) { 
			line=line.substr(1,line.length()-1);
		} else {
			arguments[numArgs++]=line.substr(0,pos);
			line=line.substr(pos,line.length()-pos+1);
		} 
	} 
	

	/// SUBJECT command: Changes subject info
    if (arguments[0]=="subj" || arguments[0]=="SUBJ") { 
		if (numArgs!=2) { 
			tDisp.print("USAGE: subj code");
		} else {
			tDisp.setText("Subj: "+arguments[1],0,0);
			gExp->subjectName=arguments[1];
		}
	} 
	
	/// RUN Command: Runs a block  
	else if (arguments[0]=="run" || arguments[0]=="RUN") { 
		if (numArgs!=3) { 
			tDisp.print("USAGE: run blocknumber targetfile");
		} else {
			sscanf(arguments[1].c_str(),"%i",&bn);
			if (gExp->theBlock->init(bn,arguments[2])) { 
				gExp->theBlock->state=START_BLOCK; 
			} 
		}
	} 

	/// Quit Command: ends the program  
	else if (arguments[0]=="quit" || arguments[0]=="exit") { 
		exit(0);
	} else if (arguments[0].length()>0) { 
		correctCommand=gExp->parseCommand(arguments,numArgs);
		if (!correctCommand){ 
			tDisp.print("Unknown command"); 
		} 
	} 
} 


// -----------------------------------------------
//  This handler is called when the application is exiting.  Deallocates any state 
//  and cleans up.
// --------------------------------------------------------------------------------
void onExit()
{
	gExp->onExit();

	//cout<<"Press Key when ready"<<endl;
	//getchar();
	//cout<<"Press Key when ready"<<endl;
	//getchar();

	delete gExp;
}


// -----------------------------------------------
// keypress: Exit on Q 
// --------------------------------------------------------------------------------
void keypress(unsigned char key, int x, int y)
{
   // any keyboard press will cause the application to exit
	switch (key){
	case 'q': 
		exit(0);
	default:
		gKey=key;
		gKeyPressed=1;
	}
}

////////////////////////////////////////////////////////////////////////
/// The Call back updateHaptics just calls the 
///  currentTrial->updateHaptics through Polymorphism
////////////////////////////////////////////////////////////////////////
void updateHaptics(void){
	// get time and update Manipulandum 
	currentTrial->updateHaptics();
} 

////////////////////////////////////////////////////////////////////////
/// The Call back update_graphics just calls the 
///  currentTrial->updateGraphics through Polymorphism
////////////////////////////////////////////////////////////////////////
void updateGraphics(int value){
	currentTrial->updateGraphics(value);
} 

////////////////////////////////////////////////////////////////////////
///             Experiment			                 **
/// Contains implemention for experiment 
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////
Experiment::Experiment(string tit,string c,string dDir){
	title=tit;
	code=c;
	gKeyPressed=0;
	theBlock=NULL;
	theTrial=NULL;
	dataDir=dDir;  ///< Data directory path 	
}

////////////////////////////////////////////////////////////////////////
// Backwards-compatible constructor for older projects which don't 
// specify a data directory
////////////////////////////////////////////////////////////////////////
Experiment::Experiment(string tit,string c){
	title=tit;
	code=c;
	gKeyPressed=0;
	theBlock=NULL;
	theTrial=NULL;	
	dataDir="data/";
}

////////////////////////////////////////////////////////////////////////
/// The destructor deletes the Block and all the trials in the experiment  
/// 
////////////////////////////////////////////////////////////////////////
Experiment::~Experiment(){
	if (!theBlock==NULL){
		delete theBlock;
	} 
	if (!theTrial==NULL){
		delete theTrial;
	} 
}

static const WORD MAX_CONSOLE_LINES = 500;	
/////////////////////////////////////////////////////////
/// Makes the IO console for cout<< and cerr<< output  
/// 
/////////////////////////////////////////////////////////
void Experiment::redirectIOToConsole()
{
	int hConHandle;
	long lStdHandle;
	CONSOLE_SCREEN_BUFFER_INFO coninfo;
	FILE *fp;
	// allocate a console for this app
	AllocConsole();
	
	// ============================================================ Ali
	freopen("CONIN$", "r", stdin);
	freopen("CONOUT$", "w", stdout);
	freopen("CONOUT$", "w", stderr);
	std::cout << "Hello\n"; //you should see this on the console
	// ============================================================
	

	// set the screen buffer to be big enough to let us scroll text
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE),&coninfo);
	coninfo.dwSize.Y = MAX_CONSOLE_LINES;
	SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE),coninfo.dwSize);
	

	// redirect unbuffered STDOUT to the console
	lStdHandle = (long)GetStdHandle(STD_OUTPUT_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	fp = _fdopen( hConHandle, "w" );
	*stdout = *fp;
	setvbuf( stdout, NULL, _IONBF, 0 );
	
	// redirect unbuffered STDIN to the console
	lStdHandle = (long)GetStdHandle(STD_INPUT_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	fp = _fdopen( hConHandle, "r" );
	*stdin = *fp;
	setvbuf( stdin, NULL, _IONBF, 0 );
	
	// redirect unbuffered STDERR to the console
	lStdHandle = (long)GetStdHandle(STD_ERROR_HANDLE);
	hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
	fp = _fdopen( hConHandle, "w" );
	*stderr = *fp;
	setvbuf( stderr, NULL, _IONBF, 0 );
	
	// make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog	
	// point to console as well
	ios::sync_with_stdio();	
}



/******************************************************
**                        Block                      **
******************************************************/
/////////////////////////////////////////////////////////
/// The Constructor only sets the number of Trials to zero 
/// 
/////////////////////////////////////////////////////////
Block::Block(){
	trialNum=0;
	blockNumber=0;
}

/////////////////////////////////////////////////////////
/// Destructor frees the memory allocated by the block, 
/// including the recorded data 
/////////////////////////////////////////////////////////
Block::~Block(){
	// Clean up from last Block 
	Trial* trialPtr;
	for (int i=0;i<trialVec.size();i++){ 
		trialPtr=trialVec.at(i);
		if (!trialPtr==NULL) { 
			delete trialPtr;			
			//trialVec.assign(i,NULL);
		} 
	} 
	trialVec.clear();
	vector<Trial *>().swap(trialVec); // delete vector by swap method
	cout<<"~Block() called!"<<endl;
}

/////////////////////////////////////////////////////////
/// Initializes the Block by reading from a target file 
/// init makes the .mov and .dat file for later data saving. 
/// Calls Trial::read to specify how a trial is read
/// \param blockn Block number 
/// \param filename Name of the target file to be read in
/// \return boolean whether successful or not  
/////////////////////////////////////////////////////////
bool Block::init(int blockn,string filename){
	char buffer[50];
	blockNumber=blockn;
	Trial *trialPtr=NULL;
	string code; 
	ifstream testFile;
	bool isMovFile;

	// Clean up from last Block (not necessary as long as we clear it in destructor?)
	if (!trialVec.empty()){
		for (int i=0;i<trialVec.size();i++){ 
			trialPtr=trialVec.at(i);
			if (!trialPtr==NULL) { 
				delete trialPtr;
				cout<<"trialVec.at("<<i<<") = "<<trialPtr<<" deleted!"<<endl;
				trialPtr=NULL;
				//trialVec.assign(i,NULL);
			} 
		} 
		trialVec.clear();
		vector<Trial *>().swap(trialVec); // delete vector
		cout<<"freed trialVec!"<<endl;
	}

	// Generate file names and check that the trial has not been run 
	code=gExp->getCode() + "_" + gExp->subjectName; 
	sprintf(buffer,"%2.2d",blockNumber);
	datFilename=gExp->dataDir + code + ".dat"; 
	movFilename=gExp->dataDir + code + "_" + buffer + ".mov";
	
	datFile.open(datFilename.c_str(),ios::out | ios::app); 
	if (!datFile){ 
		tDisp.print(datFilename + " could not be opened");
		return FALSE; 
	} else { 
		datFile.close();
	} 



	// Test if mov file alreay exists...
	// 
	testFile.open(movFilename.c_str(),ios::in);
	if (!testFile){ 
		isMovFile=FALSE;
	} else {
		isMovFile=TRUE;
		testFile.close();
	} 


	// Load the new target File and allocate 
	targetFilename="target/" + filename;
	targetFile.clear();
	targetFile.open(targetFilename.c_str(),ios::in);
	if (!targetFile){ 
		tDisp.print(filename + " Not Found");
		return FALSE; 
	} 

	if (isMovFile) { 
		if (MessageBox(tDisp.windowHnd, "Block exist. Mov-file will be overwritten!", "Error",
			MB_ICONEXCLAMATION | MB_OKCANCEL)==2) { 
			targetFile.clear();
			targetFile.close();
			return FALSE; 
		} 
	} 

	targetFile.ignore(200,'\n'); // ignore the header line 
	while (!targetFile.eof() && !targetFile.fail() && !targetFile.bad())	{ 
	
		trialPtr=this->getTrial();
		trialPtr->read(targetFile); 
		if (!targetFile.eof() && !targetFile.fail() && !targetFile.bad()){
			trialVec.push_back(trialPtr);
		} else { 
			numTrials=trialVec.size();
			sprintf(buffer,"%d trials read",numTrials);
			tDisp.print(buffer);
			delete(trialPtr);
		} 
		
	} 
	trialNum=0;
	targetFile.clear();
	targetFile.close();
	return TRUE;
}


/////////////////////////////////////////////////////////
/// Saves the data collected in a Block. Gives error message 
/// if files could not be opened for writing. 
/// The routine calls trial->writeDat() to write a line in the dat file, 
/// and trial->writeMov() to write a trial into the mov file  
/// \sa Trial::writeDat() Trial::writeMov() Trial::writeHeader()
/////////////////////////////////////////////////////////
void Block::save(){
	int n;
	datFile.open(datFilename.c_str(),ios::out | ios::app); 
	if (!datFile){ 
		tDisp.print(datFilename + "could not be opened");
		return; 
	} 
	movFile.open(movFilename.c_str(),ios::out); 
	if (!movFile){ 
		tDisp.print(movFilename + "could not be opened");
		return; 
	} 
	
	datFile.setf(ios::fixed,ios::floatfield);
	movFile.setf(ios::fixed,ios::floatfield);
	datFile.precision(3);
	movFile.precision(3);
	if (blockNumber==1) { 
		datFile<<"BN\tTN\t";
		trialVec.at(0)->writeHeader(datFile);
	}
	
	for (n=0;n<trialNum;n++) {
		datFile<<blockNumber<<"\t"<<n+1<<"\t";
		trialVec.at(n)->writeDat(datFile);
	} 
	for (n=0;n<trialNum;n++) {
		movFile<<"Trial "<<n+1<<endl;
		trialVec.at(n)->writeMov(movFile);
	} 
	datFile.close(); 
	movFile.close();
}


/////////////////////////////////////////////////////////
/// Control loop for the Block. When started it calls start(). 
/// Then it sets the state of the 
/// first trial to START_TRIAL and waits until it is in the state END_TRIAL.
/// Then it automatically advances to the next trial.
/// After the block it calls give_feedback and save. 
/// \sa start() give_feedback() save() end() 
/////////////////////////////////////////////////////////
void Block::control(){ 
	
	/// check if user interrupted current block 
	if (TextDisplay::keyPressed && state == RUNNING) { 
		if (TextDisplay::key=='q' || TextDisplay::key=='Q') { 
			state = SAVING;
			currentTrial->end();
			TextDisplay::keyPressed=0;

		}
	}

	switch (state) { 
	
	/// 
	case WAIT_BLOCK:				
		break; 
	case ANNOUNCE:				
		this->announce();		// Call the announcing routine 
		break; 
	case START_BLOCK:				
		tDisp.lock();			// Lock the tDisplay
		state=RUNNING;
		trialNum=0;
		this->start();					// call Block start routine 
		sprintf(myBuffer,"BN: %d  TN: %d",blockNumber,trialNum+1);
		tDisp.setText(myBuffer,5,0);
		currentTrial=trialVec.at(trialNum);
		currentTrial->start();

		break;
	case RUNNING:				// Block is currently going 
		if (currentTrial->isFinished()) {
			trialNum++;
			if (trialNum<numTrials){ 
				currentTrial=trialVec.at(trialNum);
				currentTrial->start();
				sprintf(myBuffer,"BN: %d  TN: %d",blockNumber,trialNum+1);
				tDisp.setText(myBuffer,5,0);
			} else {
				state=SAVING;
			} 
		} 
		break;
	case SAVING:
		
		this->giveFeedback();			// Call give feedback routine 
		this->save();
		
		//TextDisplay::keyPressed=0;
		state=FEEDBACK;
		break; 
	case FEEDBACK: 
		if (TextDisplay::keyPressed && TextDisplay::key==13) { 
			state = END_BLOCK;
			this->end();				// End the Block 
			TextDisplay::keyPressed=0;
			tDisp.unlock();
		}
		break;
	case END_BLOCK:				// Finished the Block
		currentTrial=gExp->theTrial;	// make the waiting trial the current one 
		state=WAIT_BLOCK;
		break;
	} 
} 


/******************************************************
**             Trial				                 **
******************************************************/

/////////////////////////////////////////////////////////
/// empty constructor
/// 
/////////////////////////////////////////////////////////
Trial::Trial(){
};

/////////////////////////////////////////////////////////
/// empty destructor
/// 
/////////////////////////////////////////////////////////
Trial::~Trial(){
};

/////////////////////////////////////////////////////////
/// median: Helper function most used for Block::giveFeedback 
/// calculates the median of an array of values 
/////////////////////////////////////////////////////////
double median(double array[],int num_val) { 
	int i,j; 
	double dummy; 
	for (i=0;i<num_val-1;i++) { 
		for (j=i+1;j<num_val;j++) { 
			if (array[i]>array[j]) { 
				dummy=array[i]; 
				array[i]=array[j];
				array[j]=dummy; 
			} 
		} 
	} 
	if (num_val%2==0) { 
		i=num_val/2; 
		return((array[i-1]+array[i])/2); 
	} else { 
		i=(num_val-1)/2; 
		return(array[i]); 
	} 
} 

/////////////////////////////////////////////////////////
/// quartiles: Helper function most used for Block::giveFeedback 
/// calculates q1,q2,q3 of an array of values 
/////////////////////////////////////////////////////////
void quartiles(double array[], int num_val, double& q1, double& q2, double& q3) {
	int i, j;
	double dummy;

	// Sort the array (using selection sort as per original implementation)
	for (i = 0; i < num_val - 1; i++) {
		for (j = i + 1; j < num_val; j++) {
			if (array[i] > array[j]) {
				dummy = array[i];
				array[i] = array[j];
				array[j] = dummy;
			}
		}
	}

	int i1 = num_val / 4;
	int i2 = num_val / 2;
	int i3 = num_val / 4 * 3;

	q1 = array[i1];
	q2 = array[i2];
	q3 = array[i3];
}