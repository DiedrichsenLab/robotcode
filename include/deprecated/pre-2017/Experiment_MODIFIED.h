/// Universal class for the implementation of an Experiment 
/// 
/// Relies on a Text display and a Timer 
/// 
/// Joern Diedrichsen 2006
/// j.diedrichsen@ucl.ac.uk
//////////////////////////////////////////////////////////////////////////

#if !defined DEF_EXPERIMENT
#define DEF_EXPERIMENT

#include <math.h>
#include <assert.h>
#include <string> 
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;
#include "TextDisplay.h"

// State of the block 
enum BlockState  { 
	WAIT_BLOCK,				// Block not started yet 
	ANNOUNCE,				// Announicing the block
	START_BLOCK,			// Kick off the Block 
	RUNNING,				// Is Running the trials 
	SAVING,
	FEEDBACK,
	END_BLOCK				// Finished the Block 
}; 


double median (double array[],int num_val);  ///< Calculates the median of an array of values 

///////////////////////////////////////////////////////////
/// \brief Abstract Class that specifies the basic functions of a trial
/// 
/// To specify the behavior of your experiment, make a class MyTrial that
/// Inherits from the class Trial and specify the pure virtual functions. 
/// 
///////////////////////////////////////////////////////////
class Trial {
public:
	Trial();											///< Constructor 
	virtual ~Trial();									///< Destructor 
	virtual void read(istream &in)=0;					///< Trial input from Target file 
	virtual void writeHeader(ostream &out)=0;			///< Write the header of a data file
	virtual void writeDat(ostream &out)=0;				///< Trial output to data file 
	virtual void writeMov(ostream &out)=0;				///< Trial output to mov file 
	virtual void updateGraphics(int eye) =0;			///< Update Graphics window (called ~60hz) 
	virtual void updateHaptics() = 0;					///< Update Haptics (called with 1000 hz) 
	virtual void control()=0;							///< main implementation of trial
	virtual void start()=0;								///< Start the Trial
	virtual void end()=0;								///< response to a request to end trial 
	virtual bool isFinished() = 0;						///< Trial Finished ? 
	virtual bool isValid(){return true;}			    ///< Has trial been valid?
	virtual Trial *copy() {return this;}			    ///< get a copy of the trial
	virtual void updateTextDisplay(){}			///< Update the text display (call from Text display)
	virtual void copyHaptics(){}				///< Copy haptic state for thread safe copying 
	virtual void record(){}						///< recording of data frame 
};


///////////////////////////////////////////////////////////
/// \brief Abstract block class with behavior for 
/// a collection of trials. 
/// 
/// A block is created by typing run 
/// at the Text Display. It then reads in a target file and 
/// creates as many trial objects as lines in the target file. 
/// It then runs these trials and saves the data to disk. 
/// 
///////////////////////////////////////////////////////////
class Block {
public: 
	Block();											///< Constructor
	~Block();											///< Destructor
	virtual bool init(int blocknum,string filename);	///< initialize the Trials
	virtual void control();								///< control loop for block
	virtual void announce(){}							///< Calls the announcing routine for a block 
	virtual void start() {}								///< starts the block
	virtual void giveFeedback(){}						///< function for Block feedback
	virtual void end(){}								///< Called after feedback: Clean feedback + End the block
	virtual Trial* getTrial()=0;						///< create a new Trial 
	virtual void save();								///< save mov and dat file 
public:
	int blockNumber;									///< Block number 
	int trialNum;										///< Number of the current Trial
	int numTrials;										///< Total number of trials 
	vector<Trial *> trialVec;							///< Vector of trials 
	BlockState state;									///< state of the block 

	string datFilename;									///< Data file name 
	string movFilename;									///< Mov file name 
	string targetFilename;								///< Name of current Target file 
	ofstream datFile;									///< Data file 
	ofstream movFile;									///< Mov file 
	ifstream targetFile;								///< Target file for input
	bool isSaved;										///< is Block saved?
};

///////////////////////////////////////////////////////////
/// C-style Callback functions 
///////////////////////////////////////////////////////////
void parseCommand(string line);					///< Parse command -> covers basics, then calls Experiment::parseCommand
void updateHaptics(void);						///< s626 update (calls currentTrial-> updateHaptics)
void updateGraphics(int i);					    ///< calls currentTrial->updateGraphics 
void keypress(unsigned char key, int x, int y);	///< callback for screen 
void onExit();									///< is called on Exit (calls gExp->onExit) 


///////////////////////////////////////////////////////////
/// \brief Abstract Root class that specifies a lot of the  
/// default behaviors of an experiment
/// 
/// Experiment: Abstract class that implements basic experiment needs  
/// Your experiment can inherit most of these behaviors   
///
///////////////////////////////////////////////////////////
class Experiment{
public:
	Experiment(string name, string code);				///< Constructor 
	virtual ~Experiment();								///< Destructor 
	virtual void control(void)=0;						///< handle Windows events 
	void redirectIOToConsole();							///< Create IO console 
	virtual bool parseCommand(string arguments[],int numArgs){return false;}  ///< Extra function for new textDisplay commands 
	virtual void onExit(void){}							///< On exit callback 
	string getCode(){return code;}						///< returns experiment code 
public: 
	friend class Block;						
	string subjectName;						///< Name of current Subject s
	Block *theBlock;						///< Pointer to current block object
	Trial *theTrial;						///< Pointer to current block trial
protected:
	string code;							///< Code 
	string title;							///< Name 
};

#endif 
