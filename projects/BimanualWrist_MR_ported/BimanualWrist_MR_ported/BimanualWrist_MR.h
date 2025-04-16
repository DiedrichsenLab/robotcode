/////////////////////////////////////////////////////////////
/// Bimanual wrist movement for fMRI experiment
/// 
/// Atsushi Yokoi, 2014
/// Ali Ghavampour, 2022
/////////////////////////////////////////////////////////////

#include "Experiment.h"
#include "DataManager.h"
#include "Target.h"
//#include <string.h> 
//#include "Target.h"

#include "TRCounter626.h"
#include "Screen.h"
#include "S626sManager.h"
#include "Timer626.h"
#include <string.h> 
#include <sstream>
#include "WristManipulandum.h" //WristManipulandum


#define N_ROBOTS 			2			///< Number of wrist manipulanda

/////////////////////////////////////////////////////////////
/// These are the states of all the trial. 
/// \sa MyTrial::control()
/////////////////////////////////////////////////////////////
enum TrialState
{
	WAIT_TRIAL,				///< 0 Trial is not started yet, is default mode
	START_TRIAL,			///< 1 Start trial	
	WAIT_TR,				///< 2 Wait for TR pulse to begin the trial	
	WAIT_CENTER_HOLD,		///< 3 Wait for the hand to be start position 	
	SHOW_TARGET,			///< 4 Show target but don't move yet
	GO_CUE,					///< 5 Show go signal
	MOVING,					///< 6 During the movement 
	GIVE_FEEDBACK,			///< 7 Show feedback 
	ACQUIRE_HRF,			///< 8 Non mandatory state, for use in scanner to hold recording
	END_TRIAL,				///< 9 Absorbant State: Trial is finished
};


/////////////////////////////////////////////////////////////
/// \brief Data storage class that holds data for one time sample 
/// for the DataManager
/// 
/// This class stores one frame of the data record that
/// gets safed into *.mov files
/// The recording is done in the end of MyTrial::updateHaptics() 
/// \sa MyTrial::updateHaptics() 
/////////////////////////////////////////////////////////////
class DataRecord {
public:
	DataRecord() {}						///< Empty Constructor 
	DataRecord(int state, int hand, int UB, int PointsCounter, int rt);		///< Constructor that records the data
	void write(ostream& out);			///< Write to Disk 
public:
	int state;							///< State of the Trial 
	double time;						///< Estimated time based of number of interrupts 
	double timeReal;					///< real time from the timer of the s626 board 
	Vector2D position[N_ROBOTS];		///< position of Robot 
	Vector2D cursor[N_ROBOTS];					///< position of Cursor 
	Vector2D velocityFilt;				///< Velocity from the Kalman filter 
	Vector2D encoderAngles[N_ROBOTS];	///< angles of encoders, not hand in world position
	//fMRI synchronising variables
	double	TotTime;
	int		TR;
	int		currentSlice;
	double	TRtime;

	Vector2D dRadius;					///< hand radius
	Vector2D dAngle;					///< hand angle

	int u_or_b;
	int h;

	int points;
	int reactiontime;

};


/////////////////////////////////////////////////////////////
/// \brief This is the core class that determines most of the behavior of the 
/// experiment. 
/// 
/// There class contains  a number of call back routines that 
/// need to be provided. 
/// 
/////////////////////////////////////////////////////////////
class MyTrial :public Trial {
public:
	MyTrial();
	virtual void read(istream& in);					///< Trial input from Target file
	virtual void writeHeader(ostream& out);			///< Write Header of data file
	virtual void writeDat(ostream& out);			///< Write Trial output to data file
	virtual void writeMov(ostream& out);			///< Write movement data (Data Records) to movfile
	virtual void updateGraphics(int i);				///< Graphics routine: called with ~60 Hz
	virtual void updateTextDisplay();				///< Update Text display: called here with 10Hz 
	virtual void updateHaptics();					///< Haptics routine: called with 1000 Hz
	virtual void control();						    ///< Control routine: called at 200 Hz
	virtual void start() { state = START_TRIAL; }	    ///< Start the trial: just kicks it off 
	virtual void end();								///< Response to a request to end trial (q-key press)
	inline void	 penalty();							///< Deem GoodMovement as false, take one point
	inline void  reward();							///< Deem Gm as true, award points
	inline void  passNoPoints();					///< Good Movement, but no points
	inline void  badNoPoints();					///< Good Movement, but no point penalty
	inline void computeOutcome();					///< compute outcome after finishing moving state
	inline bool inTargetArea(int gHand);			///< is the cursor in target area?
	virtual bool isFinished() { return(state == END_TRIAL ? TRUE : FALSE); }		///< Trial ended? Yes if in state ENDTRIAL

private:
	TrialState state;			///< State of the trial
	TrialState errState;		///< State of error, if any

	int r;						///< iterator for N_ROBOTS
	bool overTime;				///< over movement time...

	bool ranOnce;

	int u_or_b;					///< unimanual (0) or bimanual (1)
	int hand;					///< Moving hand 0: left 1:right 
	double RT;					///< Reaction time 
	double MT;					///< Movement time 

	Vector2D endpoint[N_ROBOTS];	///< Endpoint of movement, include in .mov, datarecord
	double endRadius[N_ROBOTS];		///< EndRadius calculated from endpoint
	double endAngle[N_ROBOTS];		///< endAngle 

	double maxRadius[N_ROBOTS];				///< each hand has one maxradius

	bool GoodMovement;			///< movement was good or not

	//experimental conditions (from tgt file)
	int		trialType;					///< Target type (center-out/out-and-back)	
	double	RT_min;						///< 
	double	RT_max;						///< 
	double 	targetDistance;				///< Distance between start box and target
	double	targetAngle[N_ROBOTS];		///< Actual direction of the target 
	double	EndRadius_max;  			///< for feedback, if the endRadius > max_endRadius then failed trial
	int		gHoldTime;					///< time while holding at center before go cue, 1000 ms
	int 	reachTime;					///< time to reach and comeback, 3000
	int 	feedbackTime;
	int		time2plan;
	double	innerTargetTolerance;		///< In pizza slice area, how much tolerance inwards
	double 	outerTargetTolerance;		///< Tolerance outwards (cm)
	double 	targetAngleTolerance;
	double 	centerTolerance;
	double	MaxTrialTime; 				///< absolute limit on the trial length

	//variables for fMRI synchronisation
	double startTRReal;     			///< Ask if this is used
	int startTR;						///< Starting value for TR count
	//int startSlice;						///< Starting value for slice no. 
	//int startSlicereal;					///< Starting value for slice no. 
	double startTime;					///< Time of the start of the trial 
	double startTimeReal;				///< Time of the start of the trial 
	int pointsMyTrial;					///< copy from global variable gPointsBlock

	DataManager<DataRecord, 10000> dataman;	///<  DataManager  // this value 2000 limits how much lines in mov file can be saved
};

/////////////////////////////////////////////////////////////
/// \brief Behavior of a block in your experiment. Most is inherited from Block
/// 
/// Most behaviors of this class are inherited from the abstract class Block
/// A block is a collection of trials. A block is created by typing run 
/// at the Text Display. It then reads in a target file and 
/// creates as many trial objects as lines in the target file. 
/// It then runs these trials and saves the data to disk. 
/// 
////////////////////////////////////////////////////////////////
class MyBlock :public Block
{
public:
	MyBlock();														// Constructor
	virtual ~MyBlock() {};													// Destructor
	virtual Trial* getTrial();
	virtual void start();					///< This is called upon start of the Block
	virtual void giveFeedback();			///< This is called when the Block ends 
};

/////////////////////////////////////////////////////////////
/// \brief An experiment is the root-object of the program. 
/// 
/// An Experiment is the root-object of the program. 
/// Most of the behaviors can be inherited from the 
/// abstract Experiment class. 
///
/////////////////////////////////////////////////////////////
class MyExperiment :public Experiment
{
public:
	MyExperiment(string name, string code, string dDir);
	virtual void control();									///< Main event loop 
	bool parseCommand(string arguments[], int numArgs);
	virtual void onExit(void);								///< Is called on Exit of the Program before destruction
};


//////////////////////////////////////////////////////////////
class FixCross : public Target {
public:
	void draw();
};
void FixCross::draw() {
	//setColor(1);
	gScreen.setColor(color);
	gScreen.drawBox(Vector2D(size[0], 0.3), Vector2D(position[0], position[1]));
	gScreen.drawBox(Vector2D(0.3, size[1]), Vector2D(position[0], position[1]));
}
