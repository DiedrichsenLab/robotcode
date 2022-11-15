/////////////////////////////////////////////////////////////////////////////
// ManipulandumRed_API2D.h 
//
// written for the 2DoF variant of the 3DoM pantograph robot under a s626 io board 
//
// Derived from Manipulandum.h originally by Joern Diedrichsen, 2009
// j.diedrichsen@ucl.ac.uk
// 
// Modified by Nick Roach and Julius Klein, 2011
// n.roach@imperial.ac.uk, j.klein@imperial.ac.uk
//
/////////////////////////////////////////////////////////////////////////////
#ifndef MANIPULANDUM_H_
#define MANIPULANDUM_H_

#include "Vector2d.h"
#include "Vector3d.h"
#include "Matrix2d.h"
#include "Matrix3d.h"
#include "KalmanFilter.h"
#include "ATI_DAQ.h"  
#include <string> 


using namespace std;

/////////////////////////////////////////////////////////////////////////////
/// \brief Class for control of the threedom robot (located in QS33, lab1) with use of grasp endpoint device (2D)  
///
/// Function init is called with a parameter file that contains the data from 
/// the Calibration. Then you only have to call update() in regular intervals 
/// in the updateHaptics() loop, and you have access to estimates of position and velocity. 
/// Manipulandum uses a KalmanFilter for smoothing. 
/// \author Nick Roach, Joern Diedrichsen, Julius Klein 2011
/// 
/////////////////////////////////////////////////////////////////////////////

class ManipulandumRed2D {
public:
	ManipulandumRed2D(void);						  ///< Constructor 
	~ManipulandumRed2D();						  ///< Destructor 
	void init(string paramfile);			  ///< initialize the manipulandum with parameter file 
	inline void update(){update(dt);}		  ///< Updates the sensory state of the robot 	
	void update(double dt);					  ///< Updates the sensory state of the robot for time dt
	void recenter();						  ///< Put (0,0) of world at current position
	void recenter(const Vector2D &);	      ///< Put (0,0) of global coordinates at the specified position 

	void getCounter(long &sh,long &el,long &wr);	///< Get the raw counter values 
	void getPosition(Vector2D &thePos);				///< Get Positions 
	Vector2D getPosition() {return position;}		///< Get Position 
	void getVelocity(Vector2D &theVel);				///< Get Velocities
	Vector2D getVelocity() {return velocity;}		///< Get Velocity
	void getAngles(Vector3D &theAngles);			///< Get angles of the Shoulder and Elbow
	Vector3D getAngles(){return theta;};			///< Get the Shoulder and Elbow angle in radians

	void setForce(Vector2D theForce);				///< Sets endpoint forces in Euclidean cooridinates #
	void setTorque(Vector3D theTorque);				///< Sets motor torques 
	void setVolts(double v0,double v1,double v2);	///< sets voltages to shoulder and elbow directly 
	
	void getLocalFTForce(Vector3D &theFTForce);    ///< Getforce measured by ft in sensor frame
	Vector3D getLocalFTForce();                    ///< Getforce measured by ft in sensor frame
	void getGlobalFTForce(Vector3D &theFTForce);;  ///< Get force measured by ft in global frame
	Vector3D getGlobalFTForce();				   ///< Get force measured by ft in global frame

	void enableWristControl();		///< Activates PID contol of wrist motor to align ft sensor (Will cause brief drop in force output when called for safety)
 	void disableWristControl();		///< Deactivates PID contol of wrist motor to align ft sensor (Will cause brief drop in force output when called for safety)
	void enableOutput();			///< Master enable for Robot force output (Will cause brief drop in force output when called for safety)
	void disableOutput();			///< Master disable for Robot force output (Will cause brief drop in force output when called for safety)

	void printState(int col=0);		///< prints the state of a robot on the text display 
	void updateOutput();				///< Runs robot servo loop, collects current encoder postions, updates wrist control and motor outputs

private: 

	void checkVoltLimits(void);					///< sets forces in Euclidean cooridinates 
	void checkPosLimits(void);					///< check position is in operating range
	Vector2D ati2World(Vector2D atiforce);      ///< computes the force in world coordinate based on ATI force readings 

	
	//Forward kinematics and Jacobian
	Vector2D forwardKinePos(Vector3D shoulderAngle); ///>calculate robot local end effector position given elbow angles in rads	
	Matrix2D Jacobian(Vector2D shoulderAngles);		 ///>calculates static motor torques for given motor angles and desired forces

	void updateWristControl(double dt); ///< updates wrist angle PID controller 
	void resetWDT();					///< pulses wdt reset line 
	void pollWDT();						///< change wdt line state. Called each servo loop to prevent timeout

	void checkLimitSwitch();		///< check hard limits, disable robot if encountered 
	void checkMotorHardEnable();	///< checks the status of motor hard enable line. ie whether WDT is active, emg stop is off and enable switch down.

public: 
	///robot values that can be read 
	Vector2D position;		///< global position, follwing recentering etc. 
	Vector2D velocity;		///< Current velocity, unfiltered in m/s 
	Vector2D positionFilt;  ///< Position after Kalmanfiltering 
	Vector2D velocityFilt;  ///< Velocity estimate after Kalman filtering 
	Vector2D force;			///< Current Force Goal in global cooridinates  
	Vector2D forceProd;		///< Current estimate of force produced in global coordinates 

	double currentRate;		///< Current servo cycle rate 

	//Position of various parts of the robot in global cord (unaffected by recenter)
	Vector2D motorPos[2];	///< Position of motors  (local coordinates) 
	Vector2D elbowPos[2];	///< Position of elbows (local coordinates) 
	Vector2D elbow_angles[2];	///< Angle of elbows (local coordinates) 

	//Copies of safety gain and torque output variables (effectively read only)
	double safeVoltGainOut;		///< public copy of safety gain ramp on final output voltage 
	Vector3D torqueDemandedOut;	///< public copy of current output torque  
	int outputEnableOut;		///< public copy of Master output enable for all motors (1=enabled, 0= disabled)
	int hardEnableOut;			///< public copy of status of hard enable flag for motors (1=enabled, 0= disabled)
	int hardLimitOut;			///< public copy of status of hard limti switches flag for motors (1=at limit, 0= in range)

	double wristASetAngle; ///< Wrist angle set point 

private: 
	///robot parameters determined from calibration file 
	string paramfilename;	///< name of current parameter file 
	int board_cnt[3];		///< Board for encoder channels 
	int channel[3];			///< Channels for counters 
	int board_da;			///< Board number used for force output 
	int da_offset;			///< Channel number of da offset 
	double ang_ati2arm;		///< Angle from ATI force tranducer to arm coordinates 

	double jointLengthUpper[2];    ///< Length of robot upper joint 
	double jointLengthLower[2];    ///< Length of robot upper joint 
	double robotDistance;		   ///< distance between the two robots 
	double wristLength;			   ///< Length of wrist joint 

	Vector3D offset;		///< Offset for 0 degrees on Shoulder/ Elbow encoder 
	Vector3D dir_cnt;		///< Counting direction of Encoders 
	Vector3D torque2volts;    ///< Torque to volts transversion at motors 

	Vector3D torqueDemanded; ///< Master motor torque demand variable

	///robot values that are updated by update step
	bool isFirstUpdate;			///< is first update? 
	KalmanFilter<2>	filter;		///< Kalman filter for optimal velocity and position estimation 
	KalmanFilter<2>	wristfilter;		///< Kalman filter for optimal velocity and position estimation 
	long rShCnt;				///< Shoulder encoder count 
	long lShCnt;				///< Elbow encoder count 
	long wrCnt;				///< wrist encoder count 
	Vector3D theta;				///< Angles of shoulder joint [0] and elbow joint [1]
	Vector2D rawPosition;	///< raw global position, unfiltered in m 
	Vector3D torque;			///< Current Torques of shoulder [0] and elbow [1] 
	Vector2D lastPosition;		///< Last position 
	Matrix2D JdxdTheta;		///< Jacobian dx/theta 
	Matrix2D curJdTheta;    ///< Temp variable for current Jacobian result

	Vector3D volts;					///< variable for voltage output to motors 
	Vector2D positionOffset;		///< position offset for recentering. 

	Vector3D globalFTForce;		///< FT force in sensor co-ordinate system
	Vector3D localFTForce;      ///< FT force in global robot co-ordinate system

	//safety cutouts
	Vector2D wristLimitAngle;		///< limit of safe wrist motor angle (rad)
	Vector2D rShouldLimitAngle;		///< limit of safe right shoudler angle (rad)
	Vector2D lShouldLimitAngle;		///< limit of safe left shoudler angle (rad)

	double safeVoltRiseTime;				///< Post Cutout voltage gain recovery duration in seconds
	double safeVoltGain;				///< Final gain of motor drive volatages (allow for safe "ramping" of output)

	float shouldVoltLimit;		///< Maximum safe voltage level for shoulder motors
    float wristVoltLimit;		///< Maximum safe voltage level for wrist motors

	int outputEnable; ///< Master output enable for all motors (0 = Disable, 1 = Enable). Controlled by enableOutput/disableOutput rather than directly 

	int wdt_pulse_state; ///< state of watchdog control line
	int first_cycle;


	//wrist angle loop vars
	int wristFBOn; ///<Enable pid for wrist angle control (0=Inactive, 1=Active)
	double wristAPTerm; ///< proportional term for wrist angle control
	double wristAITerm; ///< integral term for wrist angle control
	double wristADTerm; ///< integral term for wrist angle control
	double wristAError; ///< error term for wrist angle control
	double wristAErrLast; ///< integral term for wrist angle control

	//public:
	double wristAKp; ///< Proportional gain for wrist angle control
	double wristAKi; ///< Integral gain for wrist angle control
	double wristAKd; ///< Deriavtive for wrist angle control
	double wristADemandTorque; ///< Intermediate demand torque signal to wrist motor

	double dt;	///< holder for servo sample period
	

}; 

#endif


