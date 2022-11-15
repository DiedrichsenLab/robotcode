/////////////////////////////////////////////////////////////////////////////
// ManipulandumDR.h 
// 
// written for a 2-link robot under a s626 io board 
// and electric motors with torque control 
// Joern Diedrichsen, 2009
// j.diedrichsen@ucl.ac.uk
// 
/////////////////////////////////////////////////////////////////////////////
#ifndef MANIPULANDUM_H_
#define MANIPULANDUM_H_

#include "Vector2d.h"
#include "Matrix2d.h" 
#include "KalmanFilter.h"
#include <string> 
using namespace std;

/////////////////////////////////////////////////////////////////////////////
/// \brief Class for control of the 2-link robot under a s626 io board 
///
/// Function init is called with a parameter file that contains the data from 
/// the Calibration. Then you only have to call update() in regular intervals 
/// in the updateHaptics() loop, and you have access to estimates of position and velocity. 
/// Manipulandum uses a KalmanFilter for smoothing. 
/// \author Joern Diedrichsen
/// 
/////////////////////////////////////////////////////////////////////////////
class Manipulandum {
public:
	Manipulandum(void);						  ///< Constructor 
	~Manipulandum();						  ///< Destructor 
	void init(string paramfile);			  ///< initialize the manipulandum with parameter file 
	inline void update(){update(dt);}		  ///< Updates the sensory state of the robot 	
	void update(double dt);					  ///< Updates the sensory state of the robot for time dt
	void recenter();						  ///< Put (0,0) of world at current position
	
	void getCounter(long &sh,long &el);		  ///< Get the raw counter values 
	void getPosition(Vector2D &thePos);	      ///< Get Positions 
	Vector2D getPosition() {return position;} ///< Get Position 
	void getVelocity(Vector2D &theVel);		  ///< Get Velocities
	Vector2D getVelocity() {return velocity;} ///< Get Velocity
	void getAngles(Vector2D &theAngles);	  ///< Get angles of the Shoulder and Elbow
	Vector2D getAngles(){return theta;};	  ///< Get the Shoulder and Elbow angle in radians
	
	void setForce(Vector2D theForce);		  ///< sets forces in Euclidean cooridinates 
	void setVolts(double v0,double v1);	      ///< sets voltages to shoulder and elbow directly 
	Vector2D ati2World(Vector2D atiforce);    ///< computes the force in world coordinate based on ATI force readings 
	double getSafetySwitch() {return switchState;} ///< Get the state of the safety  // change to bool later 

	void printState(int col=0);					      ///< prints the state of a robot on the text display 
	void setFromLocal(const Matrix2D &,const Vector2D &);	///< Sets the Transformation between local (Robot) coordinates and the World
	

public: 
	///robot values that can be read 
	Vector2D position;		///< Current position, unfiltered in m
	Vector2D velocity;		///< Current velocity, unfiltered in m/s 
	Vector2D positionFilt;  ///< Position after Kalmanfiltering 
	Vector2D velocityFilt;  ///< Velocity estimate after Kalman filtering 
	Vector2D force;			///< Current Force Goal in global cooridinates  
	Vector2D forceProd;		///< Current estimate of force produced in global coordinates 
	double switchState;		///< State of the safety switch (change to bool later) 
	double dt;				///< default update rate 
public: 
	///robot parameters determined from calibration file 
	string paramfilename;	///< name of current parameter file 
	int board_cnt[2];		///< Board for encoder channels 
	int channel[2];			///< Channels for counters 
	int board_da;			///< Board number used for force output 
	int da_offset;			///< Channel number of da offset 
	double ang_ati2arm;		///< Angle from ATI force tranducer to arm coordinates 
	Matrix2D AFromLocal;	///< rotation Robot->World  x_world = A * x_robot + v
	Vector2D vFromLocal;	///< offset Robot->World  x_world = A * x_robot + v
	Matrix2D AToLocal;		///< inverse (AFromLocal)
	Vector2D vToLocal;		///< vFromLocal
	Vector2D jointLength;   ///< Length of robot joints 
	Vector2D offset;		///< Offset for 0 degrees on Shoulder/ Elbow encoder 
	Vector2D dir_cnt;		///< Counting direction of Encoders 
	Vector2D torque2volts;  ///< Torque to volts transversion at motors 
	int safetySwitch;		///< AD channel for safety readout 
	
	///robot values that are updated by update step
	bool isFirstUpdate;			///< is first update? 
	KalmanFilter<2>	filter;		///< Kalman filter for optimal velocity and position estimation 
	long sh_cnt;				///< Shoulder encoder count 
	long el_cnt;				///< Elbow encoder count 
	Vector2D theta;				///< Angles of shoulder joint [0] and elbow joint [1]
	Vector2D torque;			///< Current Torques of shoulder [0] and elbow [1] 
	Vector2D positionLocal;		///< position in robot space 
	Vector2D lastPosition;		///< Last position 
	Matrix2D Jdx_dtheta;		///< Jacobian dx/theta 
	Vector2D volts;

	bool errorState;
}; 

#endif


