/////////////////////////////////////////////////////////////////////////////
/// Manipulandum class 
/// 
/// 
/// written for a 2-link robot under a s626 io board 
/// Joern Diedrichsen, 2007 
/// j.diedrichsen@bangor.ac.uk
/// 
/////////////////////////////////////////////////////////////////////////////
#ifndef MANIPULANDUMMR_H_
#define MANIPULANDUMMR_H_

#include "Vector2d.h"
#include "Matrix2d.h" 
#include "KalmanFilter.h"
#include <string> 
using namespace std;

/////////////////////////////////////////////////////////////////////////////
/// \brief Class for control of the 2-link fMRI compatible robot  
///
/// Function init is called with a parameter file that contains the data from 
/// the Calibration. Then you only have to call update() in regular intervals 
/// in the updateHaptics() loop, and you have access to estimates of position and velocity. 
/// Manipulandum uses a KalmanFilter for smoothing. 
/// \author Joern Diedrichsen, 2009-2011
/// 
/////////////////////////////////////////////////////////////////////////////
class ManipulandumMR {
public:
	ManipulandumMR(void);						  ///< Constructor 
	~ManipulandumMR();						  ///< Destructor 
	void init(string paramfile);			  ///< initialize the manipulandum with parameter file 
	inline void update(){update(dt);}		  ///< Updates the sensory state of the robot 	
	void update(double dt);					  ///< Updates the sensory state of the robot for time dt
	void recenter();						  ///< Put (0,0) at current position 
	void getCounter(long &sh,long &el);		  ///< Get the raw counter values 
	void getPosition(Vector2D &thePos);	      ///< Get Positions 
	Vector2D getPosition() {return position;} ///< Get Position 
	void getVelocity(Vector2D &theVel);		  ///< Get Velocities
	Vector2D getVelocity() {return velocity;} ///< Get Velocity
	
	void getAngles(Vector2D &theAngles);	  ///< Get angles of the Shoulder and Elbow
	Vector2D getAngles(){return theta;};	  ///< Get the Shoulder and Elbow angle in radians
	
	void setForce(Vector2D theForce);		  ///< sets forces in Eucledian cooridinates 
	void setValves(double v0,double v1,double v2,double v3); ///< sets voltages of valves directly 
	void printState();					      ///< prints the state of a robot on the text display 
	void setFromLocal(const Matrix2D &,const Vector2D &);	///< Sets the Transformation between local (Robot) coordinates and the World

public: 
	///robot values that can be read 
	Vector2D position;		///< Current position, unfiltered in m
	Vector2D velocity;		///< Current velocity, unfiltered in m/s 
	Vector2D positionFilt;  ///< Position after Kalmanfiltering 
	Vector2D velocityFilt;  ///< Velocity estimate after Kalman filtering 
	Vector2D force;			///< Current Force Goal in global cooridinates  
	Vector2D forceProd;		///< Current estimate of force produced in global coordinates 
	Vector2D forcePiston;   ///< Current Force Goal at each of the pistons 
	Vector2D forcePistonProd;///< Current estimate of force produced at the pistons (assuming a 60ms time constant)
	double offsetPiston;	///< Offset of valves to avoid starting hysterisis
	double dt;				///< default update rate 
	double overdrive;
public: 
	///robot parameters determined from calibration file 
	string paramfilename;	///< name of current parameter file 
	int board_cnt[2];		///< Board for encoder channels 
	int channel[2];			///< Channels for counters 
	int board_da;			///< Board number used for force output 
	Matrix2D AFromLocal;	///< rotation Robot->World  x_world = A * x_robot + v
	Vector2D vFromLocal;	///< offset Robot->World  x_world = A * x_robot + v
	Matrix2D AToLocal;		///< inverse (AFromLocal)
	Vector2D vToLocal;		///< vFromLocal
	Vector2D jointLength;    ///< Length of robot joints 
	Vector2D linkLength;	///< Length of links between axle and pistons 
	Vector2D offset;		///< Offset for 0 degrees on Shoulder/ Elbow encoder 
	Vector2D dir_cnt;		///< Counting direction of Encoders 
	Vector2D alpha;			///< Angle between axle and positon base 
	Vector2D diagLength;	///< Length of diagonal between axle and piston base 	
	double force2volts;		///< Force to volts transversion at pistons 
	double tauPiston;		///< Time constant of force production at the pistons 
	
	///robot values that are updated by update step
	bool isFirstUpdate;			///< is first update? 
	KalmanFilter<2>	filter;		///< Kalman filter for optimal velocity and position estimation 
	long sh_cnt;				///< Shoulder encoder count 
	long el_cnt;				///< Elbow encoder count 
	Vector2D theta;				///< Angles of shoulder joint [0] and elbow joint [1]
	Vector2D phi;				///< angles relative to attachment of pistons 
	Vector2D positionLocal;		///< position in robot space 
	Vector2D lastPosition;		///< Last position 
	Vector2D pistonLength;		///< Length of the pistons 
	Matrix2D Jdx_dtheta;		///< Jacobian dx/theta 
	Matrix2D Jdtheta_dl;		///< jacobian dtheta to dl
	double volts[4];

	bool errorState;
}; 

#endif


