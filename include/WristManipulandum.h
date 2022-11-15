///////////////////////////////////////////////////////////////////////////////
/// WristManipulandum Class 
/// fMRI compatible 2-DoF wrist manupulandum
/// under a s626 iocard 
/// 
/// Atsushi Yokoi, 2014
///////////////////////////////////////////////////////////////////////////////

#ifndef WRISTMANUPULANDUM_H
#define WRISTMANIPULANDUM_H 

#define NUMJOINTS 2
#define BOX_LEFT 0 
#define BOX_RIGHT 1 
#include "Vector2d.h"
#include "Matrix2d.h" 
#include "KalmanFilter.h"
#include <stdio.h>
#include <string> 
using namespace std;

enum transformation {LIN, SIN, TAN}; // TODO: choose which transform ang to posit

class WristManipulandum { 
public:
	// initialization
	WristManipulandum();											///< Default based on average g to volts  scale based on calibration of 17/6/2010 JD
	~WristManipulandum(); 											
	bool init(int type);											///< Initialize on the channel, type is either LEFT or RIGHT
	bool init(int type, string filename);							///< Initialize on the channel with calibration file 
	void setScale(double s);										///< Set all to same scale and baseline 
	void setScale(Vector2D mScale);									///< Set individual scale and baseline  
	// update
	inline void update(){update(dt);} 								///< Update state
	void update(double dt);											///< Update state
	// getter
	inline double getVolts(int i) {return volts[i];}							///< 
	inline double getdV(int i){return dV[i];};									///<

	inline double getPosition(int i) {return pos[i];}							///< 
	inline double getPositionFilt(int i) {return posfilt[i];}					///< 
	inline double getVelocity(int i) {return vel[i];}							///< 
	inline double getVelocityFilt(int i) {return velfilt[i];}					///< 
	
	inline double getAngles(int i) {return ang[i];}								///< 
	inline double getAnglesFilt(int i) {return angfilt[i];}						///< 
	inline double getAngVel(int i) {return angvel[i];}							///< 
	inline double getAngVelFilt(int i) {return angvelfilt[i];}							///< 

	Vector2D getPosition(){return Vector2D(pos[0],pos[1]);} 					///< 
	Vector2D getPositionFilt(){return Vector2D(posfilt[0],posfilt[1]);} 		///< 
	Vector2D getVelocity(){return Vector2D(vel[0],vel[1]);} 					///< 
	Vector2D getVelocityFilt(){return Vector2D(velfilt[0],velfilt[1]);} 		///< 
	Vector2D getAngles(){return Vector2D(ang[0],ang[1]);} 						///< 
	Vector2D getAnglesFilt(){return Vector2D(angfilt[0],angfilt[1]);} 			///< 
	
	// setter
	inline void setVolts(double v0,double v1){} 					///< dummy
	inline void recenter(){} 										///< set current position zero in display coordinate
	inline void printState(){}										///< print current state (do nothing for now)
	void setZero(double volts[NUMJOINTS]);							///< set baseline to the last 100 readings 
	// transformation
	void Manip2Disp(double *pm, double *vm, double *pd, double *vd);///< transform from manipulandum coordinate into display coordinate
	// other member variables
	int boardOn;													///< is board active? 	
	void on(int finger);							///< set finger on 
	void off(int finger);							///< set finger of 
	// state variables that can be read outside	
	Vector2D position;		///< Current position, unfiltered in m
	Vector2D velocity;		///< Current velocity, unfiltered in m/s 
	Vector2D positionFilt;  ///< Position after Kalman filtering 
	Vector2D velocityFilt;  ///< Velocity estimate after Kalman filtering 

private: 
	// AD/DA IO
	int board;						///< Board number
	int ADoffset;					///< Channel number for first AD transducers 
	int ADchannel[NUMJOINTS];		///< Channel number for first AD transducers 

	int TBlockPinLeft[NUMJOINTS];	///< Pins for Left Device, Read in Terminal Block
	int TBlockPinRight[NUMJOINTS];	///< For Right Device. USE ONLY WITH DIFFERENT BOX OR PINS

	int DAchannel;					///< channel number for DA transducers 
	int DIOoffset;					///< Channel number for DIO channel 
	double outputVolts;				///< output voltage in pin 42
	double dt;						///< default update rate 
	KalmanFilter<NUMJOINTS>	filterang;	///< Kalman filter for optimal velocity and position estimation 

	// sensor data for internal use
	double baseline[NUMJOINTS];		///< baseline voltage values
	
	double volts[NUMJOINTS]; 		///< raw voltage signal from encoders
	double dV[NUMJOINTS];			///< variation in voltage

	// angular positions, velocities
	double ang[NUMJOINTS]; 			///< joint angle (degree), 0=yaw, 1=pitch
	double ang_old[NUMJOINTS];		///< previous angles, to calculate angular velocity
	double angfilt[NUMJOINTS]; 		///< Filtered joint angle 
	double angvel[NUMJOINTS]; 		///< joint angular velocity (degree/sec)
	double angvelfilt[NUMJOINTS]; 	///< Filtered joint angular velocity
	// linear positions, velocities
	double pos[NUMJOINTS]; 			///< position on display (m), 0=x, 1=y
	double posfilt[NUMJOINTS]; 		///< Filtered position on display
	double vel[NUMJOINTS];			///< velocity on display (m/sec)
	double velfilt[NUMJOINTS];		///< Filtered velocity on display
	
	// calibration parameters
	Vector2D scale;			 		///< scaling factors
	double length; 					///< length of (virtual) fencing grip (m)
	double vpd_h, vpd_v;			///< volts per degree horizontal and vertical
	
	// init stuff
	string paramfilename;			///< Weird redudant string for input parameters, but is in use
	
	// filter parameters
	double initialAng[NUMJOINTS];///< start position for Kalman filter
	double acelAngNoiseVar;			///< term for aceleration noise variance
	double angNoiseVar;			///< term for position noise variance

	transformation tTransf;
	bool ang_or_angfilt;
	int transf_int_type;
	inline void ang2pos(transformation tTrans, double angIn[], double posOut[]);
	
	// buffers
	double bufferVolts[20];
	
	inline double calcvelocity(double posit[], int size);
//	inline transformation int2transf(int transf_int_type);
}; 

#endif 