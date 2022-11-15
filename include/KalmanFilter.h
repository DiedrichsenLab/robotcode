////////////////////////////////////////////////////
// KalmaFilter.h n filter for efficient and optimal velocity information
// based on noisy position data 
// Flexible class for n-dimensional position data
// All channels are treated as being independent and equal in variance and dt
// Kalman filter assumes that acceleration is a Gaussian White noise process (velocity is brownian motion) 
//   2007, Joern Diedrichsen 
//   j.diedrichsen@bangor.ac.uk
////////////////////////////////////////////////////

#ifndef KALMANFILTER 
#define KALMANFILTER 

#include "Vector2d.h"
#include "Matrix2d.h"

template <int N>

////////////////////////////////////////////////////
/// \brief  Kalman filter for efficient and optimal velocity estimation based on (noisy) position data with possibly noisy timing
/// 
/// Template class for  n-dimensional position data.
/// All channels are treated as being independent and equal in variance and dt.
/// Kalman filter assumes that acceleration is a Gaussian White noise process (velocity is brownian motion) .
/// We thank Daniel Wolpert for pointing us to this method.
/// \author Joern Diedrichsen 
////////////////////////////////////////////////////
class KalmanFilter{ 
public: 
	KalmanFilter();						      
	~KalmanFilter(){} 
	void init(double *y0,double w=30, double v=0.0005, double dt=0.001); ///< initialize fresh 
	void update(double *y);						///< Update Kalman filter with new measurement and standard dt 
	void update(double *y,double dt);			///< Update Kalman filter with new measurement and dt 
	void velocity(double *v);					///< Return filtered velocity 
	void position(double *p);					///< Return filtered position 
public: 
	Vector2D x[N];								///< Current state on nth channel 
	Vector2D xm;								///< current state after time update 
	Vector2D K;									///< Current Kalman gain 
	Matrix2D P;									///< Current uncertainty matrix for all channels 
	Matrix2D Pm;								///< Current uncertainty matrix after time update 
	Matrix2D A;									///< Transition matrix 
	Matrix2D Q;									///< Conditional uncertainty matrix 
	double w2;									///< Variance of state transitions 
	double v2;									///< Measurement variance 
	double dt,dt2,dt3;							///< delta t and powers of 
}; 

////////////////////////////////////////////////////
/// N is the number of channels 
////////////////////////////////////////////////////
template <int N>
KalmanFilter<N>::KalmanFilter() {
}

////////////////////////////////////////////////////
/// \param *x0 array of initial states of the the channels 
/// \param W standard devation of the prior on the accelerations 
/// \param V noise standard deviation of the position observations 
/// \param DT standard delta-t, in case you want to precompute Kalman gains  
/// 
////////////////////////////////////////////////////
template <int N>
void KalmanFilter<N>::init(double *x0,double W, double V, double DT){
	int n; 
	w2=W*W;				// This is the noise variance on acceleration 
	v2=V*V;				// This is the noise variance on position observation 
	dt=DT;				// size of default time step 
	for (n=0;n<N;n++) { 
		x[n][0]=x0[n];
		x[n][1]=0;
	} 
	A=Matrix2D(1,dt,0,1); 
	dt2=dt*dt;
	dt3=dt2*dt/2;
	Q=Matrix2D((dt2*dt2)/4,dt3,dt3,dt2)*w2;
	P=Q;
}

////////////////////////////////////////////////////
/// \param y[] is an array of the position data observed from the sensors
/// 
////////////////////////////////////////////////////
template <int N>
void KalmanFilter<N>::update(double y[]){

}

////////////////////////////////////////////////////
/// returns position estimate for all channels
/// \param y[] is an array of the position data observed from the sensors
/// \param dt is the the time since the last update (in this case in ms)
///
////////////////////////////////////////////////////
template <int N>
void KalmanFilter<N>::update(double y[],double dt){
	int n; 

	dt2=dt*dt;
	dt3=dt2*dt/2;
	A=Matrix2D(1,dt,0,1); 
	Q=Matrix2D((dt2*dt2)/4,dt3,dt3,dt2)*w2;
	Pm=A*P*A.getTranspose()+Q;
	K=Vector2D(Pm[0][0],Pm[1][0])/(Pm[0][0]+v2);
	P=Matrix2D(1-K[0],0,-K[1],1)*Pm;

    for (n=0;n<N;n++) { 
		xm=A*x[n];
		x[n]=xm+K*(y[n]-xm[0]);
	} 
}

////////////////////////////////////////////////////
/// \param p[] array into which the position data will be laid
///
////////////////////////////////////////////////////
template <int N>
void KalmanFilter<N>::position(double p[]){
	int n;
	for (n=0;n<N;n++) { 
		p[n]=x[n][0];
	} 
}

////////////////////////////////////////////////////
/// \param v[] array into which the velocity data will be laid
///
////////////////////////////////////////////////////
template <int N>
void KalmanFilter<N>::velocity(double v[]){
	int n;
	for (n=0;n<N;n++) { 
		v[n]=x[n][1];
	} 
}

#endif 