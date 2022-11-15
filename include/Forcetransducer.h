// -------------------------------------------------------------------------
// Forcetransducer class 
// 
// under a s626 iocard 
// 
// --------------------------------------------------------------------------

#ifndef FORCETRANSDUCER_H
#define FORCETRANSDUCER_H 

#define NUMFORCEREADINGS 20 

class Forcetransducer { 
public: 
	Forcetransducer(); 
	~Forcetransducer(); 
	bool init(int channel, int board=0);	///< Initialize on the channel 
	void setScale(double scale,double base);///< Set scale and baseline  
	void update();							///< 
	void zeroForce();						///< set baseline to the last 20 readings 
	inline double getForce() {return force[0];}				///< 
	inline double getVolts() {return volts;}				///< 

private: 
	double baseline; 
	double scale; 
	double volts; 
	double force[NUMFORCEREADINGS]; 
	int channel;								///< Channel of AD-card 
	int board;									///< Board number 
}; 

#endif 