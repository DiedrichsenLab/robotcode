//////////////////////////////////////////////////////////////
//   
//  Timer.h
//  Timer class 
//  Six timers are available and can be reset seperately 
//  EXAMPLE: 
//  reset(1) 
//  while (gTimer.[1]<100) { 
//  }  
// 
//   Written 2012 joern Diedrichsen (j.diedrichsen@ucl.ac.uk) 
//////////////////////////////////////////////////////////////
#ifndef NUMTIMERS 
#define NUMTIMERS 6

//////////////////////////////////////////////////////////////
///  \brief Timer class that works with the s626 board
/// 
/// The class holds two clocks 
/// \li  one that counts up every update cycles by dt,
/// \li one that reads the  "real" time from the s626 board 
/// Furthermore, it has 6 stop-watches (working on the first clock) 
/// that can be reset to zero as you wish. They simply remember when they got reset and give you  
/// the difference between the nowtime and their last resetting time. 
/// Usually the timers are used in the following fashion
/// \li 0: Timer that gives you the time from the start of the last block
/// \li 1: Timer that gives you the time from the start of the current trial 
/// \li 2: Flexible event timer that you can use in MyTrial::control() to stop the time since the last event. 
/// \li 3: USed by screen to keep track of the screen refresh rate 
/// \li 4: Time elapsed since the last control loop call 
/// \li 5: Time elapsed since the last data record was done. 
/////////////////////////////////////////////////////////////////
class Timer { 
public: 
	Timer();					///< Constructor: default is 1ms update cycles 
	void init();	///< Initialize Timer on board 0, any combination of channels 
	void reset(int i);					///< reset timer number i to zero
	void clear();						///< clear all timers and clocks 
	void countupReal();					///< get the real time phatom or s626 device 
	inline void countup() {countupReal();}	///< Increase update count by update cycle 
	inline double read(int number)		///< read clock 1, timer i
		{return (nowtime-null[number]);}
	inline double readReal(int number)	///< read clock 2, timer i
		{return (realtime-nullReal[number]);}
	inline double operator[] (int number) ///< read clock 1, timer i 
		{return (nowtime-null[number]);}
	inline double dt()						///< elapsed time since last updtae 
		{return realtime-lastrealtime;}
	double nowtime;
	double realtime; 
	double null[NUMTIMERS];
	double nullReal[NUMTIMERS];
private:
	double lastrealtime;
	long last_a1; 
	double updatecycle; 
};

#endif 