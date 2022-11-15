#include "Timer.h" 
#include <time.h>
#include <stdio.h>
#include <iostream>
using namespace std; 

///////////////////////////////////////////////////////////////////////////
/// Timer storage class: Keeps track of number of update cycles and real time 
/// This version takes the time  from system clock (clock) 
/// Joern Diedrichsen, 2007 
/// j.diedrichsen@bangor.ac.uk
/////////////////////////////////////////////////////////////////////////////

/// -----------------------------------------------------------------------
/// Constructor 
/// -----------------------------------------------------------------------
Timer::Timer(){ 
	for (int i=0;i<NUMTIMERS;i++){ 
		null[i]=0;
		nullReal[i]=0;
	} 
	nowtime=0;realtime=0;lastrealtime=-0.001;
} 

/// -----------------------------------------------------------------------
/// init
/// -----------------------------------------------------------------------
void Timer::init(){ 
} 

/// -----------------------------------------------------------------------
/// reset: Sets a single timer to zero
/// -----------------------------------------------------------------------
void Timer::reset(int timer){ 
	if (timer >=0 && timer < NUMTIMERS) { 
		null[timer]=nowtime;
		nullReal[timer]=realtime;
	} 
} 


/// -----------------------------------------------------------------------
/// clear: Clears all the timers to zero 
/// -----------------------------------------------------------------------
void Timer::clear(){ 

	nowtime=0;
	lastrealtime=0;
	realtime=0;
	last_a1=0;
	for (int i=0;i<NUMTIMERS;i++){ 
		null[i]=0;
		nullReal[i]=0;
	} 

} 

/// -----------------------------------------------------------------------
/// countup real: gets the real time from the s626 device 
/// -----------------------------------------------------------------------
void Timer::countupReal() { 
	clock_t time; 
	time=clock(); 
	realtime=((double)time)/CLOCKS_PER_SEC*1000; 
	nowtime=realtime; 
} 
