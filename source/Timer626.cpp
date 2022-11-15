#include "Timer626.h" 
#include "S626sManager.h" 

extern S626sManager s626;

///////////////////////////////////////////////////////////////////////////
/// Timer storage class: Keeps track of number of update cycles and real time 
/// this version updates itself from the s626-board; 
/// Joern Diedrichsen, 2007 
/// j.diedrichsen@bangor.ac.uk
/////////////////////////////////////////////////////////////////////////////

/// -----------------------------------------------------------------------
/// Constructor 
/// -----------------------------------------------------------------------
Timer::Timer(double dt){ 
	for (int i=0;i<NUMTIMERS;i++){ 
		null[i]=0;
		nullReal[i]=0;
	} 
	nowtime=0;
	realtime=0;
	lastrealtime=-0.001;
	updatecycle=dt;
	lowchannel=-1;
	highchannel=-1; 
} 

/// -----------------------------------------------------------------------
/// init
/// -----------------------------------------------------------------------
void Timer::init(int low, int high, int board){ 
	lowchannel=low;
	highchannel=high; 
	timerBoard=board; 
	s626.initTimer(lowchannel,timerBoard);				///< Timer is made up from 1A + 1B 
	if(highchannel>=0) { 
		s626.initOverflow(highchannel,timerBoard);
	} 
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
	S626_InterruptEnable( 0, false);

	nowtime=0;
	lastrealtime=0;
	counterOverflow=0; 
	realtime=0;
	last_a1=0;
	for (int i=0;i<NUMTIMERS;i++){ 
		null[i]=0;
		nullReal[i]=0;
	} 
	S626_CounterSoftIndex( 0, lowchannel);
	if (highchannel >=0) { 
		S626_CounterSoftIndex( 0, highchannel);
	} 
	S626_InterruptEnable( 0, true);
} 

/// -----------------------------------------------------------------------
/// countup real: gets the real time from the s626 device 
/// -----------------------------------------------------------------------
void Timer::countupReal() { 
	unsigned long a1,b1;
	double da1,db1; 

	lastrealtime=realtime;
	
	a1=S626_CounterReadLatch(0,lowchannel);

	if (highchannel>=0) { 
		b1=S626_CounterReadLatch(0,highchannel);
	} else { 
		if (a1<last_a1) {
			counterOverflow++;
		} 
		b1=counterOverflow; 
	} 
	last_a1=a1;
	
	db1 = (double)b1; 
	da1 = (double)a1; 

	realtime=(da1+db1*16777216)/2000; 
	/// Old version was 
	// (a1+(b1<<24))/2000
	// led to overflow every 32 bit, as calculation was 
	// handled internaly as 32 bit number


} 
