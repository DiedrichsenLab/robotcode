// -----------------------------------------------------------------------
// Manages the resource for one or more S626 card
// 
// S626sManager uses counters on board 0 as  
// 0A,as interrupt clock to call the update callback
// 1A,1B as absolute 2MHz clock for accurate timing 		
// Original by Joern Diedrichsen, 2009
// Revised by N.Roach 2011 to utilise digital inputs
// -----------------------------------------------------------------------

#include "S626sManager.h" 
#include <windows.h>
//#include <conio.h>
#include <stdio.h>
#include <iostream>

using namespace std; 

void (* S626sManager::updateCallback)(void);

// --------------------------------------------------------------------------------
// Interrupt handler 
// -----------state---------------------------------------------------------------------
void AppISR()
{
 // Get interrupt request (IRQ) status flags.
    WORD IntStatus[4];							// Array that receives interrupt status.
    S626_InterruptStatus( 0, IntStatus );       // Fetch IRQ status for all sources.

	if ( IntStatus[3] & OVERMASK(CNTR_0A) )
	{
		// Clear counter A overflow capture flag to negate the interrupt request.
		S626_CounterCapFlagsReset( 0, CNTR_0A);
		// get to the callback from the main program
		(*S626sManager::updateCallback)();

	}
	// Enable board’s master interrupt.
	S626_InterruptEnable( 0, true );
}

// --------------------------------------------------------------------------------
// Constructor 
// --------------------------------------------------------------------------------
S626sManager::S626sManager(){ 
	S626_DLLOpen();
	int c,b; 
	for (b=0;b<NUMBOARDS;b++) { 
		for (c=0;c<16;c++){ 
			isUsedAD[b][c]=false;
		} 
		for (c=0;c<4;c++){ 
			isUsedDA[b][c]=false;
		} 
		for (c=0;c<6;c++){ 
			isUsedCounter[b][c]=false;
		}
		numAD[b]=0;
	} 
	numBoards=0; 

} 

// -----------------------------------------------------------------------
// Destructor: Free's resources again
// -----------------------------------------------------------------------
S626sManager::~S626sManager(){ 
	for (int i=0;i<numBoards;i++) { 
		outDA(0,0,i);
		outDA(1,0,i);
		outDA(2,0,i);
		outDA(3,0,i);
		S626_CloseBoard(i);
	} 
    S626_DLLClose();
} 

// -----------------------------------------------------------------------
// S626sManager::init 
/// Opens a board for a certain hardware address
// -----------------------------------------------------------------------
void S626sManager::init(string paramfile) { 
	// Declare Model 626 board to driver and launch the interrupt thread.
	int i; 
	string s; 
	ifstream inputFile(paramfile.c_str(),ios::in);
	if(inputFile ==0){
		cout<<"s626sManager.init: Parameterfile could not be found\n";
		exit(-1); 
	} else{
		inputFile>>numBoards; 
		getline(inputFile,s);
		for (i=0;i<numBoards;i++) { 
			inputFile>>pcibus[i];
			getline(inputFile,s); 
			inputFile>>slot[i]; 
			getline(inputFile,s); 
		} 
		inputFile>>interruptBoard;
	} 

	/// Try to open the boards 
	for (i=0;i<numBoards;i++) { 
		unsigned int address = (pcibus[i]<<16) |  slot[i]; 
		if (interruptBoard==i) { 
			S626_OpenBoard( i, address, AppISR, THREAD_PRIORITY_TIME_CRITICAL);
		} else { 
			S626_OpenBoard( i, address, 0, 0);
		} 
	
		ErrCode = S626_GetErrors( i );

		
		if ( ErrCode ){
	        cout<<( "S626: ONE OR MORE ERRORS DETECTED:" )<<endl;
			if ( ErrCode & 0x00000001 )		printf( " * Failed to open kernel-mode driver\n" );
			if ( ErrCode & 0x00000002 )		printf( " * Can't detect/register board\n" );
			if ( ErrCode & 0x00000004 )		printf( " * Memory allocation error\n" );
			if ( ErrCode & 0x00000008 )		printf( " * Can't lock DMA buffer\n" );
			if ( ErrCode & 0x00000010 )		printf( " * Can't launch interrupt thread\n" );
			if ( ErrCode & 0x00000020 )		printf( " * Can't enable IRQ\n" );
			if ( ErrCode & 0x00000040 )		printf( " * Missed interrupt\n" );
			if ( ErrCode & 0x00000080 )		printf( " * Can't instantiate board object\n" );
			if ( ErrCode & 0x00000100 )		printf( " * Unsupported kernel-mode driver version\n" );
			if ( ErrCode & 0x00010000 )		printf( " * D/A communication timeout\n" );
			if ( ErrCode & 0x00020000 )		printf( " * Illegal counter parameter\n" );
			exit(-1); 
		} else { 
			cout <<"No Error"<<endl;
		} 
	} 
};	




// -----------------------------------------------------------------------
// init the quadrature counter 
// -----------------------------------------------------------------------
void S626sManager::initCounter(int c,int b){ 
	if (isUsedCounter[b][c]) { 
		cout << "S626: Counter Already in use"<< b << " " << c <<endl; 
	}else { 
		S626_CounterModeSet( b, c,
			( LOADSRC_INDX << BF_LOADSRC ) | // Index causes preload.
			( INDXSRC_HARD << BF_INDXSRC ) | // Hardware index is enabled.
			( INDXPOL_POS << BF_INDXPOL ) | // Active high index.
			( CLKSRC_COUNTER << BF_CLKSRC ) | // Operating mode is Counter.
			( CLKPOL_NEG << BF_CLKPOL ) | // Active high clock.
			( CLKMULT_4X << BF_CLKMULT ) | // Clock multiplier is 4x.
			( CLKENAB_ALWAYS << BF_CLKENAB ) ); // Counting is always enabled.
		// Initialize preload value to zero so that the counter core will be set
		// to zero upon the occurance of an Index.
		S626_CounterPreload( b, c, 0x000000 );
		// Enable latching of accumulated counts on demand. This assumes that
		// there is no conflict with the latch source used by paired counter 2B.
		S626_CounterLatchSourceSet( b, c, LATCHSRC_AB_READ );
		// Disable Interrupts 
		S626_CounterIntSourceSet( b, c, INTSRC_NONE);
		isUsedCounter[b][c]=true;
	} 
} 

// -----------------------------------------------------------------------
/// init a single-phase TTL Counter 
// -----------------------------------------------------------------------
void S626sManager::initTTLCounter(int c,int b){ 
	if (isUsedCounter[b][c]) { 
		cout << "S626: Counter Already in use: "<< b << " " <<c<<endl; 
	}else { 
		S626_CounterModeSet( b, c,
			( LOADSRC_INDX << BF_LOADSRC ) | // Index causes preload.
			( INDXSRC_SOFT << BF_INDXSRC ) | // Only Software index is enabled.
			( CLKSRC_COUNTER << BF_CLKSRC ) | // Operating mode is Counter.
			( CLKPOL_POS << BF_CLKPOL ) |	  // Detect up-events .
			( CLKMULT_1X << BF_CLKMULT ) | // Counter Multiplier set to 1x.
			( CLKENAB_ALWAYS << BF_CLKENAB ) ); // Counting is always enabled.
		// Initialize preload value to zero so that the counter core will be set
		// to zero upon the occurance of an Index.
		S626_CounterPreload( b, c, 0x000000 );
		// Enable latching of accumulated counts on demand. This assumes that
		// there is no conflict with the latch source used by paired counter 2B.
		S626_CounterLatchSourceSet( b, c, LATCHSRC_AB_READ );
		// Disable Interrupts 
		S626_CounterIntSourceSet( b, c, INTSRC_NONE);
		S626_CounterSoftIndex(b,c);
		isUsedCounter[b][c]=true; 
	} 
} 

// -----------------------------------------------------------------------
// init the counter as a Timer 
// -----------------------------------------------------------------------
void S626sManager::initTimer(int c, int b) { 
	if (isUsedCounter[b][c]) { 
		cout << "S626: Counter Already in use"<< b << " " <<c<<endl; 
	} else { 
		S626_CounterModeSet( b, c,
			( LOADSRC_INDX << BF_LOADSRC ) | // Index causes preload.
			( INDXSRC_SOFT << BF_INDXSRC ) | // Hardware index is enabled.
			( INDXPOL_NEG << BF_INDXPOL ) | // Active high index.
			( CLKSRC_TIMER << BF_CLKSRC ) | // Operating mode is Counter.
			( CNTDIR_UP << BF_CLKPOL ) | // Counter direction up.
			( CLKMULT_1X << BF_CLKMULT ) | // Clock multiplier is 4x.
			( CLKENAB_ALWAYS << BF_CLKENAB ) ); // Counting is always enabled.
		// Initialize preload value to zero so that the counter core will be set
		// to zero upon the occurance of an Index.
		S626_CounterPreload( b, c, 0x000000 );
		// Disable Interrupts 
		S626_CounterIntSourceSet( b, c, INTSRC_NONE);
		S626_CounterSoftIndex( b, c);
		// Enable latching of accumulated counts on demand. This assumes that
		// there is no conflict with the latch source used by paired counter 2B.
		S626_CounterLatchSourceSet( b, c, LATCHSRC_AB_READ );
		isUsedCounter[b][c]=true; 
	} 
} 

// -----------------------------------------------------------------------
// init counter as overflow timer
// -----------------------------------------------------------------------
void S626sManager::initOverflow(int c,int b) { 
	if (isUsedCounter[b][c]) { 
		cout << "S626: Counter Already in use"<< b << " " <<c<<endl; 
	}else { 
		S626_CounterModeSet( b, c,
			( LOADSRC_INDX << BF_LOADSRC ) | // Index causes preload.
			( INDXSRC_SOFT << BF_INDXSRC ) | // Hardware index is enabled.
			( INDXPOL_NEG << BF_INDXPOL ) | // Active high index.
			( CLKSRC_EXTENDER << BF_CLKSRC ) | // Operating mode is Counter.
			( CNTDIR_UP << BF_CLKPOL ) | // Counter direction up.
			( CLKMULT_1X << BF_CLKMULT ) | // Clock multiplier is 4x.
			( CLKENAB_ALWAYS << BF_CLKENAB ) ); // Counting is always enabled.
		// Initialize preload value to zero so that the counter core will be set
		// to zero upon the occurance of an Index.
		S626_CounterPreload( b, c, 0x000000);
		S626_CounterSoftIndex( b, c);
		// Enable latching of accumulated counts on demand. This assumes that
		// there is no conflict with the latch source used by paired counter 2B.
		S626_CounterLatchSourceSet( b, c, LATCHSRC_AB_READ );
		// disable all interrupts from this counter.
		S626_CounterIntSourceSet( b, c, INTSRC_NONE);
		isUsedCounter[b][c]=true; 
	} 
} 

// -----------------------------------------------------------------------
// init the counter 0A on board 0 as the interrupt counter 
// -----------------------------------------------------------------------
void S626sManager::initInterrupt(void (*fcn)(void),double updateR) { 
	updateCallback=fcn;				// This is the callback to register 
	updateRate=updateR; 
	int c=CNTR_0A;
		S626_CounterModeSet( 0, c,
			( LOADSRC_INDX << BF_LOADSRC ) | // Index causes preload.
			( INDXSRC_SOFT << BF_INDXSRC ) | // Hardware index is enabled.
			( CLKSRC_TIMER << BF_CLKSRC ) | // Operating mode is Counter.
			( CNTDIR_DOWN << BF_CLKPOL ) | // Counter direction down.
			( CLKMULT_1X << BF_CLKMULT ) | // Clock multiplier is 4x.
			( CLKENAB_INDEX << BF_CLKENAB ) ); // Counting is initially diabled.
		// one less, because the counter has to go down to zero 
		S626_CounterPreload( 0, c, updateRate * 2000-1);
		// Generate a soft index to force transfer of PreLoad value into counter core.
		S626_CounterSoftIndex( 0, c);
		// Enable transfer of PreLoad value to counter in response to overflow. This
		// will cause the initial counts to reload every time the counts reach zero.
		S626_CounterLoadTrigSet( 0, c, LOADSRC_OVER );
		// Enable the counter to generate interrupt requests upon captured overflow.
		S626_CounterIntSourceSet( 0, c, INTSRC_OVER );
		// Enable the timer. The first interrupt will occur after the specified
		// time interval elapses.
		S626_CounterEnableSet( 0, c, CLKENAB_ALWAYS );
		
		// Enable the interrupt routine
		S626_InterruptEnable( 0, true );
} 

// -----------------------------------------------------------------------
// stop Interrupt 
// -----------------------------------------------------------------------
void S626sManager::stopInterrupt() { 
		S626_InterruptEnable( 0, false);
} 

// -----------------------------------------------------------------------
/// get Quadrature counter
/// Channels: 
/// 0: 0A 1:1A 2:2A 3:0B 4:1B 5:2B 
// -----------------------------------------------------------------------
long S626sManager::getCounter(int c,int b) { 
	long x;
	x=S626_CounterReadLatch(b,c);
	if (x>0x800000){
		x=x-0xffffff-1;
	}
	return x;
} 

// -----------------------------------------------------------------------
/// reset Counter using soft index 
// -----------------------------------------------------------------------
void S626sManager::resetCounter(int c,int board){ 
	S626_CounterSoftIndex(board,c);
} 

// -----------------------------------------------------------------------
// get the absolute Time in ms 
// -----------------------------------------------------------------------
double S626sManager::getTime(int lowchannel,int highchannel) { 
	// Get absolute time from counter 1A / 1B 
	unsigned long a1,b1;
	double da1,db1;
	a1=S626_CounterReadLatch(0,lowchannel);
	if (highchannel>=0) { 
		b1=S626_CounterReadLatch(0,highchannel);
	} else { 
		b1=0; 
	} 

		db1 = (double)b1; 
	da1 = (double)a1; 

	return(da1+db1*16777216)/2000; 
} 


// -----------------------------------------------------------------------
// resetTime: set Timer to zero
// -----------------------------------------------------------------------
void S626sManager::resetTime(int lowchannel, int highchannel) { 
	// Cause soft index on both counter-register, setting them both to zero
	S626_CounterSoftIndex( 0, lowchannel);
	if (highchannel >=0) { 
		S626_CounterSoftIndex( 0, highchannel);
	} 
} 

// -----------------------------------------------------------------------
// registerAD 
// -----------------------------------------------------------------------
int S626sManager::registerAD(int c,int r,int b){ 
	if (isUsedAD[b][c]) { 
		cout << "s626error: Channel already in use:"<< b << " " <<c<<endl; 
		exit(-1);
	} 
	if (numAD[b]>15) { 
		cout<< "s626error: Too many channels registered"<<endl;
	} 

	range[b][numAD[b]]=r;
	if (r==5) { 
		poll_list[b][numAD[b]] = ((c & ADC_CHANMASK) | ADC_RANGE_5V | ADC_EOPL); // Chan c, range.
	} else { 
		poll_list[b][numAD[b]] = ((c & ADC_CHANMASK) | ADC_RANGE_10V | ADC_EOPL); 
	} 
	if (numAD>0) { 
		poll_list[b][numAD[b]-1]=poll_list[b][numAD[b]-1] - ADC_EOPL;
	} 
	numAD[b]++;
	S626_ResetADC( b, poll_list[b]);	
	//for (int i=0;i<numAD[b];i++){ 
	//	cout<<(int)poll_list[b][i]<<endl;
	//} 
	return(numAD[b]-1);
}

// -----------------------------------------------------------------------
// get AD channel 
// -----------------------------------------------------------------------
double S626sManager::getAD(int c,int b) { 
	return(((double)databuf[b][c])/32767*(double)range[b][c]);
} 

// -----------------------------------------------------------------------
// updateAD 
// -----------------------------------------------------------------------
void S626sManager::updateAD(int b){ 
	S626_ReadADC( b, databuf[b]);	
} 


// -----------------------------------------------------------------------
// output to DA 
#define DAC_VSCALAR 819.1 // Binary-to-volts scalar for DAC.
void S626sManager::outDA(int channel, double volts, int board){
// Make adjustments to prevent conversion errors.
	if ( volts > 10.0 ) volts = 10.0;
	else if ( volts < -10.0 ) volts = -10.0;
	// Program new DAC setpoint.
	S626_WriteDAC( board, channel, (LONG)( volts * DAC_VSCALAR ) );
}

// -----------------------------------------------------------------------
// output to DIO
void S626sManager::outDIO(int channel, short setting, int board){
	// Sets the channel i from exisiting value to new value
	int group; 
	int bit; 
	WORD mask;
	WORD state; 
	
	if (channel>47 || channel<0) { 
		cout<< "s626DIO: Illegal channel number:"<< channel << endl; 
		return; 
	} 
	group=channel/16; 
	bit=(channel%16); 
	state =S626_DIOWriteBankGet(board,group); 
	if (setting!=0 && setting!=1) { 
		cout<<"s626DIO: illegal setting (0/1)"<<endl;
	} 
	mask=(1 << bit);		// Binary shift operator 
	// printf("Exist: %x\n",state);  
	state=state & ~mask; 
	// printf("Mask: %x\n",mask);  
	mask=setting << bit; 
	state = state | mask; 
	// printf("New: %x\n",state);  
	S626_DIOWriteBankSet(board,group,state); 
}
// -----------------------------------------------------------------------
// read DIO
unsigned int S626sManager::readDIO(int channel, int board){
	// Gets a bank of 16 DIO bits
	unsigned int state = 0;
	int group; 
	int bit; 
	unsigned int mask;
		
	if (channel>47 || channel<0) { 
		cout<< "s626DIO: Illegal channel number:"<< channel << endl; 
		return (-1); 
	} 

	group=channel/16; 
	bit=(channel%16); 	
	state=S626_DIOReadBank(0,0); 
	mask=(1 << bit);		// Binary shift operator 
	state=state&mask;
	state=(state>> bit);

	return (state);
}
