///////////////////////////////////////////////////////////////////////////
/// TRCounter class: Keeps track of the 
/// TRCounter either extrnally triggered or internally simulated 
/// uses Counter 0B from the first s626 manager 
/// Joern Diedrichsen, 2007, Tobias Wiestler, Alexandra Reichenbach  
/// j.diedrichsen@bangor.ac.uk

///	Jan 2009 Extend to read the update signal from a serial port using the Serial.cpp of Ramon de Klein
/// Jan 2011 Slice counter Siemens Trio
/// May 2012 Slice counter bug fix in simulation
/// Oct 2013 Adjustments for external TTL pulse - cleaned up couting overall
///			 Cleaned up the structure of the TR counter overall
/// Mode: 
///			1: Serial character - receiving the slice number with every slice (FIL + BUCNI) 
///         2: Serial character - receiving a single character ('T') for every TR (Philips) 
///			3: TTL counter - Receiving a TTL pulse that is processed by the s626 card with every TR 
///         4: TTL counter - Receiving a TTL pulse with every slice (not implemented yet) 
/// In all modes, the TR counter should sit on 
/// TR=0 slice=0 totalTime=0, if the first pulse has not been received 
/// After the first pulse 
/// TR=1 slice=1 totalTime=0. 
/// Then these three should start counting up until the end of the scan
///
/// In simulation mode, TR and currentSlice are set to 1 when the first update call happens 
/// In the modes that get a signal per TR (TTL + Phillips), current slice is 
/// Not counted up.
/// 
/// Needs to have the gTimer class initialized and running 
/////////////////////////////////////////////////////////////////////////////


#include "TRCounter626.h" 
#include "Timer626.h" 
#include "S626sManager.h" 

#include <tchar.h>
#include <windows.h>
#include "Serial.h"
#include <bitset>
#include <math.h>

extern S626sManager s626;
extern Timer gTimer; 

/// -----------------------------------------------------------------------
/// Constructor 
/// -----------------------------------------------------------------------
TRCounter::TRCounter(){ 
	nulltime=0;
	TR=0;
	currentSlice= 0; 
	lengthTR=2700;
	isSimulated=1;
	isRunning=0; 
	sliceNumber=1; 
} 

void TRCounter::init(int channel,int board){ 
	init3(channel,board,1);
}

// -----------------------------------------------------------------------
//  TRCounter ::init1 
/// Initialize the TRCounter with Serial Port receiving the serial number of the slice with every slice 
/// comport: "COM1"
/// baud: 196000 
/// int numSlices 
/// ----------------------------------------------------------------------
void TRCounter::init1(LPCTSTR comport, int baud, int numSlices){ 
	mode = 1; 
	sliceNumber= numSlices;
	openSerial(comport,baud); 
} 

// -----------------------------------------------------------------------
//  TRCounter ::init2 
/// Initialize the TRCounter with Serial Port receiving on character each TR 
/// comport: "COM1"
/// baud: 196000 
/// int numSlices 
/// ----------------------------------------------------------------------
void TRCounter::init2(LPCTSTR comport, int baud, char trchar, int numSlices){ 
	mode = 2; 
	counterChar=trchar;
	sliceNumber= numSlices;
	openSerial(comport,baud); 
} 


// -----------------------------------------------------------------------
//  TRCounter ::init3 
/// Initialize with TTL counter getting a TTL every single TR  
/// int c: Counter number : often 3
/// int board: often 0 
/// int numSlices 
/// ----------------------------------------------------------------------
void TRCounter::init3(int c,int b,int numSlices){ 
	mode = 3; 
	channel=c; 
	board=b;
	sliceNumber= numSlices;
	s626.initTTLCounter(channel,board);
} 


/// -----------------------------------------------------------------------
///  TRCounter ::init4 
/// Initialize with TTL counter getting a TTL every single slice  
/// int c: Counter number : often 3
/// int board: often 0 
/// int numSlices 
/// ----------------------------------------------------------------------
void TRCounter::init4(int c,int b,int numSlices){ 
	cout << "mode 4 not implemented yet"<<endl; 
	exit(-1); 
} 


/// -----------------------------------------------------------------------
/// TRCounter :: openSerial  
/// Initialize with TTL counter getting a TTL every single slice  
/// comport: "COM1"
/// baud: 196000 
/// ----------------------------------------------------------------------
void TRCounter::openSerial(LPCTSTR comport,int baud) { 
	//Open the serial port
	lLastError = nSerial.Open(_T(comport),0,0,true);
	//ERROR HANDLING---------------
	if (lLastError != ERROR_SUCCESS)
	{
		std::cout<<"Unable to open port "<<lLastError<<" "<<ERROR_SUCCESS <<std::endl;
		nSerial.Close();
	}
	//-----------------------------
	
    // Setup the serial port 
    lLastError = nSerial.Setup( CSerial::EBaudrate(baud),				
		CSerial::EData8,
		CSerial::EParNone,
		CSerial::EStop1);
	//ERROR HANDLING---------------
	if (lLastError != ERROR_SUCCESS)
	{
		std::cout<<"Unable to configure port "<<lLastError<<" "<<ERROR_SUCCESS <<std::endl;
		nSerial.Close();
	}
	//-----------------------------
	
	//Setup the Handshakeing
    lLastError = nSerial.SetupHandshaking(CSerial::EHandshakeHardware);
	
	//ERROR HANDLING---------------
	if (lLastError != ERROR_SUCCESS)
	{
		std::cout<<"Unable to set handshaking"<<lLastError<<" "<<ERROR_SUCCESS <<std::endl;
		nSerial.Close();
	}
	//-----------------------------
	
	// Register only for the receive event
    lLastError = nSerial.SetMask(CSerial::EEventBreak |
		CSerial::EEventCTS   |
		CSerial::EEventDSR   |
		CSerial::EEventError |
		CSerial::EEventRing  |
		CSerial::EEventRLSD  |
		CSerial::EEventRecv);
	//ERROR HANDLING---------------
	if (lLastError != ERROR_SUCCESS)
		std::cout<<"Unable to set COM-port event mask"<<std::endl;	
	//-----------------------------	
	
	// Setup the read timeout
	lLastError = nSerial.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
	//ERROR HANDLING---------------
	if (lLastError != ERROR_SUCCESS)
		std::cout<<"Unable to set blocking."<<std::endl;
	
	//-----------------------------	
	hevtOverlapped = ::CreateEvent(0,TRUE,FALSE,0);
	if (hevtOverlapped == 0) {
		std::cout<<"Unable to create manual-reset event for overlapped I/O."<<std::endl;
		nSerial.Close();
	}
	
	std::cout<<"serial is initialized"<<std::endl;
} 



// -----------------------------------------------------------------------
// TRCounter::reset 
/// -----------------------------------------------------------------------
void TRCounter::reset(){ 
	nulltime=gTimer.getRealtime();
	TR=0;
	currentSlice= 0; 
	oldCounter=0; 
	newCounter=0; 
	switch (mode) { 
	case 1: 
	case 2:
		nSerial.Purge(); 
		break; 
	case 3:
	case 4: 
		s626.resetCounter(channel,board);						///< reset the counter to zero
		break; 
	} 
} 

// -----------------------------------------------------------------------
// TRCounter ::update 
/// runs in different modes for simulation, serial Phillips, serial Siemens and s626 
/// ----------------------------------------------------------------------

bool TRCounter::update(){ 
	//_CrtMemCheckpoint(&s1);

	OVERLAPPED ov={0}; 
	if (!isRunning) {
		return (true); 
	} 
	
	nowtime=gTimer.getRealtime();
	
	//------------------------------------------------------	
	// Simulation mode 
	// -----------------------------------------------------
	if (isSimulated)  {	// simulated: check S626 Timer 
		
		//  if it hasn't started yet - start the TR counter 
		if (TR==0) {
			nulltime=nowtime-1;
			starttime=nowtime-1;
			TR=1;
			currentSlice=1;
		}
		currentSlice= ceil((nowtime-nulltime)/(lengthTR/sliceNumber)); // AR: added ceil (once the slice acquisition has begun, the slice is counted up) 
		TR = ceil((nowtime-nulltime)/lengthTR); 
		starttime=((double)TR-1)*lengthTR+nulltime;  
		return(true); 
	}  
	
	switch (mode) { 
		///-----------------------------
		// Siemens: 2 Serial characters per slice
		// First slice starts at 1, and then keeps counting up continously
		///-----------------------------
	case 1: 
		dwBytesRead = 0; 	
		// Wait for an event
		// Setup the overlapped structure
		ov.hEvent = hevtOverlapped;				// Issue new asynchronous event 
		nSerial.WaitEvent(&ov, 0);	
		eEvent = nSerial.GetEventType();  			// Save event
		if (eEvent & CSerial::EEventRecv){
			///Read the input character by character
			//  data from the COM-port
			do { 
				lLastError = nSerial.Read(szBuffer,2,&dwBytesRead);
				if (lLastError != ERROR_SUCCESS){ 
					std::cout<<"Unable to read from COM-port."<<std::endl;
				}
				if (dwBytesRead==2) { 
					currentSlice= (((BYTE)szBuffer[0] & 0xFFFF)<<8|((BYTE)szBuffer[1] & 0xFFFF));
					TR=((currentSlice-1)/sliceNumber)+1;		// Slice 1...sliceNumber is TR 1, sliceNumber+1... is TR 2
					if ((currentSlice-1) % sliceNumber ==0 ){	// New TR has started 
						starttime=nowtime;
					}
				}
			} while (dwBytesRead==2);		/// Finish up the event  
		}
		break;
		///-----------------------------
		// Philips : 1 Serial characters per TR (couterChar)
		///-----------------------------
	case 2: 
		dwBytesRead = 0; 	
		// Wait for an event
		// Setup the overlapped structure
		ov.hEvent = hevtOverlapped;				// Issue new asynchronous event 
		nSerial.WaitEvent(&ov, 0);	
		eEvent = nSerial.GetEventType();  			// Save event
		
		if (eEvent & CSerial::EEventRecv){
			///Read the input character by character
			//  data from the COM-port
			do { 
				lLastError = nSerial.Read(szBuffer,1,&dwBytesRead);
				if (lLastError != ERROR_SUCCESS){ 
					std::cout<<"Unable to read from COM-port."<<std::endl;
				}
				if (dwBytesRead==1) { 
					if (szBuffer[0]==counterChar) {	
						TR=TR++;
						starttime=nowtime;
					} else {  //PRINT buffer if it doesn't equal to the counterChar
						std::cout<<"Character for the TR-Counter is set to: "<<counterChar
							<<" Current Buffer: "<<szBuffer[0]<<"  "<<((unsigned short)(szBuffer[0]))<<endl; 
					} 
				}
			} while (dwBytesRead==1);		/// Finish up the event  
		}
		break;
		///-----------------------------
		// 1 TTL pulse per TR 
		///-----------------------------
	case 3:
		newCounter=s626.getCounter(channel,board);
		if (newCounter!=oldCounter && (TR==0 || (nowtime-starttime)>50)){ 
			TR++; 
			oldCounter=newCounter; 
			starttime=nowtime; 
		} 
		break;
	}
	
	/// is Running, but no pulse has been received yet: reset the clock until first one is received 
	if (TR==0) { 
		nulltime=gTimer.getRealtime();
		starttime=gTimer.getRealtime(); 
	} 

	return(true); 
} 
		
		
		
/// -----------------------------------------------------------------------
/// Sets the mode either to Simulates (when lengthTR>0) or to TR counting (lengthTR<=0)
/// -----------------------------------------------------------------------
void TRCounter::simulate(double lt){ 
	if (lt>0){ 
		isSimulated=true;
		lengthTR=lt;
	} else { 
		isSimulated=false; 
		lengthTR=-lt;
	}	 
}

/// -----------------------------------------------------------------------
/// isRunning = true. Simulate as if you had gotten the first TR
/// -----------------------------------------------------------------------
void TRCounter::start() { 
	TR=0;
	currentSlice=0;
	starttime=gTimer.getRealtime(); 
	nulltime=gTimer.getRealtime(); 
	isRunning=true;
	if(!isSerial) { 
		s626.resetCounter(channel,board);						///< reset the counter to zero	
		oldCounter=0; 
	} 
} 