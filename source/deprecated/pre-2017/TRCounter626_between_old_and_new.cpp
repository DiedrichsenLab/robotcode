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

///////////////////////////////////////////////////////////////////////////
/// TRCounter class: Keeps track of the 
/// TRCounter either extrnally triggered or internally simulated 
/// uses Counter 0B from the first s626 manager 
/// Joern Diedrichsen, 2007, Tobias Wiestler 
/// j.diedrichsen@bangor.ac.uk

///	Jan 2009 Extend to read the update signal from a serial port using the Serial.cpp of Ramon de Klein
/// Jan 2011 Slice counter Siemens Trio
/// May 2012 Slice counter bug fix in simulation
/// Oct 2013 Adjustments for external TTL pulse - cleaned up couting overall
///
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
/// -----------------------------------------------------------------------
/// Constructor 
/// -----------------------------------------------------------------------
TRCounter::TRCounter(){ 
} 

/// -----------------------------------------------------------------------
// TRCounter ::init 
/// Initialize the TR Counter with the Counter on the s626 board
/// -----------------------------------------------------------------------
void TRCounter::init(int c,int b){ 
	nulltime=0;
	TR=0;
	currentSlice= 0; 
	lengthTR=2700;
	isSimulated=1;
	isRunning=0; 
	channel=c; 
	board=b;
	s626.initTTLCounter(channel,board);
	isSerial=false;
} 

// -----------------------------------------------------------------------
//  TRCounter ::initSerial 
/// Initialize the TRCounter with Serial Port Action
/// comport: "COM1"
/// baud: 196000 
/// trchar: Character that is sent for a TR
/// ----------------------------------------------------------------------
void TRCounter::initSerial(LPCTSTR comport, int baud, char trchar, int numSlices){ 
	nulltime=0;
	TR=0;
	currentSlice= 0; //Tobi May 2012
	lengthTR=2000;
	isSimulated=0;
	isRunning=0; 
	isSerial=true;
	counterChar=trchar;
	sliceNumber= numSlices;
	
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
	
	std::cout<<"serial is init "<<std::endl;
} 
// -----------------------------------------------------------------------
// TRCounter::reset 
/// -----------------------------------------------------------------------
void TRCounter::reset(){ 
	nulltime=gTimer.getRealtime();
	TR=0;
	lastTR=0;
	currentSlice= 0; 
	if (isSerial) { 
		nSerial.Purge(); 
	} else { 
		s626.resetCounter(channel,board);						///< reset the counter to zero 	
	} 
} 

// -----------------------------------------------------------------------
// TRCounter ::update 
/// runs in different modes for simulation, serial Phillips, serial Siemens and s626 
/// ----------------------------------------------------------------------

bool TRCounter::update(){ 
	if (isRunning) 	{ 
		nowtime=gTimer.getRealtime();
		//------------------------------------------------------	
		// Simulation mode 
		// -----------------------------------------------------
		if (isSimulated)  {	// simulated: check S626 Timer 
			
			// AR: moved this before the if condition and added ceil (once the slice acquisition has begun, the slice is counted up) 
			if (TR==0 && currentSlice==0) {
				nulltime=nowtime-1;
				starttime=nowtime;
				TR=1;
				currentSlice=1;
			}
			currentSlice= ceil((nowtime-nulltime)/(lengthTR/sliceNumber)); 
			if ((nowtime-starttime)>lengthTR)
			{ 
				TR++; 
				lastTR=nowtime-starttime; 
				starttime=nowtime; 
				//cout << TR << "\t" << currentSlice << "\t" << (nowtime-nulltime)/(lengthTR/sliceNumber) << endl;
				return(true); 
			} 			
		}  
		//------------------------------------------------------	
		// Serial mode 
		// -----------------------------------------------------
		else if (isSerial) 
		{		
			HANDLE hevtOverlapped = ::CreateEvent(0,TRUE,FALSE,0);;
			if (hevtOverlapped == 0)
			{
				std::cout<<"Unable to create manual-reset event for overlapped I/O."<<std::endl;
				nSerial.Close();
			}
			// Setup the overlapped structure
			OVERLAPPED ov = {0};
			ov.hEvent = hevtOverlapped;
			
			// Wait for an event
			nSerial.WaitEvent(&ov, 0);
			
			
			// Save event
			const CSerial::EEvent eEvent = nSerial.GetEventType();
			// HANDEL EVENTS-------------------
			/// Handle break event
			if (eEvent & CSerial::EEventBreak){		
				//printf("\n### BREAK received ###\n");
			}
			/// Handle CTS event
			if (eEvent & CSerial::EEventCTS){
				//printf("\n### Clear to send %s ###\n", nSerial.GetCTS()?"on":"off");
			}
			/// Handle DSR event
			if (eEvent & CSerial::EEventDSR){
				//printf("\n### Data set ready %s ###\n", nSerial.GetDSR()?"on":"off");
			}
			/// Handle error event
			if (eEvent & CSerial::EEventError){
				printf("\n### ERROR: ");
				switch (nSerial.GetError()){
				case CSerial::EErrorBreak:		printf("Break condition");			break;
				case CSerial::EErrorFrame:		printf("Framing error");			break;
				case CSerial::EErrorIOE:		printf("IO device error");			break;
				case CSerial::EErrorMode:		printf("Unsupported mode");			break;
				case CSerial::EErrorOverrun:	printf("Buffer overrun");			break;
				case CSerial::EErrorRxOver:		printf("Input buffer overflow");	break;
				case CSerial::EErrorParity:		printf("Input parity error");		break;
				case CSerial::EErrorTxFull:		printf("Output buffer full");		break;
				default:						printf("Unknown");					break;
				}
				printf(" ###\n");
			}
			/// Handle ring event
			if (eEvent & CSerial::EEventRing){
				//printf("\n### RING ###\n");
			}
			/// Handle RLSD/CD event
			if (eEvent & CSerial::EEventRLSD){
				//printf("\n### RLSD/CD %s ###\n", nSerial.GetRLSD()?"on":"off");
			}
			///-----------------------------
			//Handle data receive event 
			///-----------------------------
			if (eEvent & CSerial::EEventRecv){
				DWORD dwBytesRead = 0;
				char szBuffer[3];
				///Read the input character by character
				do{
					// Read data from the COM-port
					lLastError = nSerial.Read(szBuffer,sizeof(szBuffer)-1,&dwBytesRead);
					if (lLastError != ERROR_SUCCESS)
						std::cout<<"Unable to read from COM-port."<<std::endl;
					if (dwBytesRead > 0){
						///-----------------------------
						// Siemens: 2 Serial characters per slice
						// First slice starts at 1, and then keeps counting up continously
						///-----------------------------
						if (sliceNumber>0){ // if slice number is set count the TR accourding to the slices	
							if (dwBytesRead!=2) { 
								cout<< "Got " << dwBytesRead << " Serial Characters"<<endl;
							} 
							currentSlice= (((BYTE)szBuffer[0] & 0xFFFF)<<8|((BYTE)szBuffer[1] & 0xFFFF));
							TR=((currentSlice-1)/sliceNumber)+1;		// Slice 1...sliceNumber is TR 1, sliceNumber+1... is TR 2 
							if ((currentSlice-1) % sliceNumber ==0 ){	// New TR has started 
								lastTR=nowtime-starttime; 
								starttime=nowtime;
								// cout<<"Slice: "<< currentSlice<<"   "<< "TR: "<< TR<<endl;
								return (true);
							}
						}
						///-----------------------------
						// Philips: 1 serial character per volume 
						///-----------------------------
						else{ // Check for update signal for the volume
							if (dwBytesRead!=1) { 
								cout<< "Got " << dwBytesRead << " Serial Characters"<<endl;
							} 
							

							if (szBuffer[0]==counterChar)
							{	
								TR=TR++;
								lastTR=nowtime-starttime; 
								starttime=nowtime;
								return (true);
							} 
							
							else{//PRINT buffer if it doesn't equal to the counterChar*/
								std::cout<<"Character for the TR-Counter is set to: "<<counterChar
									<<" Current Buffer: "<<szBuffer[0]<<"  "<<((unsigned short)(szBuffer[0]))<<endl; 
							}
						}
					}
				}
				while (dwBytesRead == sizeof(szBuffer)-1);
			}
		}
		//------------------------------------------------------	
		// Mode over TTL counter from s626 Board 
		// -----------------------------------------------------
		else {					/// not simulated: S626 counter operation 
			newCounter=s626.getCounter(channel,board);
			if (newCounter!=oldCounter && (TR==0 || (nowtime-starttime)>50)){ 
				TR++; 
				oldCounter=newCounter; 
				lastTR=nowtime-starttime; 
				starttime=nowtime; 
				return (true);
			} 
		}  /// is Running, but no pulse has been received yet: reset the clock until first one is received 
		if (TR==0) { 
			nulltime=gTimer.getRealtime();
			starttime=gTimer.getRealtime(); 
		} 

	} return (false); 
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