// ------------------------------------------------------------------
// NIManager:
// ---------
// Manages the resources of one or more national instruments board
// Does Allocation and low level interrupt handling, Analog input-output
// and interrupt generation/management using the national instruments
// board (usb)
// Class should provide a function list exactly the same as the S626sManager
// class so that we can use whichever manager given the experimental setup
// ------------------------------------------------------------------

#ifndef NIManager_H_
#define NIManager_H_

#define NUMBOARDS 2												///< Maximal numbers of boards
#define NUMCHANNELS 16                                          //< Maximal number of channels
#include "NIDAQmxBase.h"
#include <string> 
#include <iostream>
#include <fstream>

using namespace std;
///////////////////////////////////////////////////////////////////////
/// \brief Handles all communication with the National Instrument boards
/// 
/// Manages the resources of one or more NI board. You should never
/// have to talk to the national instruments NI-DAQmx library functions
/// directly. The class does allocation and low level interrupt handling
/// so that multiple devices cannot conflict in their resource allocation. 
///  
///////////////////////////////////////////////////////////////////////
class NIManager {
public: 
	NIManager();
	~NIManager();
    
	void init(string configfile);                                   ///< Initialize the boards under a certain config file
	void initInterrupt(void (*fcn)(void),int updateR);              ///< Initializes interrupt and starts
	void stopInterrupt();                                           ///< stop the interrupt routine
	
private:
	int updateRate;                                                 ///< update rate in ms
	int ErrCode;                                                    //< value of zero indicates success, positive value indicates an error
	static void (*updateCallback)(void) ;                           //< callback function to be implemented by the user

	// Board stuff
	int numBoards;                                                  ///< How many NI boards are configured on the machine
    string boardIDs[NUMBOARDS];                                                ///< IDs used to identify the boards on the machine
    TaskHandle hBoard[NUMBOARDS];                                   ///<  handler for the NI board

	// Allocate channels
	bool isUsedCh[NUMBOARDS][NUMCHANNELS];                                   //< Specifies which channels are used
};


#endif 