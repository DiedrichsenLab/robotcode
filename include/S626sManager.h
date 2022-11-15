// ------------------------------------------------------------------
// S626sManager: 
// Manages the resources of one or more S626 board 
// Does Allocation and low level interrupt handling 
// Analog input output 
// Allocates functions to avoid conflict between different devices 
// Using the s626 card 
// ------------------------------------------------------------------

#ifndef S626sManager_H_
#define S626sManager_H_

#define NUMBOARDS 2												///<Maximal numbers of boards 
#include "WIN626.h" 
#include <string> 
#include <iostream>
#include <fstream>

using namespace std;
///////////////////////////////////////////////////////////////////////
/// \brief Handles all communication with the s626 boards
/// 
/// Manages the resources of one or more S626 board. You should never 
/// have to talk to the dll directly. The class does allocation and low level interrupt handling 
/// so that multiple devices cannot conflict in their resource allocation. 
///  
///////////////////////////////////////////////////////////////////////
class S626sManager { 
public: 
	S626sManager(); 
	~S626sManager();
	friend void AppISR();										 ///< Primary callback function which call updateCallback
	void init(string configfile);								 ///< Initialize the boards under a certain config file 
	void initCounter(int c,int board=0);						 ///< Initializes c as Quad counter with Index 
	void initTTLCounter(int c,int board=0);					     ///< Initializes c as a single phase TTL counter 
	void initTimer(int c,int board=0);							 ///< Initializes c as timer
	void initOverflow(int c,int board=0);						 ///< Initializes c as overflow timer 
	void initInterrupt(void (*fcn)(void),double updateR);			 ///< Initializes interrupt and starts
	void stopInterrupt();							 ///< stop the interrupt routine 
	
	long getCounter(int c,int board=0);							 ///< Get result from couters 
	void resetCounter(int c,int board=0);						 ///< resets the counter to preload register (0) 
	double getTime(int low=CNTR_1A, int high=CNTR_1B);			 ///< Get absolute time in ms (counter 1A/1B) 
	void resetTime(int low=CNTR_1A, int high=CNTR_1B);		     ///< reset time to zero (counter 1A/1B) 
	int getErrorState() {return ErrCode;}						 ///< get error state 
	double getUpdateRate(){return updateRate;}					 ///< returns update rate in ms 
	double getAD(int c,int board=0);										 ///< returns value for AD in voltage
	int registerAD(int c,int range,int board=0);				 ///< puts an AD-channel on the poll-list 
	void updateAD(int board=0);									 ///< pulls all registered AD channels 
	void outDA(int channel,double volts,int board=0);			 ///< Put voltage on outDA 
	void outDIO(int channel,short state,int board=0);				 ///< Put state(1/0) on DIO
	unsigned int S626sManager::readDIO(int channel, int board);			///< read DIO
private: 
	double updateRate;												 ///< update rate in ms
	int ErrCode;
	static void (*updateCallback)(void) ;

	// Board stuff
	int numBoards;												///< How many s626 boards are registered? 
	int pcibus[NUMBOARDS];										///< PCI Bus of the board 
	int slot[NUMBOARDS];										///< slot of card 
	int interruptBoard;											///< Board that holds intterupt timer (-1 for none) 
	int timerBoard;												///< Board that holds timer (-1 for none) 
	int timerChannel;											///< Channels occupied by timer (e.g.: 1: 1A, 3:0B) 
	int timerOverflow;											///< Channel for timer overflow 
	// AD channel stuff
	unsigned char poll_list[NUMBOARDS][16]; // List of items to be digitized.
	short databuf[NUMBOARDS][16]; // Buffer to receive digitized data.
	int range[NUMBOARDS][16];		// Voltage range (5 or 10);
	int numAD[NUMBOARDS];
	
	// Allocate channels 
	bool isUsedAD[NUMBOARDS][16]; 
	bool isUsedDA[NUMBOARDS][4];
	bool isUsedCounter[NUMBOARDS][6]; 
};


#endif 