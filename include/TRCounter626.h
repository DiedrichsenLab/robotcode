//////////////////////////////////////////////////////////////
///   
///  TRCounter626.h
///  Uses Counter 0B from sensoray board to count TTL pulses from 
///  the scanner 
///   can be set to simulation based on the s626 clock (1A+1B) 
/// 
///   (c) 2007 joern Diedrichsen (j.diedrichsen@bangor.ac.uk) 
///		  2013 - This version matches the 2013 source file
/// Mode: 
///			1: Serial character - receiving the slice number with every slice (FIL + BUCNI) 
///         2: Serial character - receiving a single character ('T') for every TR (Philips) 
///			3: TTL counter - Receiving a TTL pulse that is processed by the s626 card with every TR 
///         4: TTL counter - Receiving a TTL pulse with every slice (not implemented yet) 
///	 Jan 2009 Extend to read the update signal from a serial port using the Serial.cpp of Ramon de Klein
//////////////////////////////////////////////////////////////
#ifndef TRCOUNTER_H 
#define TRCOUNTER_H

#include <tchar.h>
#include <windows.h>
#include "Serial.h"
#include <bitset>

class TRCounter { 
public: 
	TRCounter();							///< Constructor
	void init1(LPCTSTR comport= _T("COM1"),int baud= 9600, int sliceNumber= 1);							///< A double character per slice (slice number)  
	void init2(LPCTSTR comport= _T("COM1"),int baud= 9600, char trchar='T', int sliceNumber= 1);		///< A single character per TR 
	void init3(int channel=3,int board=0,int sliceNumber=1);											///< A single TTL pulse per TR 
	void init4(int channel=3,int board=0,int sliceNumber=1);											///< A single TTL pulse per slice 
	void init(int channel=3,int board=0);																///< Backwards compatibility: Initialize the TR counter with defaults for IOBoard 
	void initSerial(LPCTSTR comport= _T("COM1"),int baud= 9600, char trchar='T', int sliceNumber= 0)   ///< Backwards compatibility: Initialize the TR counter for the serial Port 
		{init1(comport,baud,sliceNumber);}
	void openSerial(LPCTSTR comport,int baud); ///< open serial port 
	bool update();							///< Checks counter and sets time; 
	void reset();							///< reset counter to zero 
	void start();							///< start the counter 
	void stop(){isRunning=false;};			///< stops the counter 
	void simulate(double lengthTR);			///< starts or stops simulating TRs 
	int readTR(){return TR;}				///< Gives the TR-count 
	int readSlice(){return currentSlice;}	///< Gives the TR-count 
	double readTime(){return nowtime-starttime;}///< gives timing count in current TR based on 
	double readTotTime(){return nowtime-nulltime;}///< gives timing since start of Run (first TR)
private:
	int mode;								///< Mode of the TR counter (1-4)
	int board;								///< Board for TR counter (default 0) 
	int channel;							///< Channel for TR counter (default CNTR_0B =3 ) 
	long double nulltime;					///< Time of last call to reset 
	long double nowtime;					///< Current time based on Update() 
	long double starttime;					///< Time of detection of current TR; 	
	long double startSliceTime;             ///< Time of detection of initial slice 
	int TR;									///< Number of current TR 
	bool isRunning;							///< is the TRcounter currently running? 
	bool isSimulated;						///< Are TR's simulated or measured? 
	int newCounter;							///< State of the counter 
	int oldCounter;							///< old state of the counter 
	double lengthTR;						///< (Expected) Length of TR
	unsigned int currentSlice;				///< Slice number 
	int sliceNumber;						///< Do we have to count slices? (default = 0 => do not count slices but volumes)

	CSerial nSerial;						///< Serial Port control object 
	bool isSerial;							///< Current state for TR counter (default = 0 => not Serial) 
	LONG lLastError;						///< Errorvariable for serialport
	HANDLE hevtOverlapped;					///< Handle variable for overlapp
	DWORD dwBytesRead;						///< number of bytes read 
	CSerial::EEvent eEvent;	
	char counterChar;						///< character that signals to raise the TR-counter
	char szBuffer[5];						///< serial buffer 	
};

#endif 