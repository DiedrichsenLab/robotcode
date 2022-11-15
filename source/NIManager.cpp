// -----------------------------------------------------------------------
// Manages the resource for one or more National instruments card
//
// Original by Peter Haas, Tobias Wiestler, Joern Diedrichsen, Naveed Ejaz, 2012
// -----------------------------------------------------------------------

#include "NIManager.h" 
#include <windows.h>
//#include <conio.h>
#include <stdio.h>
#include <iostream>

using namespace std; 

// --------------------------------------------------------------------------------
// The callback to be implemented by the user depending on the task
// -----------state---------------------------------------------------------------------
void (* NIManager::updateCallback)(void);

// --------------------------------------------------------------------------------
// Constructor 
// --------------------------------------------------------------------------------
NIManager::NIManager()
{
    int b,c;
    for (b = 0; b < NUMBOARDS; b++)
    {
        hTask[b] = 0;
        
        for (c = 0; c < NUMCHANNELS; c++)
        {
            isUsedCh[b][c] = false;
        }
    }
}

// -----------------------------------------------------------------------
// Destructor: Free's resources again
// -----------------------------------------------------------------------
NIManager::~NIManager()
{
    int b,c;
    for (b = 0; b < numBoards; b++)
    {
        if ( hTask[b] != 0 )
        {
            DAQmxBaseStopTask(hTask[b]);
            DAQmxBaseClearTask(hTask[b]);
        }
    }
}

// -----------------------------------------------------------------------
// NIManager::init
/// Opens the NI board corresponding to that specific id
// -----------------------------------------------------------------------
void NIManager::init(string paramfile)
{
	int i;
	string s;
	ifstream inputFile(paramfile.c_str(),ios::in);
	
    // reading in settings from the parameter file
    if(inputFile ==0)
    {
		cout<<"NIManager.init: Parameter file could not be found\n";
		exit(-1);
	}
    else
    {
		inputFile >> numBoards;
		getline(inputFile,s);
        
		for (i = 0; i < numBoards; i++)
        {
			inputFile >> boardIDs[i];
			getline(inputFile,s);
		}
	}
    
	/// create handles for the boards
    for (i = 0; i < numBoards; i++)
    {
        ErrCode = DAQmxBaseCreateTask("",&taskHandle[i]);
        		
		if ( ErrCode != 0)
        {
	        cout << ( "NIManager.init: One or more errors detected:" ) << endl;

            char ErrMessage[2048] = {'\0'};
            DAQmxBaseGetExtendedErrorInfo(ErrMessage,ErrCode);
            cout << "Error " << ErrCode << ": " << ErrMessage << endl;
            
			exit(-1);
		}
        else
        {
			cout << "Board " << i << ": successfully initiated." << endl;
		}
	}
}

// -----------------------------------------------------------------------
// initialize the interrupt generation proceedure using the NI internal clock
// -----------------------------------------------------------------------
void NIManager::initInterrupt(void (*fcn)(void),int updateR)
{
} 

// -----------------------------------------------------------------------
// stop Interrupt 
// -----------------------------------------------------------------------
void NIManager::stopInterrupt()
{
} 

