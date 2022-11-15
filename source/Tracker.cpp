////////////////////////////////////////////////////////////////////////////
/// Tracker.cpp
/// 
/// 19/01/2010 - comments to understand code and highlight the changes needed
////////////////////////////////////////////////////////////////////////////

// include corresponding header file
#include "Tracker.h" //19/01/2010 changed from TrackerDR.h
#include "TextDisplay.h"
#include "S626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 

using namespace std; 
extern TextDisplay tDisp;
extern char buffer[100]; 

// -----------------------------------------------------------
// Tracker::Tracker 
// Constructor 
// 19/01/2010 declare new instance of a Tracker object
// -----------------------------------------------------------
Tracker::Tracker()  {
	
}  

// -----------------------------------------------------------
// Tracker::Tracker 
// Destructor 
// -----------------------------------------------------------
Tracker::~Tracker()  {
}  


// -----------------------------------------------------------
// Tracker::init
/// reads calibration from parameter file 
/// and calculates all required variables from that 
/// 19/01/2010 At moment no calibration - 
// -----------------------------------------------------------
void Tracker::init()
{
	trackerDict=NULL;
	trackerDict = vhtIOConn::getDefault( vhtIOConn::tracker );
	tracker = new vhtTracker(trackerDict);

	firstReceiver = tracker->getLogicalDevice(0);
	
			
	secondReceiver = tracker->getLogicalDevice(1);
	
	tracker->getLogicalDevice(0)->getTransform( &trackerXForm1 );
	tracker->getLogicalDevice(0)->getTransform( &trackerXForm2 );
} 

// -----------------------------------------------------------
/// Tracker::update
/// 19/01/2010 This should call update() from Tracker sdk?
/// 
// -----------------------------------------------------------
void Tracker::update(){

	firstReceiver->update(); 
	secondReceiver->update(); 

	receiverPos[0]=Vector3D(firstReceiver->getRawData((vht6DofDevice::Freedom)0),firstReceiver->getRawData((vht6DofDevice::Freedom)1),firstReceiver->getRawData((vht6DofDevice::Freedom)2));
	
	receiverPos[1]=Vector3D(secondReceiver->getRawData((vht6DofDevice::Freedom)0),secondReceiver->getRawData((vht6DofDevice::Freedom)1),secondReceiver->getRawData((vht6DofDevice::Freedom)2));

	receiverOr[0]=Vector3D(firstReceiver->getRawData((vht6DofDevice::Freedom)3),firstReceiver->getRawData((vht6DofDevice::Freedom)4),firstReceiver->getRawData((vht6DofDevice::Freedom)5));

	receiverOr[1]=Vector3D(secondReceiver->getRawData((vht6DofDevice::Freedom)3),secondReceiver->getRawData((vht6DofDevice::Freedom)4),secondReceiver->getRawData((vht6DofDevice::Freedom)5));
	
} 

// -----------------------------------------------
/// prints the state of the robot to the Text display
/// 19/01/2010 need this to output the joint angles or raw sensor values
// --------------------------------------------------------------------------------

void Tracker::printState(int r,int row,int col){ 
	sprintf(buffer,"%2.2f %2.2f %2.2f",receiverPos[r][0],receiverPos[r][1],receiverPos[r][2]); 
	tDisp.setText(buffer,row,col); 
	sprintf(buffer,"%2.2f %2.2f %2.2f",receiverOr[r][0],receiverOr[r][1],receiverOr[r][2]); 
	tDisp.setText(buffer,row+1,col); 
} 


// -----------------------------------------------
/// prints the state of the robot to the Text display
/// 19/01/2010 need this to output the joint angles or raw sensor values
// --------------------------------------------------------------------------------

void Tracker::getVals(){ 
	receiverPos[0]=Vector3D(firstReceiver->getRawData((vht6DofDevice::Freedom)0),firstReceiver->getRawData((vht6DofDevice::Freedom)1),firstReceiver->getRawData((vht6DofDevice::Freedom)2));
	
	receiverPos[1]=Vector3D(secondReceiver->getRawData((vht6DofDevice::Freedom)0),secondReceiver->getRawData((vht6DofDevice::Freedom)1),secondReceiver->getRawData((vht6DofDevice::Freedom)2));

	receiverOr[0]=Vector3D(firstReceiver->getRawData((vht6DofDevice::Freedom)3),firstReceiver->getRawData((vht6DofDevice::Freedom)4),firstReceiver->getRawData((vht6DofDevice::Freedom)5));

	receiverOr[1]=Vector3D(secondReceiver->getRawData((vht6DofDevice::Freedom)3),secondReceiver->getRawData((vht6DofDevice::Freedom)4),secondReceiver->getRawData((vht6DofDevice::Freedom)5));
	
}


// -----------------------------------------------
/// gets the quaternion	
// --------------------------------------------------------------------------------

void Tracker::getQuart(){ 

trackerXForm1.getRotation( quartern[0] );
trackerXForm2.getRotation( quartern[1] );
}