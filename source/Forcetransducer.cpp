// -------------------------------------------------------------------------
// Forcetransducer class 
// 
// under a s626 iocard 
// 
// --------------------------------------------------------------------------

// include corresponding header file
#include "Forcetransducer.h"
#include "S626sManager.h"
#include <string> 
#include <iostream>
#include <fstream> 
#include <stdio.h>

using namespace std; 
extern S626sManager s626;


// defining constants for Forcetransducer


// -----------------------------------------------------------
// Forcetransducer::Forcetransducer 
// Constructor 
// -----------------------------------------------------------
Forcetransducer::Forcetransducer()  {
	baseline=0;
	scale=1;
}  

// -----------------------------------------------------------
// Forcetransducer::Forcetransducer 
// Destructor 
// -----------------------------------------------------------
Forcetransducer::~Forcetransducer()  {
}  


// -----------------------------------------------------------
// Forcetransducer::Forcetransducer 
// Init: 
// -----------------------------------------------------------
void Forcetransducer::setScale(double s, double o){ 
	scale =s; 
	baseline=o;
}

// -----------------------------------------------------------
// Forcetransducer::Forcetransducer 
// Init: 
// -----------------------------------------------------------
bool Forcetransducer::init(int ch,int b)
{
	channel=s626.registerAD(ch,10.0,b);
	board = b; 
	cout<<channel<<endl;
	if (channel>-1) 
		return true; 
	else 
		return false; 
} 


// -----------------------------------------------------------
// Forcetransducer::update: 
// 
// -----------------------------------------------------------
void Forcetransducer::update(){
	// update the force 
	//int i;
	volts=s626.getAD(channel,board);
	//for (i=NUMFORCEREADINGS-2;i>=0;i--) { 
	//	force[i+1]=force[i];
	//} 
	force[0]=(volts-baseline)*scale; 
} 


// -----------------------------------------------------------
// zeroForce 
// 
// -----------------------------------------------------------
void Forcetransducer::zeroForce(){
	// update the force 
	baseline=volts;
} 




