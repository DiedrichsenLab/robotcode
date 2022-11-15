////////////////////////////////////////////////////////////////////////////
/// fMRICyberglove.cpp
/// 
/// 8/11/2012 - comments to understand code and highlight the changes needed
////////////////////////////////////////////////////////////////////////////

// include corresponding header file
#include "fMRICyberglove.h" 
#include "TextDisplay.h"
#include "S626sManager.h"
#include "Screen.h"
#include "Timer626.h"
#include <string> 
#include <iostream>
#include <iomanip>
#include <fstream> 

using namespace std; 
extern TextDisplay tDisp;
extern Screen gScreen; 
extern Timer gTimer; 
char buff[50];

#define PI 3.14159265
#define FT 2.0			// used for drawing
#define FTT 2.0			// used for drawing
#define FD 2.5			// used for drawing

// Sensor configuration for the 14 sensors on the fMRI cyberglove
int  fMRICyberglove::SIndex[NSEN]={FD_THUMBNEAR,FD_THUMBFAR,FD_THUMBINDEX,
								   FD_INDEXNEAR,FD_INDEXFAR,FD_INDEXMIDDLE,
								   FD_MIDDLENEAR,FD_MIDDLEFAR,FD_MIDDLERING,
								   FD_RINGNEAR,FD_RINGFAR,FD_RINGLITTLE,
								   FD_LITTLENEAR,FD_LITTLEFAR};

// Geometry of the Hand 
///< Length of the digit parts 
double fMRICyberglove::digitL[5][3]={{4,3.5,3},
								{5,3,2.5},
								{5,3.5,2.5},
								{4.5,3,2.5},
								{4,2.5,2}};

// -- Unclear why these weights exist -- from non-fMRI cyberglove
// -- Used in the calculateKinematics function
double fMRICyberglove::WfingerAbd[5][4]={{1,0,0,0},
{0,0.9,0.4,0},
{0,-0.1,0.4,0},
{0,-0.1,-0.6,0},
{0,-0.1,-0.6,-1}}; 

// -----------------------------------------------------------
// fMRICyberglove::fMRICyberglove 
// Constructor 
// 8/11/2012 declare new instance of an fMRI cyberglove object
// -----------------------------------------------------------
fMRICyberglove::fMRICyberglove()  
{
	gain = 30;
}  

// -----------------------------------------------------------
// fMRICyberglove::fMRICyberglove 
// Constructor 
// 29/11/2012 overloaded constructor which applies a static gain to sensor readings
// -----------------------------------------------------------
fMRICyberglove::fMRICyberglove(int sGain)  
{
	gain = sGain;
} 

// -----------------------------------------------------------
// fMRICyberglove::fMRICyberglove 
// Destructor 
// -----------------------------------------------------------
fMRICyberglove::~fMRICyberglove()  
{
	if (hGlove != NULL)
	{	
		cout << "Closing glove" << endl;
		fdClose(hGlove);
	}
}  

// -----------------------------------------------------------
// Cyberglove::init
/// reads calibration from parameter file 
/// and calculates all required variables from that 
/// 8/11/2012 At moment no calibration - 
// -----------------------------------------------------------
void fMRICyberglove::init()
{
	// Attempting to identify and open the glove

	char gloveID[5] = "USB0";	// connects to the first USB fMRI glove that is found
								// the glove we are using is the Ultra 14 channel - right handed glove

	hGlove = fdOpen(gloveID); // connects to the glove (Ultra 14 channel - Right handed)
	if (hGlove)
	{
		int gloveType = FD_GLOVENONE;
		char gloveName[20] = "?";
		gloveType = fdGetGloveType(hGlove);
		
		switch (gloveType)
		{
			case FD_GLOVENONE:		strcpy(gloveName,"None"); break;
			case FD_GLOVE7:			strcpy(gloveName,"Glove7"); break;
			case FD_GLOVE7W:		strcpy(gloveName,"Glove7W"); break;
			case FD_GLOVE16:		strcpy(gloveName,"Glove16"); break;
			case FD_GLOVE16W:		strcpy(gloveName,"Glove16W"); break;
			case FD_GLOVE5U:		strcpy(gloveName,"DG5 Ultra serial"); break;
			case FD_GLOVE5UW:		strcpy(gloveName,"DG5 Ultra serial, wireless"); break;
			case FD_GLOVE5U_USB:	strcpy(gloveName,"DG5 Ultra USB"); break;
			case FD_GLOVE14U:		strcpy(gloveName,"DG14 Ultra serial"); break;
			case FD_GLOVE14UW:		strcpy(gloveName,"DG14 Ultra serial, wireless"); break;
			case FD_GLOVE14U_USB:	strcpy(gloveName,"DG14 Ultra USB"); break;
		}
		cout << "Glove type: " << gloveName;
		
		if ( fdGetGloveHand(hGlove) == FD_HAND_RIGHT )
			cout << " " << "(Right handed)" << endl;
		else
			cout << " " << "(Left handed)" << endl;
	}

	// turning on flags which enable unpdates
	updateRaw = true;
	updateCal = true;
	updateScaled = true;
} 

// -----------------------------------------------------------
/// Cyberglove::update
/// 8/11/2012 Updates the raw and calculated values from the sensors
/// 
// -----------------------------------------------------------
void fMRICyberglove::update()
{
	// check to see if new data has been read by the glove since the last time function was called
	if ( fdNewData(hGlove) )
	{
		for(int indx = 0; indx < NSEN; indx++)	
		{
			if (updateRaw)
				raw[indx] = fdGetSensorRaw(hGlove,SIndex[indx]);
			if (updateCal)
				angle[indx] = gain*fdGetSensorScaled(hGlove,SIndex[indx]);		// code stub to be removed
			if (updateScaled)
				scaled[indx] = fdGetSensorScaled(hGlove,SIndex[indx]);
		}
	}
} 

// -----------------------------------------------
/// prints the raw sensor values from the cyberglove to screen
// --------------------------------------------------------------------------------
void fMRICyberglove::printStateRaw()
{ 
	for( int finger = 0; finger < NSEN; finger++ ) 
	{
		cout << fixed << setprecision(1) << raw[finger] << " ";
	}
	cout << endl;
}

// -----------------------------------------------
/// prints the scaled sensor values from the cyberglove to screen
// --------------------------------------------------------------------------------
void fMRICyberglove::printStateScaled()
{ 
	for( int finger = 0; finger < NSEN; finger++ ) 
	{
		cout << fixed << setprecision(1) << scaled[finger] << " ";
	}
	cout << endl;
} 


bool fMRICyberglove::readCalibration(string filename) 
{
	// implement calibration read here
	return false;
} 


bool fMRICyberglove::writeCalibration(string filename) 
{  
	// implement calibration write here
	return false; 
} 



///////////////////////////////////////////////////////////////
/// updateTextDisp: called from TextDisplay 
///////////////////////////////////////////////////////////////
void fMRICyberglove::printAng(int col)
{ 	
	sprintf(buff,"T: %3.2f %3.2f",angle[0],angle[1]);
	tDisp.setText(buff,1,col);
	sprintf(buff,"I: %3.2f %3.2f",angle[3],angle[4]);
	tDisp.setText(buff,2,col);
	sprintf(buff,"M: %3.2f %3.2f",angle[6],angle[7]);
	tDisp.setText(buff,3,col);
	sprintf(buff,"R: %3.2f %3.2f",angle[9],angle[10]);
	tDisp.setText(buff,4,col);
	sprintf(buff,"P: %3.2f %3.2f",angle[12],angle[13]);
	tDisp.setText(buff,5,col);
	sprintf(buff,"Abd: %3.2f %3.2f %3.2f %3.2f",angle[2],angle[5],angle[8],angle[11]);
	tDisp.setText(buff,6,col);
}


void fMRICyberglove::printRaw(int col)
{		
	sprintf(buff,"T: %3.2f %3.2f",raw[0],raw[1]);
	tDisp.setText(buff,1,col);
	sprintf(buff,"I: %3.2f %3.2f",raw[3],raw[4]);
	tDisp.setText(buff,2,col);
	sprintf(buff,"M: %3.2f %3.2f",raw[6],raw[7]);
	tDisp.setText(buff,3,col);
	sprintf(buff,"R: %3.2f %3.2f",raw[9],raw[10]);
	tDisp.setText(buff,4,col);
	sprintf(buff,"P: %3.2f %3.2f",raw[12],raw[13]);
	tDisp.setText(buff,5,col);
	sprintf(buff,"Abd: %3.2f %3.2f %3.2f %3.2f",raw[2],raw[5],raw[8],raw[11]);
	tDisp.setText(buff,6,col);
}

void fMRICyberglove::printScaled(int col){		
	sprintf(buff,"T: %3.2f %3.2f",scaled[0],scaled[1]);
	tDisp.setText(buff,1,col);
	sprintf(buff,"I: %3.2f %3.2f",scaled[3],scaled[4]);
	tDisp.setText(buff,2,col);
	sprintf(buff,"M: %3.2f %3.2f",scaled[6],scaled[7]);
	tDisp.setText(buff,3,col);
	sprintf(buff,"R: %3.2f %3.2f",scaled[9],scaled[10]);
	tDisp.setText(buff,4,col);
	sprintf(buff,"P: %3.2f %3.2f",scaled[12],scaled[13]);
	tDisp.setText(buff,5,col);
	sprintf(buff,"Abd: %3.2f %3.2f %3.2f %3.2f",scaled[2],scaled[5],scaled[8],scaled[11]);
	tDisp.setText(buff,6,col);
} 

//////////////////////////////////////////////////////////////////////
// Calculate the hand kinematics from the joint angles
//////////////////////////////////////////////////////////////////////
void fMRICyberglove::calculateKinematics()
{
	// maps the sensor values on the hand kinematics (all DIPs set to zero)

	fingerAngle[0][0]=angle[0];							// Thumb - MCP
	fingerAngle[0][1]=angle[1];							// Thumb - PIP
	fingerAngle[0][2]=0;								// Thumb - DIP
	fingerAngle[1][0]=angle[3];							// Index - MCP
	fingerAngle[1][1]=angle[4];							// Index - PIP
	fingerAngle[1][2]=0;								// Index - DIP
	fingerAngle[2][0]=angle[6];							// Middle - MCP							
	fingerAngle[2][1]=angle[7];							// Middle - PIP 
	fingerAngle[2][2]=0;								// Middle - DIP
	fingerAngle[3][0]=angle[9];							// Ring - MCP
	fingerAngle[3][1]=angle[10];						// Ring - PIP	
	fingerAngle[3][2]=0;								// Ring - DIP
	fingerAngle[4][0]=angle[12];						// Pinky - MCP 
	fingerAngle[4][1]=angle[13];						// Pinky - PIP
	fingerAngle[4][2]=0;								// Pinky - DIP

	for (int i=0;i<5;i++)
	{ 
		fingerAbd[i]=WfingerAbd[i][0]*angle[2]			// Thumb/Index - Abd 
					+WfingerAbd[i][1]*angle[5]			// Index/Middle - Abd
					+WfingerAbd[i][2]*angle[8]			// Middle/Ring - Abd
					+WfingerAbd[i][3]*angle[11];		// Ring/Pinky - Abd
	}
}

//////////////////////////////////////////////////////////////////////
// Calibrate the hand with the glove to obtain the correct estimate
// of the joint angles from the raw sensor values
//////////////////////////////////////////////////////////////////////
void fMRICyberglove::calibrate()
{
}

// --------------------------------------------------------------------------------
/// gets data from cyberglove via getData (calibrated NOT raw)
// --------------------------------------------------------------------------------
void fMRICyberglove::getValues()
{ 
}


//////////////////////////////////////////////////////////////////////
void fMRICyberglove::drawFinger(int f)
{ /*
	glPushMatrix();
	glRotatef(-fingerAbd[f],0.0,0.0,1.0);
	glRotatef (-fingerAngle[f][0], 1.0, 0.0, 0.0);
	glTranslatef (0.0,digitL[f][0]/2.0, 0.0);
	gScreen.drawBox(Vector3D(FT,digitL[f][0],FT)); 
	glTranslatef (0.0,digitL[f][0]/2+0.2, 0.0);
	glRotatef (-fingerAngle[f][1], 1.0, 0.0, 0.0);
	glTranslatef (0.0,digitL[f][1]/2, 0.0);
	gScreen.drawBox(Vector3D(FT,digitL[f][1],FT));
	glTranslatef (0.0,digitL[f][1]/2+0.2, 0.0);
	glRotatef (-fingerAngle[f][2], FT, 0.0, 0.0);
	glTranslatef (0.0,digitL[f][2]/2, 0.0);
	gScreen.drawBox(Vector3D(FT,digitL[f][2],FT));       
	glPopMatrix(); */
} 

//////////////////////////////////////////////////////////////////////
// Draw the hand on the screen
//////////////////////////////////////////////////////////////////////
void fMRICyberglove::drawHand()
{ /*
	glPushMatrix();
	glTranslatef(position[0],position[1],position[2]);

	glRotatef(orientation[2]/PI*180,0,0,1); 
	glRotatef(orientation[1]/PI*180,0,1,0); 
	glRotatef(orientation[0]/PI*180,1,0,0); 
	glRotatef(-90,0,0,1); 

	glPushMatrix();
	glTranslatef(-1.5*FD,10.0,0.0);
	drawFinger(1);
	glTranslatef(FD,0,0);
	drawFinger(2);
	glTranslatef(FD,0,0);
	drawFinger(3);
	glTranslatef(FD,0,0);
	drawFinger(4);
	glPopMatrix();
	
	glPushMatrix();
	glTranslatef(0.0,+5.0,0.0);
	gScreen.drawBox(Vector3D(3*FD+FT,10,FT));
	glPopMatrix();
	
	// Do Thumb 
	glPushMatrix();
	glTranslatef(-1.5*FD,3.0,0.0);
	glRotatef(-fingerAngle[0][0],0,1.0,0);
	glRotatef(60-fingerAbd[0],0.0,0.0,1.0);
	glRotatef(-40,0.0,1.0,0.0);
	glTranslatef (0.0,digitL[0][0]/2.0, 0.0);
	gScreen.drawBox(Vector3D(FTT,digitL[0][0],FTT)); 
	glTranslatef (0.0,digitL[0][0]/2+0.2, 0.0);
    glRotatef (fingerAngle[0][1], 0.0, 0.0, 1.0);
	glTranslatef (0.0,digitL[0][1]/2, 0.0);
    gScreen.drawBox(Vector3D(FTT,digitL[0][1],FTT));
    glTranslatef (0.0,digitL[0][1]/2+0.2, 0.0);
    glRotatef (fingerAngle[0][2], 0.0, 0.0, 1.0);
    glTranslatef (0.0,digitL[0][2]/2, 0.0);
	gScreen.drawBox(Vector3D(FTT,digitL[0][2],FTT));       	
	glPopMatrix();

	glPopMatrix(); */
} 