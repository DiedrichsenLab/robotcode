////////////////////////////////////////////////////////////////////////////
/// Cyberglove.cpp
/// 
/// 19/01/2010 - comments to understand code and highlight the changes needed
////////////////////////////////////////////////////////////////////////////

// include corresponding header file
#include "Cyberglove.h" //19/01/2010 changed from CybergloveDR.h
#include "TextDisplay.h"
#include "S626sManager.h"
#include "Screen3D.h"
#include "Timer626.h"
#include <string> 
#include <iostream>
#include <fstream> 

using namespace std; 
extern TextDisplay tDisp;
extern Screen3D gScreen; 
extern Timer gTimer; 
char buff[50];

#define PI 3.14159265

// Finger 0:Thumb 0:MCP, 1:PIP 2:DIP 3:abduct   [0 1 2 3]
// Finger 1:Index 0:MCP, 1:PIP 2:DIP            [4 5 6]
// Finger 2:Middle 0:MCP, 1:PIP 2:DIP 3:abduct  [7 8 9 10]
// Finger 3:Ring 0:MCP, 1:PIP 2:DIP 3:abduct    [11 12 13 14] 
// Finger 4:Pinkie 0:MCP, 1:PIP 2:DIP 3:abduct  [15 16 17 18]
// Finger 5:Hand 0: Arch 1: Flex 2:Abduct       [19 20 21]



int Cyberglove::IFinger[22]={0,0,0,0,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5}; 
int  Cyberglove::IJoint[22]={0,1,2,3,0,1,2,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2}; 
//double Cyberglove::sign[22]={-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,-1,-1,1,1,1,1};  // Internally assumed sign

// Geometry of the Hand 
///< Length of the digit parts 
double Cyberglove::digitL[5][3]={{4,3.5,3},
								{5,3,2.5},
								{5,3.5,2.5},
								{4.5,3,2.5},
								{4,2.5,2}};

double Cyberglove::WfingerAbd[5][4]={{1,0,0,0},
{0,0.9,0.4,0},
{0,-0.1,0.4,0},
{0,-0.1,-0.6,0},
{0,-0.1,-0.6,-1}}; 


								     
// -----------------------------------------------------------
// Cyberglove::Cyberglove 
// Constructor 
// 19/01/2010 declare new instance of a cyberglove object
// -----------------------------------------------------------
Cyberglove::Cyberglove()  {
	
}  

// -----------------------------------------------------------
// Cyberglove::Cyberglove 
// Destructor 
// -----------------------------------------------------------
Cyberglove::~Cyberglove()  {
}  


// -----------------------------------------------------------
// Cyberglove::init
/// reads calibration from parameter file 
/// and calculates all required variables from that 
/// 19/01/2010 At moment no calibration - 
/// following code is from previous Manipulandum.cpp
// -----------------------------------------------------------
void Cyberglove::init(){
	gloveDict=NULL; 
	gloveDict = vhtIOConn::getDefault( vhtIOConn::glove );
	glove = new vhtCyberGlove(gloveDict);
	updateRaw=true;
	updateCal=true;

	for(int indx=0;indx<22;indx++ ) {
		glove->setSensorGain((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx],0.01);
		glove->setSensorOffset((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx],100); 
	}
} 

// -----------------------------------------------------------
/// Cyberglove::update
/// 19/01/2010 This should call update() from Cyberglove sdk?
/// 
// -----------------------------------------------------------
void Cyberglove::update(){
	unsigned int indx; 
	this->glove->update();
	
	for(indx=0;indx<22;indx++ ) {
		if (updateRaw) 
			raw[indx]=glove->getRawData((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx]); 
		if (updateCal) {
			//angle[indx]=glove->getAngle((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx])/PI*180;
			angle[indx]=(raw[indx]-offset[IFinger[indx]][IJoint[indx]])*gain[IFinger[indx]][IJoint[indx]]/PI*180;
		}
	}
} 

// -----------------------------------------------
/// prints the raw sensor values from the cyberglove to screen
///  
// --------------------------------------------------------------------------------

void Cyberglove::printState(){ 
	cout << "Glove: \n";
	for( int finger = 0; finger < GHM::nbrFingers; finger++ ) {
		cout << finger << " ";
		for( int joint = 0; joint < GHM::nbrJoints; joint++ ) {
			cout << glove->getRawData( (GHM::Fingers)finger, (GHM::Joints)joint) << " ";
		}
	}
	cout<<"\n ";
} 


bool Cyberglove::readCalibration(string filename) {  
	ifstream calFile; 
	double dummy; 
	calFile.open(filename.c_str(),ios::in);
	if (!calFile){ 
		cout<<(filename + " Not Found");
		return false; 
	} 

	calFile.ignore(200,'\n'); // ignore the header line 
	for( int finger = 0; finger < 6; finger++ ) {
		calFile.ignore(200,'\n'); // ignore the header line 
		for( int joint = 0; joint < 4; joint++ ) {
			if (calFile.eof() || calFile.fail() || calFile.bad())	{ 
				cout<<"failed to read cal file";
				return false; 
			}
			calFile>>dummy>>offset[finger][joint] >> gain[finger][joint]; 
			calFile.ignore(200,'\n');							///< Ignore the rest of the line 
			cout<<dummy <<" "<< offset[finger][joint] << " "<<gain[finger][joint]<<endl;
		}
	}

	
	calFile.clear();
	calFile.close();

	for(int indx=0;indx<22;indx++ ) {
		glove->setSensorGain((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx],gain[IFinger[indx]][IJoint[indx]]);
		glove->setSensorOffset((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx],offset[IFinger[indx]][IJoint[indx]]); 
	}
	return true; 
} 


bool Cyberglove::writeCalibration(string filename) {  
	ofstream calFile; 
	calFile.open(filename.c_str(),ios::out);
	if (!calFile){ 
		cout<<(filename + " could bnot be opened");
		return false; 
	} 

	calFile<<"Calibration"<<endl;
	for( int finger = 0; finger < 6; finger++ ) {
		calFile<<"Finger "<<finger << endl;
		for( int joint = 0; joint < 4; joint++ ) {
			calFile<<"\t" << joint <<"\t"<<offset[finger][joint] <<"\t"<< gain[finger][joint] <<"\t0"<<endl; 
		}
	}

	
	calFile.clear();
	calFile.close();

	return true; 
} 

// -----------------------------------------------
/// gets data from cyberglove via getData (calibrated NOT raw)
/// 
// --------------------------------------------------------------------------------

void Cyberglove::getValues(){ 
}

#define FT 2.0
#define FTT 2.0
#define FD 2.5

//////////////////////////////////////////////////////////////////////
void Cyberglove::drawFinger(int f){ 
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
	glPopMatrix();
  } 

//////////////////////////////////////////////////////////////////////
void Cyberglove::drawHand(){ 
	glPushMatrix();
	glTranslatef(position[0],position[1],position[2]);

	glRotatef(orientation[2]/PI*180,0,0,1); 
	glRotatef(orientation[1]/PI*180,0,1,0); 
	glRotatef(orientation[0]/PI*180,1,0,0); 
	glRotatef(-90,0,0,1); 

	
  	
//
//	glTranslatef(

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

	glPopMatrix();
  } 

void Cyberglove::calculateKinematics(){
	fingerAngle[0][0]=angle[0]; 
	fingerAngle[0][1]=angle[1]; 
	fingerAngle[0][2]=angle[2]; 
	fingerAngle[1][0]=angle[4]; 
	fingerAngle[1][1]=angle[5]; 
	fingerAngle[1][2]=0.2*angle[4]+0.7*angle[5]; 
	fingerAngle[2][0]=angle[7]; 
	fingerAngle[2][1]=angle[8]; 
	fingerAngle[2][2]=0.2*angle[7]+0.7*angle[8];
	fingerAngle[3][0]=angle[11]; 
	fingerAngle[3][1]=angle[12]; 
	fingerAngle[3][2]=0.2*angle[11]+0.7*angle[12];
	fingerAngle[4][0]=angle[15]; 
	fingerAngle[4][1]=angle[16]; 
	fingerAngle[4][2]=0.2*angle[15]+0.7*angle[16];

	for (int i=0;i<5;i++){ 
		fingerAbd[i]=WfingerAbd[i][0]*angle[3]+WfingerAbd[i][1]*angle[10]+WfingerAbd[i][2]*angle[14]+WfingerAbd[i][3]*angle[18];
	}
} 


#define N 11
void Cyberglove::calibrate(){
	MSG msg;
	ifstream inFile; 
	ofstream outFile; 
	updateCal=false; 
	updateRaw=true; 
	double Y[N][22];
	double X[N][22];
	double X2,XY,XM,YM,varX,Y2,varY;
	
	inFile.open("calibration.txt",ios::in);
	outFile.open("test1.txt",ios::out);
	
	if (!inFile){ 
		cout<<("calibration.txt Not Found");
	} 
	inFile.ignore(200,'\n'); // ignore the header line 

	tDisp.lock();
	outFile<<"YT1\tXT1\tYT2\tXT2\tYT3\tXT3\tYTI\tXTI\tYI1\tXI1\tYI2\tXI2\tYI3\tXI3\tYM1\tXM1\tYM2\tXM2\tYM3\tXM3\tYIM\tXIM\tYR1\tXR1\tYR2\tXR2\tYR3\tXR3\tYMR\tXMR\tYP1\tXP1\tYP2\tXP2\tYP3\tXP3\tYRP\tXRP\tYHA\tXHA\tYHF\tXHF\tYAA\tXAA"<<endl;
	for (int i=0;i<N;i++) { 
		for (int s=0;s<22;s++) { 
			inFile>>angle[s];
			Y[i][s]=angle[s]/180*PI;
		} 
		update(); 
		gTimer.countupReal(); 

		InvalidateRect(gScreen.windowHnd,NULL,TRUE);
		UpdateWindow(gScreen.windowHnd);

		tDisp.keyPressed=0;
		tDisp.print("zero"); 
		while (!tDisp.keyPressed){ 
			if (PeekMessage(&msg,NULL,0,0,PM_REMOVE)){ 
				TranslateMessage(&msg);
				DispatchMessage(&msg);				
			}
			update(); 
			gTimer.countupReal(); 
			if (gTimer[3]> 50) { 
				printRaw(1);
				InvalidateRect(tDisp.windowHnd,NULL,TRUE);
				UpdateWindow(tDisp.windowHnd);
				gTimer.reset(3);
			}
		} 

		InvalidateRect(gScreen.windowHnd,NULL,TRUE);
		UpdateWindow(gScreen.windowHnd);

		for(int j=0;j<22;j++){
			X[i][j]=raw[j];
			outFile<<Y[i][j]<<"\t"<<X[i][j]<<"\t"; 
		}

		outFile<<endl;
	} 
	inFile.close(); // ignore the header line 
	outFile.close(); // ignore the header line 
	
	
	///Calculate parameters

	for (int s=0;s<22;s++) { 
		Y2=0;
		X2=0;
		XM=0;
		XY=0;
		YM=0;
		for (int j=0;j<N;j++){
			X2+=X[j][s]*X[j][s];
			Y2+=Y[j][s]*Y[j][s];
			XM+=X[j][s];
			YM+=Y[j][s];
			XY+=X[j][s]*Y[j][s];
		} 
		XM/=N;
		YM/=N;

		cout<<"N: "<<N<<"XM: "<<XM<<"YM: "<<YM<<endl;
		varX=X2-N*XM*XM; 
		varY=Y2-N*YM*YM; 
		if (varX==0 || varY==0){
			gain[IFinger[s]][IJoint[s]]=0; 
			offset[IFinger[s]][IJoint[s]]=XM;
		} else {
			gain[IFinger[s]][IJoint[s]]=(XY-N*XM*YM)/varX;
			offset[IFinger[s]][IJoint[s]]=XM-YM/gain[IFinger[s]][IJoint[s]];
		} 
	} 	

//	for(int indx=0;indx<22;indx++ ) {
//		glove->setSensorGain((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx],gain[IFinger[indx]][IJoint[indx]]);
//		glove->setSensorOffset((GHM::Fingers)IFinger[indx],(GHM::Joints)IJoint[indx],offset[IFinger[indx]][IJoint[indx]]); 
//	}
	writeCalibration("newCalibration.txt"); 
	
	updateCal=true; 
	updateRaw=true; 
	tDisp.unlock();
}


///////////////////////////////////////////////////////////////
/// updateTextDisp: called from TextDisplay 
///////////////////////////////////////////////////////////////
void Cyberglove::printAng(int col){ 	
	sprintf(buff,"D1: %3.2f %3.2f %3.2f %3.2f ",angle[0],angle[1],angle[2],angle[3]);
	tDisp.setText(buff,1,col);
	sprintf(buff,"D2: %3.2f %3.2f %3.2f ",angle[4],angle[5],angle[6]);
	tDisp.setText(buff,2,col);
	sprintf(buff,"Abd: %3.2f ",angle[10]);
	tDisp.setText(buff,3,col);
	sprintf(buff,"D3: %3.2f %3.2f %3.2f ",angle[7],angle[8],angle[9]);
	tDisp.setText(buff,4,col);
	sprintf(buff,"Abd: %3.2f ",angle[14]);
	tDisp.setText(buff,5,col);
	sprintf(buff,"D4: %3.2f %3.2f %3.2f ",angle[11],angle[12],angle[13]);
	tDisp.setText(buff,6,col);
	sprintf(buff,"Abd: %3.2f ",angle[18]);
	tDisp.setText(buff,7,col);
	sprintf(buff,"D5: %3.2f %3.2f %3.2f ",angle[15],angle[16],angle[17]);
	tDisp.setText(buff,8,col);
	sprintf(buff,"Hand: Ar:%3.2f Flex:%3.2f Abduct:%3.2f",angle[19],angle[20],angle[21]);
	tDisp.setText(buff,9,col);
}


void Cyberglove::printRaw(int col){		
	sprintf(buff,"D1: %3.2f %3.2f %3.2f %3.2f ",raw[0],raw[1],raw[2],raw[3]);
	tDisp.setText(buff,1,col);
	sprintf(buff,"D2: %3.2f %3.2f %3.2f ",raw[4],raw[5],raw[6]);
	tDisp.setText(buff,2,col);
	sprintf(buff,"Abd: %3.2f ",raw[10]);
	tDisp.setText(buff,3,col);
	sprintf(buff,"D3: %3.2f %3.2f %3.2f ",raw[7],raw[8],raw[9]);
	tDisp.setText(buff,4,col);
	sprintf(buff,"Abd: %3.2f ",raw[14]);
	tDisp.setText(buff,5,col);
	sprintf(buff,"D4: %3.2f %3.2f %3.2f ",raw[11],raw[12],raw[13]);
	tDisp.setText(buff,6,col);
	sprintf(buff,"Abd: %3.2f ",raw[18]);
	tDisp.setText(buff,7,col);
	sprintf(buff,"D5: %3.2f %3.2f %3.2f ",raw[15],raw[16],raw[17]);
	tDisp.setText(buff,8,col);
	sprintf(buff,"Hand: %3.2f %3.2f %3.3f",raw[19],raw[20],raw[21]);
	tDisp.setText(buff,9,col);
} 