////////////////////////////////////////////////////////////////////////////
// GripBox.cpp
// 
// written for a 5-finger Grip device with USB servo motors and force
// tranduscers under a s626 iocard.
//     
//  Designed, built, & originally coded by Carlos Hernandez 2016
//  Further code updates by Spencer Arbuckle 2016
////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "GripBox.h"
#include "S626sManager.h"

using namespace std; 
extern S626sManager s626;

//////////////////////////////////////////////////////////////
// Constructor 
//////////////////////////////////////////////////////////////
GripBox::GripBox(){
	channelIdx[0]=6;  // Recode the channels to more intuitive index values
	channelIdx[1]=9;  // Ugly but works
	channelIdx[2]=8;  // Channels 1:5 correspond to force transducer at front of each key (1=thumb, 5=little)
	channelIdx[3]=1;  // Channels 6:10 are force transducers further from palm of thumb (6) to little finger (10)
	channelIdx[4]=3;
	channelIdx[5]=5;
	channelIdx[6]=0;
	channelIdx[7]=7;
	channelIdx[8]=2;
	channelIdx[9]=4;			
	for (int i=0; i<NUMSENSORS; i++){	// initialize robust volt2force scaling param
			scale[i]= 502.3/1000*9.81;	// average g to volts scale based on calibration of 17/6/2010 JD
			}							// Scale param updated with calibration file (if provided on initialization)
}

//////////////////////////////////////////////////////////////
// Destructor - close communication port 
//////////////////////////////////////////////////////////////
GripBox::~GripBox()  {

	//Check if we are connected before trying to disconnect
    if(this->connected)
    {
		//We're no longer connected
        this->connected = false;
        //Close the serial handler
        CloseHandle(this->port);
    }
}  

//////////////////////////////////////////////////////////////
// Initialize communication and read calibration file
// portName:  port to communicate w/ device
// paramfile: calibration file (uses calib file w/ TWO values for each finger: see calibGripBox.m)
//////////////////////////////////////////////////////////////
bool GripBox::init(char *portName, char *paramfile)  {

	int i,ch=0;
	// 626 registerAD(channel 0, 10, board 0)
	for (i=0; i<NUMSENSORS-5; i++){
		ADchannel[i]=s626.registerAD(i,10.0,0);
		cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
		if (ch<0) 
			exit(-1); 
	} 
	for (i=5;i<NUMSENSORS; i++) {  
	ADchannel[i]=s626.registerAD(i+3,10.0,0);
	cout<<ch<<"    " <<ch<<endl; // DELAY A LITTLE BIT , OTHERWISE INITIALIZATION IS NOT WORKING! 
	if (ch<0) 
		exit(-1); 
	} 

	//We're not yet connected
    this->connected = false;
	//Try to connect to the given port throuh CreateFile
    this->port = CreateFile(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);
	//Check if the connection was successfull
	if(this->port==INVALID_HANDLE_VALUE)
		{printf("ERROR!!!");return false;}		
	else
	{
		//If connected we try to set the comm parameters
		DCB dcbSerialParams = {0};
		//Try to get the current
		if (!GetCommState(this->port, &dcbSerialParams))
		{
			//If impossible, show an error
			printf("failed to get current serial parameters!");
			return false;
		}
		else
		{
			//Define serial connection parameters for the arduino board
			dcbSerialParams.BaudRate=CBR_9600;
			dcbSerialParams.ByteSize=8;
			dcbSerialParams.StopBits=ONESTOPBIT;
			dcbSerialParams.Parity=NOPARITY;
			//Set the parameters and check for their proper application
		    if(!SetCommState(port, &dcbSerialParams))
			{
				printf("ALERT: Could not set Serial Port parameters");
				return false;
	        }
		    else
			{
			    //If everything went fine we're connected
				this->connected = true;
				//Flush any remaining characters in the buffers 
				PurgeComm(this->port, PURGE_RXCLEAR | PURGE_TXCLEAR);
				commReady = true;
				return true;
			}
		}
	}
	//-----set volts2force scale factors
	ifstream inputFile(paramfile.c_str(),ios::in);
	
	if(inputFile==0){
		cout<<"Could not open file: " <<paramfile<<endl:
		exit(-1);
	} else{
		// ignore headers in file
		i=0;
		while (!(inputFile.bad() || inputFile.eof())) {
			inputFile>>scale[i];
			i=i+1;
			} //while
	} //else
	cout<<endl;
	cout<< "V2F scale params set" <<endl; 
	cout<<endl;
	return true; 
}  // init



////////////////////////////////////////////////////////////////
// Get raw position position of specific motor
////////////////////////////////////////////////////////////////
	int GripBox::readRawPos(int channel) {
		
		int position;
		unsigned char command[2];
		unsigned char response[2];
		BOOL success;
		DWORD bytesTransferred;
		// Compose the command.
		command[0] = 0x90;
		command[1] = channel;
		// Send the command to the device.
		success = WriteFile(this->port, command, sizeof(command), &bytesTransferred, NULL);
		if (!success)
		{
			fprintf(stderr, "Error: Unable to write Get Position command to serial port.  Error code 0x%x.", GetLastError());
			return 0;
		}
		if (sizeof(command) != bytesTransferred)
		{
			fprintf(stderr, "Error: Expected to write %d bytes but only wrote %d.", sizeof(command), bytesTransferred);
			return 0;
		}
		// Read the response from the device.
		success = ReadFile(this->port, response, sizeof(response), &bytesTransferred, NULL);
		if (!success)
		{
			fprintf(stderr, "Error: Unable to read Get Position response from serial port.  Error code 0x%x.", GetLastError());
			return 0;
		}
		if (sizeof(response) != bytesTransferred)
		{
			fprintf(stderr, "Error: Expected to read %d bytes but only read %d (timeout). "
				"Make sure the Maestro's serial mode is USB Dual Port or USB Chained.", sizeof(command), bytesTransferred);
			return 0;
		}
		// Convert the bytes received in to a position.
		position = response[0] + 256*response[1];
		
		return position;
		
}

///////////////////////////////////////////////////////////////
// Set new raw target position to specific motor
///////////////////////////////////////////////////////////////
bool GripBox::setRawTarget(int channel, int target) {
		
	unsigned char command[4];
	DWORD bytesTransferred;
	BOOL success;
	// Compose the command.
	command[0] = 0x84;
	command[1] = channel;
	command[2] = target & 0x7F;
	command[3] = (target >> 7) & 0x7F;
	// Send the command to the device.
	success = WriteFile(this->port, command, sizeof(command), &bytesTransferred, NULL);
	if (!success)
	{
		fprintf(stderr, "Error: Unable to write Set Target command to serial port.  Error code 0x%x.", GetLastError());
		return false;
	}
	if (sizeof(command) != bytesTransferred)
	{
		fprintf(stderr, "Error: Expected to write %d bytes but only wrote %d.", sizeof(command), bytesTransferred);
		return false;
	}

	return true;
}

//////////////////////////////////////////////////////////////
// Connection flag
//////////////////////////////////////////////////////////////
bool GripBox::isConnected()
{
	return this->connected;
}

//////////////////////////////////////////////////////////////
// Update
//////////////////////////////////////////////////////////////
bool GripBox::update(void){

	int i,y;
	for (y=0; y<NUMSENSORS; y++) {
		i = channelIdx[y]; 
		volts[i]=s626.getAD(ADchannel[i],0);
		force[i]=(volts[i]-baseline[i])*scale[i];
	}
	
	for (i=1;i<6;i++){
	rawPosition[i]=GripBox::readRawPos(i);
	position[i] = (double)(rawPosition[i] - 4000) / 20;
	}

	return true;
}
//////////////////////////////////////////////////////////////
// Set a new position in mm
//////////////////////////////////////////////////////////////
bool  GripBox::setTarget(int channel,double position)
{
	targetPos[channel]=(int)((position*20)+4000);
	GripBox::setRawTarget(channel,targetPos[channel]);
	return true;
}

bool  GripBox::setTarget(double position[])
{
	for (int i=1;i<6;i++){
	targetPos[i]=(int)((position[i]*20)+4000);
	GripBox::setRawTarget(i,targetPos[i]);
	}
	return true;
}
//////////////////////////////////////////////////////////////
// zeroForce 
//////////////////////////////////////////////////////////////
void GripBox::zeroForce(double x[NUMSENSORS]){
	int i; 
	for (i=0; i<NUMSENSORS; i++) {
		baseline[i]=x[i];
	} 
} 
//////////////////////////////////////////////////////////////
// getForce 
//////////////////////////////////////////////////////////////
double GripBox::getForce(int channel){
	int i;
	i = channelIdx[channel];
	return force[i];
}
//////////////////////////////////////////////////////////////
// getVolts
//////////////////////////////////////////////////////////////
double GripBox::getVolts(int channel){
	int i;
	i = channelIdx[channel];
	return volts[i];
}

