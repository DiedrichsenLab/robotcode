// **********************************************************
// Class implementation of ATI
// **********************************************************

#include "ATI_DAQ.h"
#include "TextDisplay.h" 
extern TextDisplay tDisp; 

/// ----------------------------------------------------------
/// Constructor / destructor
/// ----------------------------------------------------------

ATI_DAQ::ATI_DAQ()
{ 
	tsk=0; 
	err=0;
	isRunning=0; 
} 

ATI_DAQ::~ATI_DAQ()
{
	if (tsk !=0) 
	{ 
		stop(); 
	} 
	
	/// Check if task is running and destroy if necessary 
	destroyCalib();
}

/// ----------------------------------------------------------
/// High level methods
/// ----------------------------------------------------------

void ATI_DAQ::init(float sampling, char*calib)
{ 
	sampling_rate = sampling;
	createCalib (calib);
	setPhysicalUnits ("N", "N-mm");
	getOneSample ();
	setBaseline ();
}

void ATI_DAQ::start(void)
{ 
	if (isRunning==0) { 
		tsk=0;
		int32*read=NULL;
		// DAQmx Configure Code

		err=DAQmxCreateTask ("",&tsk);
		if (err<0){
		DAQmxGetErrorString (err, errstr, 255);
		//	printf(errstr);
		}

		err=DAQmxCreateAIVoltageChan (tsk, "Dev1/ai0:5", "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, NULL);
		if (err<0){
			DAQmxGetErrorString (err, errstr, 255);
		//	printf(errstr);
		}

		err=DAQmxCfgSampClkTiming (tsk, "", sampling_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000);
		if (err<0){
			DAQmxGetErrorString (err, errstr, 255);
			//	printf(errstr);
		}

		// DAQmx Start Code

		err=DAQmxStartTask (tsk);
		if (err<0){
			DAQmxGetErrorString (err, errstr, 255);
		//	printf(errstr);
		}

		isRunning=1; 
	} 
} 


void ATI_DAQ::stop(void) 
{ 
	isRunning=0; 
	if (tsk!=0)
	{
		err=DAQmxStopTask (tsk);
		if (err<0)
		{
			DAQmxGetErrorString (err, errstr, 255);
//			printf(errstr);
		}

		err=DAQmxClearTask (tsk);
		if (err<0)
		{
			DAQmxGetErrorString (err, errstr, 255);
//			printf(errstr);
		}
	}
	tsk=0; 
}

void ATI_DAQ::update(void){
	int32 err=0; 
	if (isRunning) { 
		err = DAQmxReadAnalogF64 (tsk, -1, -1, DAQmx_Val_GroupByScanNumber, raw_sample, ATI_MAX_BUF_SIZE, read, NULL);
		if (read>0){
			cerr << read << endl;
		}
	
		if (err<0){
			DAQmxGetErrorString (err, errstr, 255);
			cerr<< errstr <<endl;
		}

		convertToPhysicalUnit(); 
	} 
} 

// ----------------------------------------------------------
// DESCR: Create a calibration structure
// PRE: paramfile: string with path to the (.cal) file name 
// POST: true if calibration file successfully loaded,
// false otherwise
// ----------------------------------------------------------

bool ATI_DAQ::createCalib(char * paramfile)
{
	// create Calibration class

	calibration = createCalibration(paramfile, 1);

	if (calibration == NULL) 
	{
		printf("\nSpecified calibration could not be loaded.\n");
		scanf(".");
		return false;
	}
	else
	{
		return true;
	}
}

// ----------------------------------------------------------
// DESCR: Free memory from calibration file
// PRE: 
// POST: 
// ----------------------------------------------------------

void ATI_DAQ::destroyCalib(void)
{
	destroyCalibration(calibration);
}


// ----------------------------------------------------------
// DESCR: Print the information from the calibration file
// PRE: a calibration has been created
// POST: true if calibration file successfully loaded, echo out the information
// false otherwise
// ARG: paramfile: string with path to the (.cal) file name 
// ----------------------------------------------------------

bool ATI_DAQ::printCalib(char * paramfile)
{
	bool ok = false;
	bool create = false;
	
	if (calibration == NULL) 
	{
		printf("\nNo calibration was created.\n");
		printf("Creating one...\n");

		createCalib (paramfile);
		create = true;
	}
	else
	{
		ok = true;
	}

	// display info from calibration file

	printf("--------------------------------------------------------------\n");
	printf("Calibration Information for %s\n",paramfile);
	printf("                  Serial: %s\n",calibration->Serial);
	printf("              Body Style: %s\n",calibration->BodyStyle);
	printf("             Calibration: %s\n",calibration->PartNumber);
	printf("        Calibration Date: %s\n",calibration->CalDate);
	printf("                  Family: %s\n",calibration->Family);
	printf("              # Channels: %i\n",calibration->rt.NumChannels);
	printf("                  # Axes: %i\n",calibration->rt.NumAxes);
	printf("             Force Units: %s\n",calibration->ForceUnits);
	printf("            Torque Units: %s\n",calibration->TorqueUnits);
	printf("Temperature Compensation: %s\n",(calibration->TempCompAvailable ? "Yes" : "No"));
	printf("--------------------------------------------------------------\n");

	// print maximum loads of axes
	
	printf("\nRated Loads\n");
	for (int i=0;i<calibration->rt.NumAxes;i++) 
	{
		char * units;
		if ((calibration->AxisNames[i])[0]=='F') 
		{
			units=calibration->ForceUnits;
		}
		else
		{
			units=calibration->TorqueUnits;
		}
		printf("%s: %f %s\n",calibration->AxisNames[i],calibration->MaxLoads[i],units);
	}
	printf("--------------------------------------------------------------\n");

	// print working calibration matrix

	printf("\nWorking Calibration Matrix\n");
	printf("     ");
	for (i=0;i<calibration->rt.NumChannels-1;i++)
	{
			printf("G%i            ",i);
	}
	printf("\n");
	for (i=0;i<calibration->rt.NumAxes;i++) 
	{
		printf("%s: ",calibration->AxisNames[i]);
		for (int j=0;j<calibration->rt.NumChannels-1;j++)
		{
			printf("%13.5e ",calibration->rt.working_matrix[i][j]);
		}
		printf("\n");
	}

	// free memory allocated to Calibration structure if created for the purpose of printing the info

	if (create)
	{
		destroyCalibration(calibration);
	}

	return ok;
}

// ----------------------------------------------------------
// DESCR: Set the physical units of force and torques
// PRE: calibration must exist
// POST: physical units set
//   -1: Calibration matrix not set
//   0: Successful completion
//   1: Invalid Calibration struct
//   2: Invalid force units
// ARG
// force: ("lb","klb","N","kN","g","kg")
// torque: ("in-lb","ft-lb","N-m","N-mm","kg-cm")
// ----------------------------------------------------------

short ATI_DAQ::setPhysicalUnits(char * force,char * torque)
{
	short sts = -1;

	if (calibration == NULL)
	{
		printf("\nA calibration has to be created first.");
		return -1;
	}

	sts = SetForceUnits(calibration,force);
	switch (sts) 
	{
		case 0: 
			break;	// successful completion
		
		case 1: 
			printf("Invalid Calibration struct"); 
			return 0;
		
		case 2: 
			printf("Invalid force units");
			return 0;

		default: 
			printf("Unknown error");
			return 0;
	}


	// Set torque units.

	sts = SetTorqueUnits(calibration,torque);
	switch (sts)
	{
		case 0: 
			break;	// successful completion

		case 1: 
			printf("Invalid Calibration struct");
			return 0;
		
		case 2: 
			printf("Invalid torque units");
			return 0;

		default: 
			printf("Unknown error");
			return 0;
	}

	return sts;
}

// ----------------------------------------------------------
// DESCR: Translate/rotate the sensor's coordinate system.
// PRE: calibration must exist
// POST: 
//   -1: Calibration matrix not set
//   0: Successful completion
//   1: Invalid Calibration struct
//   2: Invalid distance units
//   3: Invalid angle units
// ARG
//   vector: displacements and rotations in the order Dx, Dy, Dz, Rx, Ry, Rz
//   tranlation_unit: units of Dx, Dy, Dz
//   rotation_unit: units of Rx, Ry, Rz
// ----------------------------------------------------------

short ATI_DAQ::alterReferenceFrame (float vector[6], char * tranlation_unit, char * rotation_unit)
{
	short sts = -1;

	if (calibration == NULL)
	{
		printf("\nA calibration has to be created first.");
		return -1;
	}

	sts = SetToolTransform(calibration, vector,tranlation_unit,rotation_unit);

	switch (sts) 
	{
		case 0: 
			break;	// successful completion
		
		case 1: 
			printf("Invalid Calibration struct");
			return false;
		
		case 2: 
			printf("Invalid distance units");
			return false;
		
		case 3: printf("Invalid angle units");
			return false;
		
		default: 
			printf("Unknown error");
			return false;
	}
	
	return sts;
}

// ----------------------------------------------------------
// DESCR: Translate/rotate the sensor's coordinate system.
// PRE: 
// POST: 
// ARG
// ----------------------------------------------------------

void ATI_DAQ::setBaseline (void)
{
	float fraw_sample[6];
	int i = -1;

	for (i=0 ; i<6 ; i++)
	{
		fraw_sample[i] = (float) raw_sample[i];
	}

	Bias (calibration, fraw_sample);
}

// ----------------------------------------------------------
// DESCR: Translate/rotate the sensor's coordinate system.
// PRE: 
// POST: 
// ARG
// ----------------------------------------------------------

void ATI_DAQ::convertToPhysicalUnit (void)
{
	float fraw_sample[6];
	int i = -1;

	for (i=0 ; i<6 ; i++)
	{
		fraw_sample[i] = (float) raw_sample[i];
	}

	ConvertToFT (calibration, fraw_sample, ft_sample);

	force[0]=ft_sample[0];
	force[1]=ft_sample[1];
	force[2]=ft_sample[2];
	torque[0]=ft_sample[3];
	torque[1]=ft_sample[4];
	torque[2]=ft_sample[5];
}

void ATI_DAQ::getOneSample (void)
{
	int32 * read = NULL;

	// DAQmx Configure Code

	err=DAQmxCreateTask ("",&tsk);
	if (err<0)
	{
		DAQmxGetErrorString (err, errstr, 255);
		printf(errstr);
	}

	err=DAQmxCreateAIVoltageChan (tsk, "Dev1/ai0:5", "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, NULL);
	if (err<0)
	{
		DAQmxGetErrorString (err, errstr, 255);
		printf(errstr);
	}

	err=DAQmxCfgSampClkTiming (tsk, "", sampling_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, 2);
	if (err<0)
	{
		DAQmxGetErrorString (err, errstr, 255);
		printf(errstr);
	}

	// DAQmx Start Code

	err=DAQmxStartTask (tsk);
	if (err<0)
	{
		DAQmxGetErrorString (err, errstr, 255);
		printf(errstr);
	}

	// DAQmx Read Code

	err=DAQmxReadAnalogF64 (tsk, 1, -1, DAQmx_Val_GroupByChannel, raw_sample, 6, read, NULL);
	if (err<0)
	{
		DAQmxGetErrorString (err, errstr, 255);
		printf(errstr);
	}

	if (tsk!=0)  
	{
		// DAQmx Stop Code

		err=DAQmxStopTask (tsk);
		if (err<0)
		{
			DAQmxGetErrorString (err, errstr, 255);
			printf(errstr);
		}

		err=DAQmxClearTask (tsk);
		if (err<0)
		{
			DAQmxGetErrorString (err, errstr, 255);
			printf(errstr);
		}
	}
}

// -----------------------------------------------------------
// printState 
// -----------------------------------------------------------
char forcebuffer[60];
void ATI_DAQ::printState(int line){ 
	sprintf(forcebuffer,"Forces: %8.2f %8.2f %8.2f",force[0],force[1],force[2]);
	tDisp.setText(forcebuffer,line+0,0);
	sprintf(forcebuffer,"Torques: %8.2f %8.2f %8.2f",force[3],force[4],force[5]);
	tDisp.setText(forcebuffer,line+1,0);
} 


