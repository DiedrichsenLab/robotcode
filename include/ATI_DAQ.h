// -------------------------------------------------------------------------
// Forcetransducer class 
// 
// under a s626 iocard 
// 
// --------------------------------------------------------------------------

#ifndef ATI_H
#define ATI_H 

#include <iostream>

using namespace std;


// NI specific
#include "NIDAQmx.h"
#include "ftconfig.h"



#define ATI_MAX_BUF_SIZE 1200

class ATI_DAQ 
{ 
	public:
		ATI_DAQ();
		~ATI_DAQ();
		
		void init(float, char *);	    ///< initialize the ATI and unbias the channels (arg: sampling rate per channel, calibration filename)
		void start(void);				///< arm the DAQ and prepare the harware to read a sample
		void stop(void);				///< stop the task			
		void update(void);				///< acquire 1 sample and transfer to physical units
		void printState(int line);		///< Print the state in  the text display 
		void setBaseline (void);
	public: 
		/// data structures
		float force[3];				    ///< Force in N (x,y,z)
		float torque[3];				
		bool isRunning; 

	private: 
		/// private initialization methods

		bool createCalib (char *);     
		void destroyCalib(void);
		short setPhysicalUnits (char *, char *);
		bool printCalib (char *);
		short alterReferenceFrame (float[6], char *, char *);

		/// private ATI methods

		void convertToPhysicalUnit (void);

		/// private NI methods

		void getOneSample (void);


	private: 
		double raw_sample[ATI_MAX_BUF_SIZE];
		float ft_sample[ATI_MAX_BUF_SIZE];
		float sampling_rate;
		Calibration * calibration;
		int32 * read;
		TaskHandle tsk;
		int32 err;
		char errstr[255];
}; 

#endif 
