/**************************************************************
** DataManager.h
** Record object make sure that data get recorded properly 
** Also keeps track of the general timer 
** Also ensures that data is written out in case of program crash 
** Has to be inherited to define record() and write() function 
****************************************************************/ 
#ifndef DEF_DATAMANAGER
#define DEF_DATAMANAGER

#include <iostream>

///////////////////////////////////////////////////////////////
/// \brief Template class that takes care of memory allocation for recorded data 
/// 
/// Record object make sure that data get recorded properly. 
/// It ensures that data is written out in case of program crash.  
/// You just havbe to specify a DataRecord Object that can be plugged in. 
/// It also gives you access to previously recorded data, in case you want to 
/// replay a trial to the subject or do some off-line computation. 
/// If the data array structure overflows, DataManager will automatically 
/// kill older data to store the more recent one. 
/// \sa DataRecord
///
///////////////////////////////////////////////////////////////
template <class T, int N>
class DataManager{
public:
	DataManager();										///< Constructor 
	~DataManager();										///< Destructor 
	void startRecording(void) {isRecord=TRUE;}			///< Start the recording 
	void stopRecording(void) {isRecord=FALSE;}			///< Stop the recording 
	bool isRecording(void) {return isRecord;}
	int numRecords(void) {return n;}				///< Returns the number of recorded data 
	bool record(T);									///< Puts a DataRecord (class T) into allocated array
	T getRecord(int n) {return data[n];}			///< Returns record n  
	void save(ostream &file);						///< Save the data currenly contained 
	void clear();
private: 
	T data[N];										///< Core data array
	int n;											///< Number of recorded data 
	bool isRecord;
};

//////////////////////////////////////////////////////////////
/// Example usage: DataManager <DataRecord,2000>: holds 2000 of class data record. 
/// /param T Class Type to be plugged in (DataRecord)
/// /param N Maximal numbers of records to be stored 
///////////////////////////////////////////////////////////////
template <class T, int N>
DataManager<T,N>::DataManager() 
{
	n=0;
	isRecord=FALSE; 
}

/**********************
** Destructor 
***********************/
template <class T, int N>
DataManager<T,N>::~DataManager(void) {
}


/*************************
** clear 
**************************/ 
template <class T, int N>
void DataManager<T,N>::clear()
{
	n=0;
}

//////////////////////////////////////////////////////////////
/// /param  datum is of class DataRecord 
/// /return was there enough space for the record?  
///////////////////////////////////////////////////////////////
template <class T, int N>
bool DataManager<T,N>::record(T datum)
{
	if (n<N){ 
		data[n]=datum;   // default memberwise copy 
		n++;
		return TRUE;
	} 
	return FALSE;
}

//////////////////////////////////////////////////////////////
/// /param  out Stream linked to mov-file.  
///
///////////////////////////////////////////////////////////////
template <class T, int N>
void DataManager<T,N>::save(ostream &out){
	int i;
	for (i=0;i<n;i++) { 
		data[i].write(out); 
	} 
}

#endif