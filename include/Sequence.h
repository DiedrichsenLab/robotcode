///////////////////////////////////////
/// Class: Sequence (& Chunk)
///
/// v.2.0 (added temporal structure)
/// 
/// added by a-yokoi (May 2014)
///////////////////////////////////////

#ifndef SEQUENCE_H
#define SEQUENCE_H
//#include <windows.h>
//#include <conio.h>
#include <stdio.h>
//#include <iostream>
//#include <fstream>
#include <string>
#include <vector>

using namespace std; 

////////////////////////////////////////////////////////////
/// Chunk class
/// 	defines chunks to construct sequences
////////////////////////////////////////////////////////////
class Chunk{
public:
	Chunk(); 									/// default constructor	
	explicit Chunk(string fChunk); 				/// constructor with definition
	~Chunk(); 									/// destructor		
	bool initChunk(string fChunk);				/// initialize chunk class from definition file	
	void getFing(int n, vector<int>* CH);		/// get vector of single digit (1,2,...,5) for n-th chunk into CH
	vector<int> getFing(int n); 				/// get vector of single digit for n-th chunk and return it
	int getChunkLength(){return chunkLength;}; 	/// return chunkLength
	int getNumChunk(){return numChunk;};		/// return numChunk
private:
	int numChunk;								/// total number of chunks
	int chunkLength;							/// length of each chunk
	vector<vector<int>*> pvChunk; 				/// vector of pointer vector of chunk
	friend class Sequence;
	friend class MyTrial;
};

////////////////////////////////////////////////////////////
/// Tempo class
/// 	defines timing at single digit level
/// 	each tempo is defined in relative length (i.e., sums to 1)
/// 	actual values are given by MyTrial.complete*(*pvTempo.at(n)))
////////////////////////////////////////////////////////////
class Tempo{
public:
	Tempo(); 													/// default constructor	
	explicit Tempo(string fTempo); 								/// constructor with definition
	~Tempo(); 													/// destructor		
	bool initTempo(string fTempo);								/// initialize tempo class from definition file	
	void getTempo(int n, vector<double>* Tmp, int duration);	/// store target timing (in msec) for n-th tempo into Tmp
	vector<double> getTempo(int n, int duration);				/// same as above, but return value
	void getInterval(int n, vector<double>* Intervals, int duration);	/// store intervals into vector
	vector<double> getInterval(int n, int duration);					/// same as above, but return value

private:
	int numTempo;												/// total number of tempo types
	int tempoLength;											/// length of each tempo (should be same as length of sequence at single digit level)	
	vector<vector<double>*> pvTempo; 							/// vector of pointer vector of tempo
	friend class Sequence;
	friend class MyTrial;
};

////////////////////////////////////////////////////////////
class Sequence {
public:
	Sequence();														/// constructor	
	~Sequence();													/// deconstructor

	bool init(string fSequence, string fChunk, string fTempo); 		/// initialize seq, chunk, and tempo
	bool initSeq(string fSequence);									/// initialize seq. class from def file
	void getCHSequence(int n, vector<int>* chSEQ);					/// get sequence in chunk level
	void getCompleteSequence(int n, vector<int>* SEQ, Chunk* CH);	/// get complete sequence
	void getCompleteSequence(int n,vector<int>* SEQ);
	vector<int> getCompleteSequence(int n);
	void getChunkLengths(int n, vector<int>* Lengths, Chunk* CH);		/// get chunk length
	void getChunkLengths(int n, vector<int>* Lengths);					/// get chunk length
	int  getNumSeq(){return numSeq;};									/// get no. of sequence types
	int  getSeqLength(){return seqLength;};								/// get length of sequence at chunk level
	void getTargetRT(int n, vector<double>* tRT, int duration, Tempo* T); 	/// get target RT for a sequence type
	void getTargetRT(int n, vector<double>* tRT, int duration); 			/// get target RT for a sequence type
	vector<double> getTargetRT(int n, int duration); 						/// get target RT for a sequence type
	void getTargetITI(int n, vector<double>* tITI, int duration, Tempo* T); /// get target ITI for a sequence type
	void getTargetITI(int n, vector<double>* tITI, int duration);			/// get target ITI for a sequence type
	vector<double> Sequence::getTargetITI(int n, int duration);				/// get target ITI for a sequence type
	Chunk chunk;
	Tempo tempo;

private:
	int numSeq;						/// total number of sequences (defined in chunk level)
	int seqLength;					/// length of sequence (defined in chunk level)
	vector<vector<int>*> pvSeq;		/// vector of pointer vector of sequence	
	friend class MyBlock;
	friend class MyTrial;
};

#endif