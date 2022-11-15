//////////////////////////////////////////////////////////////
///	Support class for sequence hierarchical
///		Function: define sequence and chunk structures
///	
/// ver. 2.0 added temporal structure (Oct 2014)
/// ver. 1.2 inport check sequence function (Oct 2014)
/// 
///	a-yokoi (May 2014)
///
//////////////////////////////////////////////////////////////

#include "Sequence.h"
//#include <windows.h>
//#include <conio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cassert>

using namespace std;

/////////////////////////////////////////////
//
// Sequence Class
//
/////////////////////////////////////////////

///////////////////////
/// Constructor
///////////////////////
Sequence::Sequence(){
	numSeq		= 8;
	seqLength	= 4;
	pvSeq.assign(numSeq,NULL);
}

//////////////////////
/// Destructor
//////////////////////
Sequence::~Sequence(){
	vector<int>* tmp;
	for (int i=0; i<pvSeq.size(); i++){
		tmp = pvSeq.at(i);
		if (!tmp==NULL) { 
			delete tmp;
			tmp = NULL;		
		}		
	}	
	pvSeq.clear();
	vector<vector<int>*>().swap(pvSeq);
}

/////////////////////////////////////
/// Initialize
/////////////////////////////////////
bool Sequence::init(string fSeq, string fChunk, string fTempo){
	if ((initSeq(fSeq))&&(this->chunk.initChunk(fChunk))&&(this->tempo.initTempo(fTempo)))	{
		// check validity
		for (int i=0; i< this->getNumSeq(); i++){
			if (getCompleteSequence(i).size()!=this->tempo.getTempo(i,100).size()){
				cout<<"Error: Sequence No."<<i<<" and Tempo No."<<i<<" should have the same length!"<<endl;
				exit(1);
			}
		}
		return true;
	}
	else {return false;}
}

bool Sequence::initSeq(string fSeq){
	int i	= 0;
	int j	= 0;
	int tmp = 0;
	vector<int>* seq;
	string s;
	
	// load definition file
	ifstream inputFile(fSeq.c_str(),ios::in);
	
	if(inputFile == 0){
		cout<<"Couldn't open file: " <<fSeq<<endl;
		//exit(-1);
		return false;
	} 
	else{
		// clear existing definition
		//cout<<"size of pvSeq = "<<pvSeq.size()<<endl;
		for (i=0; i<pvSeq.size(); i++){
			seq = pvSeq.at(i);
			if (!seq==NULL) { 
				delete seq;			
				//cout<<"address "<<seq<<" deleted!"<<endl;			
				seq = NULL;			
			}	
		}
		pvSeq.clear();
		vector<vector<int>*>().swap(pvSeq);
		//cout<<"size of pvSeq = "<<pvSeq.size()<<endl;
	
		// ignore headers in file (1)
		getline(inputFile,s);
		// get parameters
		inputFile>>numSeq>>seqLength;
		getline(inputFile,s);

		// ignore headers in file (2)
		getline(inputFile,s);

		cout<<endl;
		cout<<"Open sequence definition file: "<<fSeq.c_str()<<endl;
		//cin>>s;
	
		// input sequence definition into chunks 
		i=0;
		while (!(inputFile.bad() || inputFile.eof())&&pvSeq.size()<numSeq){ 		
			cout<<"Seq["<<i+1<<"] = ";

			seq = new vector<int>;	
			for (j=0;j<seqLength;j++) {
				inputFile>>tmp;
				seq->push_back(tmp);
				cout<<seq->at(j)<<" ";
			}
			cout<<endl;

			pvSeq.push_back(seq);								
			getline(inputFile,s);	
			i++;
		}
		if (pvSeq.size() != numSeq) {
			cerr<<"no. of sequence doesn't match!"<<endl;
		}
	}
	return true;	
}

////////////////////////////////////////////////////////
void Sequence::getCHSequence(int n, vector<int>* chSEQ){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvSeq.size());

	vector<int>* seq;
	seq = pvSeq.at(n);	
	// delete existing data and assign new data
	chSEQ->clear();	
	for (int i=0; i<seq->size(); i++){
		chSEQ->push_back(seq->at(i));
	}
}

//////////////////////////////////////////////////////////////////////////
void Sequence::getCompleteSequence(int n, vector<int>* SEQ, Chunk* CH){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvSeq.size());

	vector<int>* seq;
	vector<int>* chunk;
	if (n<0) {n = 0;}
	if (n>pvSeq.size()-1){n=pvSeq.size()-1;}

	// delete existing data
	SEQ->clear();
	
	seq = pvSeq.at(n);	
	for (int i=0; i<seqLength; i++){		
		chunk = CH->pvChunk.at(seq->at(i)-1);	
		
		for (int j=0; j<CH->chunkLength; j++) {			
			// ignore zero or smaller values
			if (chunk->at(j)>0) {
				SEQ->push_back(chunk->at(j));				
			}
			else {
				//cout<<"else"<<endl;
			}
		}
	}
	//cout<<"ending Sequence::getCompleteSequence()"<<endl;
}
void Sequence::getCompleteSequence(int n,vector<int>* SEQ){
	getCompleteSequence(n, SEQ, &this->chunk);
}
vector<int> Sequence::getCompleteSequence(int n){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvSeq.size());
	
	vector<int>* seq;
	vector<int>* chunk_;
	vector<int> SEQ;
	if (n<0) {n = 0;}
	if (n>pvSeq.size()-1){n=pvSeq.size()-1;}

	seq = pvSeq.at(n);
	for (int i=0; i<seqLength; i++){		
		chunk_ = this->chunk.pvChunk.at(seq->at(i)-1);			
		for (int j=0; j<this->chunk.chunkLength; j++) {			
			if (chunk_->at(j)>0) {
				SEQ.push_back(chunk_->at(j));				
			}			
		}
	}
	return SEQ;
}

////////////////////////////////////////////////////////////////////////
void Sequence::getChunkLengths(int n, vector<int>* Lengths, Chunk* CH){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvSeq.size());
	
	int length = 0;
	vector<int>* seq;
	vector<int>* chunk;
	if (n<0) {
		n = 0;
	}
	
	seq = pvSeq.at(n);
	
	// clear old value if exist and assign new value
	if (!Lengths->empty()){Lengths->clear();}
	for (int i=0; i<seq->size(); i++){
		chunk = CH->pvChunk.at(seq->at(i)-1);
		length = 0;
		for (int j=0; j<CH->chunkLength; j++) {
			// ignore zero or smaller values
			if (chunk->at(j)>0) {
				length++;
			}
			else {
				//cout<<"else"<<endl;
			}
		}
		Lengths->push_back(length);
	}
}
void Sequence::getChunkLengths(int n, vector<int>* Lengths){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvSeq.size());
	
	int length = 0;
	vector<int>* seq;
	vector<int>* chunk;
	if (n<0) {
		n = 0;
	}
	
	seq = pvSeq.at(n);
	
	// clear old value if exist and assign new value
	if (!Lengths->empty()){Lengths->clear();}
	for (int i=0; i<seq->size(); i++){
		chunk = this->chunk.pvChunk.at(seq->at(i)-1);
		length = 0;
		for (int j=0; j<this->chunk.chunkLength; j++) {
			// ignore zero or smaller values
			if (chunk->at(j)>0) {
				length++;
			}
			else {
				//cout<<"else"<<endl;
			}
		}
		Lengths->push_back(length);
	}
}
/////////////////////////////////////////////////////
void Sequence::getTargetRT(int n, vector<double>* tRT, int duration, Tempo* T){
	T->getTempo(n, tRT, duration);
}
void Sequence::getTargetRT(int n, vector<double>* tRT, int duration){
	this->tempo.getTempo(n, tRT, duration);
}
vector<double> Sequence::getTargetRT(int n, int duration){
	return this->tempo.getTempo(n, duration);
}
void Sequence::getTargetITI(int n, vector<double>* tITI, int duration, Tempo* T){
	T->getInterval(n, tITI, duration);
}
void Sequence::getTargetITI(int n, vector<double>* tITI, int duration){
	this->tempo.getInterval(n, tITI, duration);
}
vector<double> Sequence::getTargetITI(int n, int duration){
	return this->tempo.getInterval(n, duration);
}



/////////////////////////////////////////////
// Chunk Class
/////////////////////////////////////////////
/// Constructor
Chunk::Chunk(){
	numChunk		= 8;
	chunkLength		= 4;
	pvChunk.assign(numChunk,NULL);
}
Chunk::Chunk(string fChunk){
	numChunk		= 8;
	chunkLength		= 4;
	pvChunk.assign(numChunk,NULL);	
	initChunk(fChunk);
}
/// Destructor
Chunk::~Chunk(){
	vector<int>* chunk;
	for (int i=0; i<pvChunk.size(); i++){
		chunk = pvChunk.at(i);
		if (!chunk==NULL) { 
			delete chunk;			
		}
		pvChunk.clear();
		vector<vector<int>*>().swap(pvChunk);
	}	
}

/// Initialize
bool Chunk::initChunk(string fChunk){
	int i	= 0;
	int j	= 0;
	int tmp = 0;
	vector<int>*	chunk;
	string s;
	
	// load definition file
	ifstream inputFile(fChunk.c_str(),ios::in);
	
	if(inputFile == 0){
		cout<<"Couldn't open file: " <<fChunk<<endl;
		//exit(-1);
		return false;
	} 
	else{
		// clear existing definition
		//cout<<endl<<"size of pvChunk = "<<pvChunk.size()<<endl;
		for (i=0; i<pvChunk.size(); i++){
			chunk = pvChunk.at(i);
			//cout<<endl<<"size of pvChunk = "<<pvChunk.size()<<endl;
			//cout<<"i="<<i<<" chunk address = "<<chunk<<endl;
			if (!chunk==NULL) { 
				delete chunk;
				//cout<<"address "<<chunk<<" deleted!"<<endl;							
				//cout<<"size of pvChunk = "<<pvChunk.size()<<endl;
			}		
		}	
		pvChunk.clear();
		vector<vector<int>*>().swap(pvChunk);
		//cout<<"size of pvChunk = "<<pvChunk.size()<<endl;
	
		// ignore headers in file (1)
		getline(inputFile,s);
		//cout<<"header="<<s.c_str()<<endl;
		// get parameters
		inputFile>>numChunk>>chunkLength;
		getline(inputFile,s);

		//cout<<"num="<<numChunk<<" length="<<chunkLength<<endl;
		//cin>>s;

		// ignore headers in file (2)
		getline(inputFile,s);
		//cout<<"header="<<s.c_str()<<endl;

		cout<<endl;
		cout<<"Open sequence definition file: "<<fChunk.c_str()<<endl;
		//cin>>s;
	
		// input sequence definition into chunks 
		i=0;
		while (!(inputFile.bad() || inputFile.eof())&&pvChunk.size()<numChunk){ 		
			cout<<"Chunk["<<i+1<<"] = ";

			chunk = new vector<int>;
			for (j=0;j<chunkLength;j++) {
				inputFile>>tmp;
				chunk->push_back(tmp);
				cout<<chunk->at(j)<<" ";				
			}
			cout<<endl;

			pvChunk.push_back(chunk);									
			getline(inputFile,s);									
			i++;
		}
		if (pvChunk.size() != numChunk) {
			cerr<<"no. of chunk doesn't match!"<<endl;
		}
	}
	return true;	
}

/// Get chunk info
void Chunk::getFing(int n, vector<int>* CH){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvChunk.size());
	
	vector<int>* ch;
	if (n<0){n=0;}
	if (n>pvChunk.size()-1){n=pvChunk.size()-1;}
	
	ch = pvChunk.at(n);
	// delete existing data and assign new data
	CH->clear();
	for (int i=0; i<ch->size(); i++){
		CH->push_back(ch->at(i));
	}
}
vector<int> Chunk::getFing(int n){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvChunk.size());
	
	if (n<0){n=0;}
	if (n>pvChunk.size()-1){n=pvChunk.size()-1;}

	return *pvChunk.at(n);
}

/////////////////////////////////////////////
// Tempo Class
/////////////////////////////////////////////
/// Constructor
Tempo::Tempo(){
	numTempo		= 8;
	tempoLength		= 12;
	pvTempo.assign(numTempo, NULL);	
}
Tempo::Tempo(string fTempo){
	numTempo		= 8;
	tempoLength		= 12;
	pvTempo.assign(numTempo, NULL);
	initTempo(fTempo);
}
/// Destructor
Tempo::~Tempo(){
	vector<double>* tempo;
	for (int i=0; i<pvTempo.size(); i++){
		tempo = pvTempo.at(i);
		if (!tempo==NULL) { 
			delete tempo;			
		}
		pvTempo.clear();
		vector<vector<double>*>().swap(pvTempo);
	}	
}

/// Initialize
bool Tempo::initTempo(string fTempo){
	int i	= 0;
	int j	= 0;
	int tmp = 0;
	vector<double>*	tempo;
	string s;
	
	// load definition file
	ifstream inputFile(fTempo.c_str(),ios::in);
	
	if(inputFile == 0){
		cout<<"Couldn't open file: " <<fTempo<<endl;
		//exit(-1);
		return false;
	} 
	else{
		// clear existing definition
		//cout<<endl<<"size of pvTempo = "<<pvTempo.size()<<endl;
		for (i=0; i<pvTempo.size(); i++){
			tempo = pvTempo.at(i);
			//cout<<endl<<"size of pvTempo = "<<pvTempo.size()<<endl;
			//cout<<"i="<<i<<" tempo address = "<<tempo<<endl;
			if (!tempo==NULL) { 
				delete tempo;
				//cout<<"address "<<tempo<<" deleted!"<<endl;			
				//cout<<"size of pvTempo = "<<pvTempo.size()<<endl;
			}		
		}	
		pvTempo.clear();
		vector<vector<double>*>().swap(pvTempo);
		//cout<<"size of pvTempo = "<<pvTempo.size()<<endl;

		// ignore headers in file (1)
		getline(inputFile,s);
		//cout<<"header="<<s.c_str()<<endl;
		// get parameters
		inputFile>>numTempo>>tempoLength;
		getline(inputFile,s);

		//cout<<"num="<<numTempo<<" length="<<tempoLength<<endl;
		//cin>>s;

		// ignore headers in file (2)
		getline(inputFile,s);
		//cout<<"header="<<s.c_str()<<endl;

		cout<<endl;
		cout<<"Open sequence definition file: "<<fTempo.c_str()<<endl;
		//cin>>s;
	
		// input sequence definition into chunks 
		i=0;
		while (!(inputFile.bad() || inputFile.eof())&&pvTempo.size()<numTempo){ 		
			//cout<<"Tempo["<<i+1<<"] = ";

			tempo = new vector<double>;
			for (j=0;j<tempoLength;j++) {
				inputFile>>tmp;
				tempo->push_back(tmp);
				//cout<<tempo->at(j)<<" ";				
			}
			//cout<<endl;

			pvTempo.push_back(tempo);									
			getline(inputFile,s);									
			i++;
		}
		if (pvTempo.size() != numTempo) {
			cerr<<"no. of tempo doesn't match!"<<endl;
			cout<<pvTempo.size()<<endl;
			cout<<numTempo<<endl;
		}
	}
	return true;	
}

/// Get tempo info 
void Tempo::getTempo(int n, vector<double>* Tmp, int duration){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvTempo.size());
	
	double ctmp = 0;
	double ctmp_ = 0;
	vector<double>* tmp;
	if (n<0){n=0;}
	if (n>pvTempo.size()-1){n=pvTempo.size()-1;}
	
	tmp = pvTempo.at(n);
	// calc total
	for (int i=0; i<tmp->size(); i++){
		ctmp += tmp->at(i);
	}
	// delete existing data and assign
	Tmp->clear();
	for (i=0; i<tmp->size(); i++){
		ctmp_ += tmp->at(i)*duration/abs(ctmp); // change intervals into single time stamp
		Tmp->push_back(ctmp_);
		//(duration is supposed to be read from .tgt file)
	}
}
vector<double> Tempo::getTempo(int n, int duration){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvTempo.size());

	double ctmp = 0;
	double ctmp_ = 0;
	vector<double> tmp;
	if (n<0){n=0;}
	if (n>pvTempo.size()-1){n=pvTempo.size()-1;}
	tmp = *pvTempo.at(n);
	for (int i=0; i<tmp.size(); i++){
		ctmp += tmp[i];
	}
	for (i=0; i<tmp.size(); i++){
		ctmp_ += tmp[i]*duration/abs(ctmp);
		tmp[i] = ctmp;
	}	
	return tmp;
}

/// Get intervals
void Tempo::getInterval(int n, vector<double>* Intervals, int duration){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvTempo.size());
	
	double ctmp = 0;
	double ctmp_ = 0;
	vector<double>* tmp;
	if (n<0){n=0;}
	if (n>pvTempo.size()-1){n=pvTempo.size()-1;}
	
	tmp = pvTempo.at(n);
	// calc total
	for (int i=0; i<tmp->size(); i++){
		ctmp += tmp->at(i);
	}
	// delete existing data and assign
	Intervals->clear();
	for (i=0; i<tmp->size(); i++){
		ctmp_ = tmp->at(i)*duration/abs(ctmp); // stretch intervals 
		Intervals->push_back(ctmp_);
		//(duration is supposed to be read from .tgt file)
	}
}
vector<double> Tempo::getInterval(int n, int duration){
	if (n<0){n=0;};//assert(n>=0);
	assert(n<pvTempo.size());

	double ctmp = 0;
	double ctmp_ = 0;
	vector<double> intervals;
	if (n<0){n=0;}
	if (n>pvTempo.size()-1){n=pvTempo.size()-1;}
	intervals = *pvTempo.at(n);
	for (int i=0; i<intervals.size(); i++){
		ctmp += intervals[i];
	}
	for (i=0; i<intervals.size(); i++){
		ctmp_	= intervals[i]*duration/abs(ctmp);
		intervals[i]	= ctmp;
	}	
	return intervals;
}