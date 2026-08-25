#pragma once

#include <windows.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "TextDisplay.h"
#include "Screen.h"
#include "Vector2d.h"
#include "DataManager.h"
#include "S626sManager.h"
#include "Experiment.h"
#include "Timer626.h"
#include "Win626.h"
#include <gl/glut.h>

using namespace std;

#define UPDATERATE 5			// haptic/control loop in ms
#define RECORDRATE 5			// mov recording in ms
#define UPDATE_TEXTDISP 10
#define SCR_SCALE 1.84/72		// cm/pixel
#define ITI 1000				// black period after each flash [ms]
#define FLASH_WIDTH 10.0		// rectangle width [cm]
#define FLASH_HEIGHT 4.0		// rectangle height [cm]

///////////////////////////////////////////////////////////////
// Enumeration of Trial State
///////////////////////////////////////////////////////////////
enum TrialState {
	WAIT_TRIAL,
	START_TRIAL,
	FLASH,
	WAIT_ITI,
	END_TRIAL
};

///////////////////////////////////////////////////////////////
// Haptic state: unused, kept for the interrupt copy pattern
///////////////////////////////////////////////////////////////
class HapticState {
public:
	HapticState() {}
};

///////////////////////////////////////////////////////////////
// Graphic state
///////////////////////////////////////////////////////////////
class GraphicState {
public:
	GraphicState();
	void reset(void);
	bool rectOn;				///< white rectangle when true, black otherwise
	string feedback;			///< block-end text
};

///////////////////////////////////////////////////////////////
/// Data Record: written to the mov file at 200 Hz
///////////////////////////////////////////////////////////////
class DataRecord {
public:
	DataRecord() {}
	DataRecord(int s, int t, double dur);
	void write(ostream& out);
public:
	int trialNum;
	int state;
	int rectOn;
	double flashDur;
	double time;				///< trial clock (interrupt)
	double timeReal;			///< trial clock (s626)
	double timeBlock;			///< time from block start (interrupt)
	double timeBlockReal;		///< time from block start (s626)
	double timeAbs;				///< s626 base clock from program start
};

///////////////////////////////////////////////////////////////
// MyBlock
///////////////////////////////////////////////////////////////
class MyBlock :public Block {
public:
	MyBlock();
	virtual Trial* getTrial();
	virtual void giveFeedback();
	virtual void start();
};

///////////////////////////////////////////////////////////////
/// MyTrial: one row of the target file
///////////////////////////////////////////////////////////////
class MyTrial :public Trial {
public:
	MyTrial();
	virtual void writeHeader(ostream& out);
	virtual void read(istream& in);
	virtual void updateGraphics(int i);
	virtual void updateHaptics();
	virtual void updateTextDisplay();
	virtual void copyHaptics();
	virtual void control();
	virtual void start();
	virtual void end();
	virtual bool isFinished();
	virtual void writeDat(ostream& out);
	virtual void writeMov(ostream& out);
	friend void MyBlock::giveFeedback();

private:
	TrialState state;
	double flashDur;			///< how long the rectangle stays white [ms], from .tgt
	double onsetTime;			///< trial clock at white onset
	double offsetTime;			///< trial clock at white offset
	double duration;			///< offsetTime - onsetTime
	double onsetTimeReal;		///< trial s626 clock at white onset
	double offsetTimeReal;		///< trial s626 clock at white offset
	double durationReal;		///< offsetTimeReal - onsetTimeReal
	double onsetTimeBlock;		///< block clock at white onset (aligns with Arduino)
	double offsetTimeBlock;		///< block clock at white offset
	double durationBlock;		///< offsetTimeBlock - onsetTimeBlock
	double onsetTimeBlockReal;	///< block s626 clock at white onset
	double offsetTimeBlockReal;	///< block s626 clock at white offset
	double onsetAbs;			///< s626 base clock at white onset
	double offsetAbs;			///< s626 base clock at white offset
	DataManager<DataRecord, 30000 / 5> dataman;
};

///////////////////////////////////////////////////////////////
/// MyExperiment
///////////////////////////////////////////////////////////////
class MyExperiment :public Experiment {
public:
	MyExperiment(string tit, string code, string dDir);
	virtual void control();
	virtual void onExit(void);
	virtual bool parseCommand(string args[], int numArgs);
};
