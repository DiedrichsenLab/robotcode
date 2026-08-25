///////////////////////////////////////////////////////////////
/// LatencyTest
/// Flash a rectangle at the top of the screen for a duration
/// read from a one-column target file. Record onset/offset.
///////////////////////////////////////////////////////////////

#include "LatencyTest.h"

///////////////////////////////////////////////////////////////
/// Global variables
///////////////////////////////////////////////////////////////
S626sManager s626;
TextDisplay tDisp;
Screen gScreen;
Timer gTimer(UPDATERATE);
HapticState hs;
GraphicState gs;

char buffer[300];
HINSTANCE gThisInst;
Experiment* gExp;
Trial* currentTrial;

///////////////////////////////////////////////////////////////
/// Main
///////////////////////////////////////////////////////////////
int WINAPI WinMain(HINSTANCE hThisInst, HINSTANCE hPrevInst,
	LPSTR kposzArgs, int nWinMode)
{
	gThisInst = hThisInst;
	gExp = new MyExperiment("LatencyTest", "LatencyTest", "C:/data/LatencyTest/");
	gExp->redirectIOToConsole();

	tDisp.init(gThisInst, 0, 0, 550, 26, 8, 2, &(::parseCommand));
	tDisp.setText("Subj:", 0, 0);

	gScreen.init(gThisInst, 1920, 0, 1680, 1050, &(::updateGraphics));
	gScreen.setCenter(Vector2D(0, 0));
	gScreen.setScale(Vector2D(SCR_SCALE, SCR_SCALE));

	s626.init("c:/robotcode/calib/s626_single.txt");
	if (s626.getErrorState() == 0) {
		atexit(::onExit);
		s626.initInterrupt(updateHaptics, UPDATERATE);
	}

	gTimer.init();
	gExp->control();
	return 0;
}

///////////////////////////////////////////////////////////////
// MyExperiment
///////////////////////////////////////////////////////////////
MyExperiment::MyExperiment(string name, string code, string dDir) : Experiment(name, code, dDir) {
	theBlock = new MyBlock();
	theTrial = new MyTrial();
	currentTrial = theTrial;
}

void MyExperiment::control(void) {
	MSG msg;
	do {
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		theBlock->control();
		currentTrial->copyHaptics();
		if (gTimer[3] > UPDATE_TEXTDISP) {
			currentTrial->updateTextDisplay();
			InvalidateRect(tDisp.windowHnd, NULL, TRUE);
			UpdateWindow(tDisp.windowHnd);
			gTimer.reset(3);
		};
		InvalidateRect(gScreen.windowHnd, NULL, TRUE);
		UpdateWindow(gScreen.windowHnd);
	} while (msg.message != WM_QUIT);
}

bool MyExperiment::parseCommand(string arguments[], int numArgs) {
	return false;
}

void MyExperiment::onExit() {
	s626.stopInterrupt();
	tDisp.close();
	gScreen.close();
}

///////////////////////////////////////////////////////////////
// MyBlock
///////////////////////////////////////////////////////////////
MyBlock::MyBlock() {
	state = WAIT_BLOCK;
}

Trial* MyBlock::getTrial() {
	return new MyTrial();
}

void MyBlock::start() {
	gs.reset();
	gs.feedback = "";
}

void MyBlock::giveFeedback() {
	sprintf(buffer, "End of Block  trials: %d", numTrials);
	gs.feedback = buffer;
	gs.rectOn = false;
}

///////////////////////////////////////////////////////////////
// MyTrial
///////////////////////////////////////////////////////////////
MyTrial::MyTrial() {
	state = WAIT_TRIAL;
	flashDur = 0;
	onsetTime = 0;
	offsetTime = 0;
	onsetTimeReal = 0;
	offsetTimeReal = 0;
}

void MyTrial::read(istream& in) {
	in >> flashDur;
}

void MyTrial::writeDat(ostream& out) {
	out << flashDur << "\t"
		<< onsetTime << "\t"
		<< offsetTime << "\t"
		<< onsetTimeReal << "\t"
		<< offsetTimeReal << "\t"
		<< endl;
}

void MyTrial::writeHeader(ostream& out) {
	out << "flashDur" << "\t"
		<< "onsetTime" << "\t"
		<< "offsetTime" << "\t"
		<< "onsetTimeReal" << "\t"
		<< "offsetTimeReal" << "\t"
		<< endl;
}

void MyTrial::writeMov(ostream& out) {
	dataman.save(out);
}

void MyTrial::start() {
	dataman.clear();
	state = START_TRIAL;
}

void MyTrial::end() {
	state = END_TRIAL;
	dataman.stopRecording();
	gs.reset();
}

bool MyTrial::isFinished() {
	return(state == END_TRIAL ? TRUE : FALSE);
}

void MyTrial::copyHaptics() {
	S626_InterruptEnable(0, false);
	S626_InterruptEnable(0, true);
}

void MyTrial::updateTextDisplay() {
	sprintf(buffer, "State: %d   trial t: %2.1f", state, gTimer[1]);
	tDisp.setText(buffer, 2, 0);

	sprintf(buffer, "flashDur: %2.1f   onset: %2.1f   offset: %2.1f", flashDur, onsetTime, offsetTime);
	tDisp.setText(buffer, 4, 0);
}

void MyTrial::updateGraphics(int what) {
	double screenH = gScreen.getSize()[1];
	double y = screenH / 2.0 - FLASH_HEIGHT / 2.0;

	if (gs.rectOn) {
		gScreen.setColor(Screen::white);
	}
	else {
		gScreen.setColor(Screen::black);
	}
	gScreen.drawBox(FLASH_WIDTH, FLASH_HEIGHT, 0.0, y);

	if (!gs.feedback.empty()) {
		gScreen.setColor(Screen::white);
		gScreen.print(gs.feedback.c_str(), 0, 0, 6);
	}
}

void MyTrial::updateHaptics() {
	gTimer.countup();
	gTimer.countupReal();
	s626.updateAD(0);
	currentTrial->control();

	if (dataman.isRecording()) {
		gTimer.reset(4);
		bool x = dataman.record(DataRecord(state, gExp->theBlock->trialNum));
		if (!x) {
			dataman.stopRecording();
		}
	}
}

void MyTrial::control() {
	switch (state) {
	case WAIT_TRIAL:
		gs.rectOn = false;
		break;

	case START_TRIAL:
		onsetTime = 0;
		offsetTime = 0;
		onsetTimeReal = 0;
		offsetTimeReal = 0;
		gs.rectOn = false;
		gs.feedback = "";
		dataman.clear();
		dataman.startRecording();
		gTimer.reset(1);
		gTimer.reset(2);
		state = FLASH;
		break;

	case FLASH:
		if (!gs.rectOn) {
			gs.rectOn = true;
			onsetTime = gTimer[1];
			onsetTimeReal = gTimer.readReal(1);
			gTimer.reset(2);
		}
		if (gTimer[2] >= flashDur) {
			state = WAIT_ITI;
		}
		break;

	case WAIT_ITI:
		if (gs.rectOn) {
			gs.rectOn = false;
			offsetTime = gTimer[1];
			offsetTimeReal = gTimer.readReal(1);
			gTimer.reset(2);
		}
		if (gTimer[2] >= ITI) {
			dataman.stopRecording();
			state = END_TRIAL;
		}
		break;

	case END_TRIAL:
		gs.rectOn = false;
		break;
	}
}

DataRecord::DataRecord(int s, int t) {
	state = s;
	trialNum = t;
	timeReal = gTimer.readReal(1);
	time = gTimer[1];
	rectOn = gs.rectOn ? 1 : 0;
}

void DataRecord::write(ostream& out) {
	out << trialNum << "\t" << state << "\t" << timeReal << "\t" << time << "\t" << rectOn << endl;
}

GraphicState::GraphicState() {
	rectOn = false;
	feedback = "";
}

void GraphicState::reset(void) {
	rectOn = false;
	feedback = "";
}
