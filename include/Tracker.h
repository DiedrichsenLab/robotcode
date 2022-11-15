// class Tracker:
//
//
// ************************************************************
#if !defined DEF_TRACKER
#define DEF_TRACKER

#include <math.h>
#include <assert.h>
#include <string> 
#include <iostream>
#include <fstream>
#include <vector>
#include "Vector3d.h"

using namespace std;

// NY added following 19/01/2010
#include <vhandtk/vhtBase.h>
#include <vhandtk/vhtQuaternion.h>
#include <vtidm/client.h>
#include <vtidm/master.h>

using std::cout;
using std::cerr;
using std::cin;


// #include "ScreenMonitor.h"
#include "S626sManager.h"
#include "Timer626.h"

///////////////////////////////////////////////////////////
/// Trial: Abstract Class with functions that have to be implemented
///////////////////////////////////////////////////////////
class Tracker{
public:
	Tracker();										///< Constructor 
	 ~Tracker();										///< Destructor 
	void init(void);									///< Initialization
	void update(void);								///< Write the header of a data file
	void printState(int r,int row,int col);
	void getVals(void);
	void getQuart(void);
public: 
	Vector3D receiverPos[2]; 
	Vector3D receiverOr[2];
	vhtQuaternion quartern[2];
	

private:
	vht6DofDevice *firstReceiver;
	vht6DofDevice *secondReceiver; 

	vhtIOConn *trackerDict;
	vhtTracker *tracker; 
	vhtTransform3D trackerXForm1;
	vhtTransform3D trackerXForm2;

};

#endif 
