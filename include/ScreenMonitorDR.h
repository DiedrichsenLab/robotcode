//////////////////////////////////////////////////////////////
/// ScreenMonitor.h 
/// Implements a viewport for the experimenter to see what the subject is doing 
///   2007, Joern Diedrichsen 
///   j.diedrichsen@bangor.ac.uk
//////////////////////////////////////////////////////////////

#ifndef SCREEN_MONITOR
#define SCREEN_MONITOR
#include <windows.h>
#include <stdlib.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include "Vector2D.h" 


using namespace std;


//////////////////////////////////////////////////////////////
/// Implements a viewport for the experimenter to see what the subject is doing 
/// ScreenMonitor runs in it's own thread 
//////////////////////////////////////////////////////////////
class ScreenMonitor {
public:
	ScreenMonitor();					///< Constructor 
	~ScreenMonitor();					///< Destructor 
	void init(HINSTANCE hinst,HWND parentWindow,int xPos, int yPos, int height,int width,void (*displayCallback)(int)); ///< initialize and open new thread
	void create(void);							///< Generate the window: called from the thread.
	
	void setScale(Vector2D scale);							// Scaling factor of pixel to coordinate system

	static BOOL createRenderContext( void );	///< Render context needs to be created 
	static void deleteRenderContext( void );		
	static void setProjection();				///< set projection 
	static void reshape(int w,int h);			///<  reshape callback 
	static void display(HWND hwnd);				///< display callback: calls currentTrial->updateGraphics(2)
	void close();								///< close the whole thing and thread 
	int MessageLoop();

public: 
	static HWND windowHnd;						///< Window Handle 
	HWND parentHnd;
	static WNDCLASSEX wcl;
	static LRESULT CALLBACK messageCallback(HWND, UINT, WPARAM, LPARAM);
	static HDC hDeviceContext; 
	static HGLRC hRenderContext;
	HANDLE hThread;								///< Thread Handle 
	unsigned int lThreadId;						///< Thread ID
	HINSTANCE hThisInst;
public: // colors and other static members 
	static int windowWidth;
	static int windowHeight;
	static Vector2D scale;			///< What is the size of a pixel in workspace coordinates?

	int windowXPos;
	int windowYPos;
	static void (*updateGraphics)(int);				///< Function handle for callback to experiment: calls currentTrial->updateGraphics(2)
	static double lastTime;							///< time of last update 
	static double lastCycle;						///< ms in last cycle 
};

unsigned int __stdcall ScreenMonitorThreadFunc( LPVOID lpParam );
#endif


