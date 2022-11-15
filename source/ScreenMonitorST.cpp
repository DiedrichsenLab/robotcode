/******************************************************************
** Screen HD 
** Class for Viewport and Windows Management 
*** Screeb monitor for single threaded operation 
*******************************************************************/

#include "ScreenMonitorDR.h"
#include "ScreenDR.h"
#include "Timer626.h" 

#define PI 3.14159265
#include <iostream>
Color_t white={255,255,255};

using namespace std;
#include <windows.h>
#include <process.h>

HWND ScreenMonitor::windowHnd;
WNDCLASSEX ScreenMonitor::wcl;
HDC ScreenMonitor::hDeviceContext; 
HGLRC ScreenMonitor::hRenderContext;


int ScreenMonitor::windowWidth;
int ScreenMonitor::windowHeight;
Vector2D ScreenMonitor::scale; 
double ScreenMonitor::lastTime;
double ScreenMonitor::lastCycle;
void (* ScreenMonitor::updateGraphics)(int);				// Function handle for callback by Experiment 

extern Timer gTimer;

/******************************************************************************
ScreenMonitor Class 
*******************************************************************************/
ScreenMonitor::ScreenMonitor()
{
}

// ------------------------------------------------------------------------------
// Destructor 
// ------------------------------------------------------------------------------
ScreenMonitor::~ScreenMonitor()
{
	deleteRenderContext();
}

//----------------------------------------------------------
// init: initialize the OpenGl and Glut calls
// --------------------------------------------------------
void ScreenMonitor::init(HINSTANCE hInst,HWND hWndParent,int xPos, int yPos, int width,int height,void (*displayFcn)(int))
{
	cout<<"init"<<endl; 
	// copy over all relevant stuff 
	windowWidth=width;
	windowHeight=height;
	windowXPos=xPos;
	windowYPos=yPos;
	updateGraphics=displayFcn;								// Store function handle for callback
	hThisInst=hInst;

	// create Thread 
	create(); 
} 


//----------------------------------------------------------
// Create Window and rendering context 
// --------------------------------------------------------
void ScreenMonitor::create() { 
	cout<<"create"<<endl; 
	wcl.cbSize = sizeof(WNDCLASSEX);
	wcl.hInstance = hThisInst;
	wcl.lpszClassName = "ScreenMonitor";
	wcl.lpfnWndProc = messageCallback;
	wcl.style = 0;
	wcl.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wcl.hIconSm = LoadIcon(NULL, IDI_WINLOGO);
	wcl.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcl.lpszMenuName = NULL;
	wcl.cbClsExtra = 0;
	wcl.cbWndExtra = 0;
	wcl.hbrBackground = (HBRUSH) GetStockObject(WHITE_BRUSH);
	if(!RegisterClassEx(&wcl)) {
		return;
	} 
	windowHnd = CreateWindow(
		"ScreenMonitor",
		"Monitor",
		WS_BORDER,
		windowXPos,
		windowYPos,
		windowWidth,
		windowHeight,
		HWND_DESKTOP,
		NULL,
		hThisInst,
		NULL
	);
	
	if (createRenderContext() ==FALSE){ 
		cerr<<"Rendering Context could not be created"<<endl;
		exit(-1);
	}
	
	ShowWindow(windowHnd, SW_SHOW);
	setProjection();
	UpdateWindow(windowHnd);

}


//----------------------------------------------------------
// Close Screen 
// --------------------------------------------------------
void ScreenMonitor::close(void)
{
	cout << "close Monitor"<<endl;
	PostThreadMessage(lThreadId, WM_CLOSE, 0, 0 );
	do {
	} while (windowHnd!=NULL); 
}

//----------------------------------------------------------
// Rendering Context 
// --------------------------------------------------------
BOOL ScreenMonitor::createRenderContext(void)
{
	cout<<"render"<<endl; 
	int nPixelFormat;
	PIXELFORMATDESCRIPTOR pfd;
	
	
	hDeviceContext = GetDC(windowHnd);
	if ( NULL == hDeviceContext )
		return FALSE;
	
	memset( &pfd, 0, sizeof(PIXELFORMATDESCRIPTOR) );
	pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW
		| PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 32;
	pfd.cDepthBits = 32;
	
	nPixelFormat = ChoosePixelFormat( hDeviceContext, &pfd );
	if ( !nPixelFormat )
		return FALSE;
	
	if ( FALSE == SetPixelFormat( hDeviceContext, nPixelFormat, &pfd ) )
		return FALSE;
	
	hRenderContext = wglCreateContext( hDeviceContext );
	if ( NULL == hRenderContext )
		return FALSE;
	
	if ( FALSE == wglMakeCurrent( hDeviceContext, hRenderContext ) )
		return FALSE;

	return TRUE;
} 

// ------------------------------------------------------------------------------
// Deallocate memory for rendering context
// ------------------------------------------------------------------------------
void ScreenMonitor::deleteRenderContext(){
	if ( NULL != hRenderContext ) {
		wglMakeCurrent( NULL, NULL );
		wglDeleteContext( hRenderContext );
		hRenderContext = NULL;
	} //if
	
	if ( NULL != hDeviceContext ) {
		ReleaseDC( windowHnd, hDeviceContext );
		hDeviceContext = NULL;
	} //if
	
}

// ------------------------------------------------------------------------------
// message Callback: Deals with Windows stuff
// ------------------------------------------------------------------------------
LRESULT CALLBACK ScreenMonitor::messageCallback(HWND hwnd, UINT message,
							WPARAM wParam, LPARAM lParam){
	switch(message)	{
	case WM_DESTROY:
		if (windowHnd!=NULL) { 
			DestroyWindow(windowHnd);
			windowHnd=NULL;
			deleteRenderContext();
		}
		PostQuitMessage(0);
		break;
	case WM_CLOSE: 
		if (windowHnd!=NULL) { 
			DestroyWindow(windowHnd);
			windowHnd=NULL;
			deleteRenderContext();
		}
		PostQuitMessage(0);
		break;
	case WM_PAINT:
		display(hwnd);
		break;
	case WM_SIZE:
		reshape(LOWORD(lParam),HIWORD(lParam));
		break;
	default:
		return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}


// --------------------------------------------------
// Set size of screen (in workspace coordinates) 
void ScreenMonitor::setScale(Vector2D s){ 
	scale=s;
	reshape(windowWidth,windowHeight);
	setProjection(); 
} 


// ------------------------------------------------------------------------------
// setProjection: Set projection geometry for eye i
// ------------------------------------------------------------------------------
void ScreenMonitor::setProjection(void) {
	
	glViewport(0, 0, windowWidth, windowHeight);
    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D((GLdouble) (Screen::center[0]-windowWidth*ScreenMonitor::scale[0]/2),
		(GLdouble) (Screen::center[0]+windowWidth*ScreenMonitor::scale[0]/2),
		(GLdouble) (Screen::center[1]-windowHeight*ScreenMonitor::scale[1]/2),
		(GLdouble) (Screen::center[1]+windowHeight*ScreenMonitor::scale[1]/2));
	glMatrixMode(GL_MODELVIEW);			// Necessary, otherwise unpredictable results 
	
	glLoadIdentity();
}

//----------------------------------------------------------------
// displayFcn Callback 
// ---------------------------------------------------------------
void ScreenMonitor::display(HWND hwnd) { 
		
		setProjection(); 
		lastCycle=gTimer[0]-lastTime;
		lastTime=gTimer[0];
	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);   
		
	/*	glPushMatrix();
		glColor3ubv(white);
		glBegin(GL_LINES);
		glVertex2f(0,0);
		glVertex2f(1,0);
		glEnd(); 
		glPopMatrix();
*/
		(*updateGraphics)(5);
		SwapBuffers( hDeviceContext );
		ValidateRect( windowHnd, NULL );
} 

// ------------------------------------------------------------------------------
// reshape Callback function 
// ------------------------------------------------------------------------------
void ScreenMonitor::reshape(int width, int height) { 

	windowWidth=width;
	windowHeight=height;
	setProjection();
} 


