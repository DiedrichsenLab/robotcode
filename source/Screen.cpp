/**********************************************************************************
**  Screen library for 2D-presentation 
**********************************************************************************/
#include "Screen.h"
#define PI 3.14159265
#include <iostream>
#include "Timer626.h"


using namespace std;
#include <windows.h>

HWND Screen::windowHnd;
WNDCLASSEX Screen::wcl;
HDC Screen::hDeviceContext; 
HGLRC Screen::hRenderContext;
bool Screen::keyPressed=0;
char Screen::key;

double Screen::lastTime;
double Screen::lastCycle;

int Screen::windowWidth;
int Screen::windowHeight;
Vector2D Screen::center;
Vector2D Screen::scale; 
void (* Screen::updateGraphics)(int);	

bool Screen::isLocked;
extern Timer gTimer; 


/***************************************************************
glLists for fast drawing of basic geometric shapes.
***************************************************************/
int Screen::circlelist;
int Screen::rectlist;
int Screen::disklist;
int Screen::boxlist;
int Screen::hexlist;
int Screen::hexFlist;


/****************************************************
Basic colors:
****************************************************8*/
Color_t Screen::black={0,0,0};				///< BLACK: 0 
Color_t Screen::white={255,255,255};		///< 1 
Color_t Screen::red={200,0,0};				///< 2 
Color_t Screen::green={0,220,100};			///< 3 
Color_t Screen::blue={20,20,255};			///< 4 
Color_t Screen::grey={180,180,180};			///< 5 
Color_t Screen::salmon={255,160,122};		///< 6
Color_t Screen::yellow={255,255,0};			///< 7
Color_t Screen::pale={80,60,0};				///< 8
Color_t Screen::lightblue={100,100,255};	///< 9
Color_t Screen::purple={100,0,100};         ///< 10
Color_t Screen::darkorange={255,140,0};     ///< 11
Color_t Screen::darkyellow={102,102,0};     ///< 12
Color_t Screen::oceanblue={30,110,255};	    ///< 13 
Color_t Screen::forestgreen={34,102,34};	///< 14
Color_t Screen::darkred={100,0,0};			///< 15
Color_t Screen::darkblue={0,0,100};			///< 16
Color_t Screen::darkgrey={105,105,105};     ///< 17


/******************************************************************************
Constructor
*******************************************************************************/

Screen::Screen()
{
	isLocked=1;
	
}

// ------------------------------------------------------------------------------
// Destructor 
// ------------------------------------------------------------------------------
Screen::~Screen()
{
	deleteRenderContext();
}

// ------------------------------------------------------------------------------
// init
// ------------------------------------------------------------------------------
/* Windows screen*/
void Screen::init(HINSTANCE hThisInst,int xPos, int yPos, int width,int height,void (*displayFcn)(int))
{
	windowWidth=width;
	windowHeight=height;
	scale=Vector2D(1.0,1.0);		// Scaling of the screen is in pixel 
	center=Vector2D(0,0);			// Center of the screen is (0,0) 	
	
	wcl.cbSize = sizeof(WNDCLASSEX);
	wcl.hInstance = hThisInst;
	wcl.lpszClassName = "SubjectScreen";
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
	updateGraphics=displayFcn;								// Store function handle for callback

	windowHnd = CreateWindow(
		"SubjectScreen",
		"Screen",
		WS_POPUP,
		xPos,
		yPos,
		width,
		height,
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

	
	glClearColor(0.0, 0.0, 0.0, 0.0); 
	glClearDepth(0.0); 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel (GL_FLAT);
	
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
	
	glLoadIdentity();
	initShapes();
	
	gluOrtho2D(Screen::center[0]-windowWidth*Screen::scale[0]/2,
		Screen::center[0]+windowWidth*Screen::scale[0]/2,
		Screen::center[1]-windowHeight*Screen::scale[1]/2,
		Screen::center[1]+windowHeight*Screen::scale[1]/2);

	UpdateWindow(windowHnd);
} 


//----------------------------------------------------------
// Close Screen 
// --------------------------------------------------------
void Screen::close(void)
{
	cout << "close Screen"<<endl;

	if (windowHnd!=NULL) { 
		DestroyWindow(windowHnd);
		windowHnd=NULL;
		deleteRenderContext();
	}
}

//----------------------------------------------------------
// Create Context 
// --------------------------------------------------------
BOOL Screen::createRenderContext(void)
{
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
void Screen::deleteRenderContext(){
	cout<<"delete REnder"<<endl;
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
LRESULT CALLBACK Screen::messageCallback(HWND hwnd, UINT message,
							WPARAM wParam, LPARAM lParam){
	switch(message)	{
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_PAINT:
		display(hwnd);
		break;
	default:
		return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}

//----------------------------------------------------------------
// displayFcn Callback 
// ---------------------------------------------------------------
void Screen::display(HWND hwnd) { 
		lastCycle=gTimer[0]-lastTime;
		lastTime=gTimer[0];
	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);      
		(*updateGraphics)(0);
		SwapBuffers( hDeviceContext );
		ValidateRect( windowHnd, NULL );	
} 

// ------------------------------------------------------------------------------
// Calculate the positions of glvertices for making geometry display lists
// ------------------------------------------------------------------------------
int Screen::renderCircle(int listIndex, GLfloat lineWidth)
{
	GLint i;
	GLfloat cosine, sine;
	glNewList(listIndex, GL_COMPILE);
	if (lineWidth==-1.0f) 
		glBegin(GL_POLYGON);
	else {
		
		glBegin(GL_LINE_LOOP);
	}
	for(i=0;i<180;i++){
		cosine=cos((double)i*2*PI/180.0);
		sine=sin((double)i*2*PI/180.0);
		glVertex2f(.5*cosine,.5*sine);
	}
	glEnd();
	glEndList();
	return 0;
}



// ------------------------------------------------------------------------------
// Calculate the positions of glvertices for making geometry display lists
// ------------------------------------------------------------------------------
int Screen::renderRect(int listIndex,GLfloat width, GLfloat height, GLfloat lineWidth)
{
	glNewList(listIndex,GL_COMPILE);
	if (lineWidth==-1.0f)glBegin(GL_POLYGON);
	else{	
		glBegin(GL_LINE_LOOP);
	}	
	glVertex2f(-width/2,-height/2);
	glVertex2f(width/2,-height/2);
	glVertex2f(width/2,height/2);
	glVertex2f(-width/2,height/2);
	glEnd();
	glEndList();
	return 0;
}


// ------------------------------------------------------------------------------
// Calculate the positions of glvertices for making geometry display lists
// ------------------------------------------------------------------------------
int Screen::renderHex(int listIndex,GLfloat width, GLfloat lineWidth)
{
	glNewList(listIndex,GL_COMPILE);
	if (lineWidth==-1.0f)glBegin(GL_POLYGON);
	else{	
		glBegin(GL_LINE_LOOP);
	}
	glVertex2f(-width/2*sin(30.0*PI/180.0),width/2*cos(30.0*PI/180.0));
	glVertex2f(-width/2,0);
	glVertex2f(-width/2*sin(30.0*PI/180.0),-width/2*cos(30.0*PI/180.0));
	glVertex2f(width/2*sin(30.0*PI/180.0),-width/2*cos(30.0*PI/180.0));
	glVertex2f(width/2,0);
	glVertex2f(width/2*sin(30.0*PI/180.0),width/2*cos(30.0*PI/180.0));
	glEnd();
	glEndList();
	return 0;
}


// ------------------------------------------------------------------------------
// Fast drawing routines for drawing simple shapes
// ------------------------------------------------------------------------------


void Screen::setColor(int color){ 
	switch (color) { 
	case 0:
		setColor(black);
		break;
	case 1: 
		setColor(white);
		break;
	case 2: 
		setColor(red);
		break;
	case 3: 
		setColor(green);
		break;
	case 4: 
		setColor(blue);
		break;
	case 5: 
		setColor(grey);
		break;
	case 6: 
		setColor(salmon);
		break;
	case 7: 
		setColor(yellow); 
		break;
	case 8:
		setColor(pale);
		break;
	case 9:
		setColor(lightblue);
		break;
	case 17:
		setColor(darkgrey);
		break;
	} 

	
} 

void Screen::drawDisk(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{
	glPushMatrix();
	glTranslatef(xpos,ypos,0.0f);
	glScalef(xsize,ysize,1.0f);
	glCallList(disklist);
	glPopMatrix();
}

void Screen::drawCircle(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{
	glPushMatrix();
	glTranslatef(xpos,ypos,0.0f);
	glScalef(xsize,ysize,1.0f);
	glCallList(circlelist);
	glPopMatrix();
}

void Screen::drawCircle(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos,int linewidth)
{
	glPushMatrix();
		glTranslatef(xpos,ypos,0.0f);
		glPushMatrix();
			glScalef(xsize,ysize,1.0f);
			glCallList(circlelist);
		glPopMatrix(); 
		glPushMatrix();
			glScalef(xsize-scale[0],ysize-scale[1],1.0f);
			glCallList(circlelist);
		glPopMatrix(); 
		glPushMatrix();
			glScalef(xsize+scale[0],ysize+scale[1],1.0f);
			glCallList(circlelist);
		glPopMatrix(); 
	glPopMatrix();
}

/////////////////////////////////////////////////////////////////////////////
/// Draws a segement from startang to endang (in degrees, 0 is up) 
/// Does not use a precompiled display list
/// \param size x and y size of the ellipse
/// \param position in workspace 
/// \param startang angle to start at (goes clockwise)
/// \param endend angle to end with 
///////////////////////////////////////////////////////////////////////////// 
void Screen::drawSegment(Vector2D size, Vector2D pos,float startang, float endang) {
	GLfloat i;
	GLfloat cosine, sine;
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
		
	glBegin(GL_LINES);
	for(i=startang;i<endang;i+=2.0){
		cosine=cos((double)i*PI/180.0);
		sine=sin((double)i*PI/180.0);
		glVertex2f(size[0]*sine,size[0]*cosine);
	}
	glEnd();
	glPopMatrix(); 
	return;
}


/////////////////////////////////////////////////////////////////////////////
/// Draws a rectangle on the screen.  
/// \param xsize width of the rectangle 
/// \param ysize height of the rectangle 
/// \param xpos position of the center of the rectangle 
/// \param ypos position of the center of the rectangle 
///////////////////////////////////////////////////////////////////////////// 
void Screen::drawRect(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{	
	glPushMatrix();
	glTranslatef(xpos,ypos,0.0f);
	glScalef(xsize,ysize,1.0f);
	glCallList(rectlist);
	glPopMatrix();
}


void Screen::drawBox(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{
	glPushMatrix();
	glScalef(xsize,ysize,1.0f);
	glTranslatef(xpos/xsize,ypos/ysize,0.0f);
	glCallList(boxlist);
	glPopMatrix();
}

void Screen::drawLine(GLfloat xposStart,GLfloat yposStart, GLfloat xposEnd,GLfloat yposEnd,GLfloat width)
{ 
	glPushMatrix();
	glBegin(GL_LINES);
		//glLineWidth(width);	/// Unfortunately, different line width are not supported under this version of OpenGL
		glVertex2f(xposStart,yposStart);
		glVertex2f(xposEnd,yposEnd);
	glEnd(); 
	glPopMatrix();
} 


void Screen::drawCircle(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(circlelist);
	glPopMatrix();
}

void Screen::drawDisk(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(disklist);
	glPopMatrix();
}

void Screen::drawRect(Vector2D size, Vector2D pos)
{	
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(rectlist);
	glPopMatrix();
}


void Screen::drawBox(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(boxlist);
	glPopMatrix();
}

void Screen::drawLine(Vector2D start,Vector2D end)
{ 
	glPushMatrix();
	glBegin(GL_LINES);
		glVertex2f(start[0],start[1]);
		glVertex2f(end[0],end[1]);
	glEnd();
	glPopMatrix();
} 

void Screen::drawArrow(Vector2D start,Vector2D end,double lengthHead)
{ 
	double a=0.3;
	Matrix2D rot(cos(a),-sin(a),sin(a),cos(a));
	Vector2D main=start-end;
	main=main/main.norm()*lengthHead; 
	Vector2D arrL=rot*main+end;
	Vector2D arrR=rot.getTranspose()*main+end;
	glPushMatrix();
	glBegin(GL_LINES);
		glVertex2f(start[0],start[1]);
		glVertex2f(end[0],end[1]);
	glEnd();
	glBegin(GL_LINES);
		glVertex2f(end[0],end[1]);
		glVertex2f(arrL[0],arrL[1]);
	glEnd();
	glBegin(GL_LINES);
		glVertex2f(end[0],end[1]);
		glVertex2f(arrR[0],arrR[1]);
	glEnd();
	glPopMatrix();
} 

void Screen::drawPlus(Vector2D size,Vector2D pos)
{ 
	glPushMatrix();
	glBegin(GL_LINES);
		glVertex2f(pos[0]-size[0],pos[1]);
		glVertex2f(pos[0]+size[0],pos[1]);
		glVertex2f(pos[0],pos[1]-size[1]);
		glVertex2f(pos[0],pos[1]+size[1]);
	glEnd();
	glPopMatrix();
} 

void Screen::drawHex(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(hexlist);
	glPopMatrix();
}

void Screen::drawHexF(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(hexFlist);
	glPopMatrix();
}


/*******************************************
initialize the geometric shape lists to
an unused glList ID
********************************************/

int Screen::initShapes()
{
	circlelist=glGenLists(1);
	disklist=glGenLists(1);
	rectlist=glGenLists(1);
	boxlist=glGenLists(1);
	hexlist=glGenLists(1);
	hexFlist=glGenLists(1);
	renderCircle(circlelist,1.0f);
	renderRect(rectlist,1.0f,1.0f,1.0f);
	renderCircle(disklist,-1.0f);
	renderRect(boxlist,1.0f,1.0f,-1.0f);
	renderHex(hexlist,1.0f,1.0f);
	renderHex(hexFlist,1.0f,-1.0f);
	return 0;
	
}

/******************************************************
Print string s and point (x,y) workspace coordinates with magnification.
magnification=0 will print nothing.
******************************************************/
void Screen::print(string text, double x, double y, GLfloat magnification)
{
	int c;
	double length=0; 
	glPushMatrix();
	glTranslatef(x, y, 1);
	glScalef(0.001*magnification,0.001*magnification,1.0);
// GLUT_STROKE_MONO_ROMAN
	for (c=0;c<text.length();c++) { 
		length+=glutStrokeWidth(GLUT_STROKE_ROMAN,text[c]);
	} 
	glTranslatef(-length/2, 0, 0);
	for (c=0;c<text.length();c++) { 
		glutStrokeCharacter(GLUT_STROKE_ROMAN, text[c]);
	} 
	glPopMatrix();
} 

/******************************************************
Print string s and point (x,y) workspace coordinates with magnification.
magnification=0 will print nothing. Flip -1 will flip the text
******************************************************/
void Screen::print(string text, double x, double y, GLfloat magnification, GLfloat flip)
{
	int c;
	double length=0; 
	glPushMatrix();
	glTranslatef(x, y, 1);
	glScalef(0.001*magnification,0.001*magnification,1.0);
	glScalef(flip,1.0,1.0);
// GLUT_STROKE_MONO_ROMAN
	for (c=0;c<text.length();c++) { 
		length+=glutStrokeWidth(GLUT_STROKE_ROMAN,text[c]);
	} 
	glTranslatef(-length/2, 0, 0);
	for (c=0;c<text.length();c++) { 
		glutStrokeCharacter(GLUT_STROKE_ROMAN, text[c]);
	} 
	glPopMatrix();
} 

/******************************************************
Print string s and point (x,y) workspace coordinates with magnification.
magnification=0 will print nothing.
******************************************************/
void Screen::printChar(char text, double x, double y, GLfloat magnification)
{
	double length=0; 
	glPushMatrix();
	glTranslatef(x, y, 1);
	glScalef(0.001*magnification,0.001*magnification,1.0);
// GLUT_STROKE_MONO_ROMAN
	length=glutStrokeWidth(GLUT_STROKE_ROMAN,text);
	glTranslatef(-length/2, 0, 0);
	glutStrokeCharacter(GLUT_STROKE_ROMAN, text);
	glPopMatrix();
} 


/***************************************
Clear screen to black.
***************************************/

void Screen::clear(){
}

/**************************************
Get ans set of private size data members.
**************************************/
void Screen::getSizePixel(int &width, int &height)
{
	height=windowHeight;
	width=windowWidth;
}

// -------------------------------------------------
// Return workspace coordinate for center of screen  
Vector2D Screen::getCenter(void){
	return center;
}


// -------------------------------------------------
// Set workspace center of workspace coordinates 
void Screen::setCenter(Vector2D cent){
	center=cent;
	reshape(windowWidth,windowHeight);
}

// --------------------------------------------------
// Return the size (in workspace coordinates) 
Vector2D Screen::getSize(void){ 
	return Vector2D(windowWidth*scale[0],windowHeight*scale[1]);
} 


// --------------------------------------------------
// Set size of screen (in workspace coordinates) 
void Screen::setScale(Vector2D s){ 
	scale=s;
	reshape(windowWidth,windowHeight);
} 

// --------------------------------------------------
// switches between setups (L/R reversal)
void Screen::switchSetup(void){ 
	scale=Vector2D(-scale.x[0],scale.x[1]);
	reshape(windowWidth,windowHeight);
} 

/******************************************************
reshape: needs to be static, so that we can use C-style call
*******************************************************/
void Screen::reshape(int w,int h){ 
	Screen::windowWidth=w;
	Screen::windowHeight=h;
	glViewport(0,0,(GLsizei) w,(GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D((GLdouble) (Screen::center[0]-Screen::windowWidth*Screen::scale[0]/2),
		(GLdouble) (Screen::center[0]+Screen::windowWidth*Screen::scale[0]/2),
		(GLdouble) (Screen::center[1]-Screen::windowHeight*Screen::scale[1]/2),
		(GLdouble) (Screen::center[1]+Screen::windowHeight*Screen::scale[1]/2));
	glMatrixMode(GL_MODELVIEW);			// Necessary, otherwise unpredictable results 
	glLoadIdentity();
}

/******************************************************
Keyboard function: Sets keypressed varaible 
*******************************************************/
void Screen::onKey(unsigned char k,int x, int y){ 
	keyPressed=1;
	key=k;
} 

/******************************************************
 Set Projection 
*******************************************************/
void Screen::setProjection(){	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(Screen::center[0]-windowWidth*Screen::scale[0]/2,
		Screen::center[0]+windowWidth*Screen::scale[0]/2,
		Screen::center[1]-windowHeight*Screen::scale[1]/2,
		Screen::center[1]+windowHeight*Screen::scale[1]/2);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity ();
} 