/**********************************************************************************
**  Screen library for 2D-presentation 
**********************************************************************************/
#include "Screen3D.h"
#define PI 3.14159265
#include <iostream>
#include "Timer626.h"


using namespace std;
#include <windows.h>

HWND Screen3D::windowHnd;
WNDCLASSEX Screen3D::wcl;
HDC Screen3D::hDeviceContext; 
HGLRC Screen3D::hRenderContext;
bool Screen3D::keyPressed=0;
char Screen3D::key;

double Screen3D::lastTime;
double Screen3D::lastCycle;

int Screen3D::windowWidth;
int Screen3D::windowHeight;
Vector2D Screen3D::center;
Vector2D Screen3D::scale; 
void (* Screen3D::updateGraphics)(int);	

bool Screen3D::isLocked;
extern Timer gTimer; 

/***************************************************************
glLists for fast drawing of basic geometric shapes.
***************************************************************/
int Screen3D::circlelist;
int Screen3D::rectlist;
int Screen3D::disklist;
int Screen3D::boxlist;


/****************************************************
Basic colors:
****************************************************8*/
Color_t Screen3D::black={0,0,0};				///< BLACK: 0 
Color_t Screen3D::white={255,255,255};		///< 1 
Color_t Screen3D::red={200,0,0};				///< 2 
Color_t Screen3D::green={0,220,100};			///< 3 
Color_t Screen3D::blue={20,20,255};			///< 4 
Color_t Screen3D::grey={180,180,180};			///< 5 
Color_t Screen3D::salmon={255,160,122};		///< 6
Color_t Screen3D::yellow={255,255,0};			///< 7
Color_t Screen3D::pale={80,60,0};				///< 8
Color_t Screen3D::lightblue={100,100,255};	///< 9
Color_t Screen3D::darkgray={50,50,50};		///< 10
Color_t Screen3D::darkgreen={0,180,0};		///< 11
Color_t Screen3D::darkgray2={30,30,30};		///< 12

/******************************************************************************
Constructor
*******************************************************************************/

Screen3D::Screen3D()
{
	isLocked=1;
	
}

// ------------------------------------------------------------------------------
// Destructor 
// ------------------------------------------------------------------------------
Screen3D::~Screen3D()
{
	deleteRenderContext();
}

// ------------------------------------------------------------------------------
// init
// ------------------------------------------------------------------------------
/* Windows Screen3D*/
void Screen3D::init(HINSTANCE hThisInst,int xPos, int yPos, int width,int height,void (*displayFcn)(int))
{
	windowWidth=width;
	windowHeight=height;
	scale=Vector2D(1.0,1.0);		// Scaling of the Screen3D is in pixel 
	//center=Vector2D(0,0);			// Center of the Screen3D is (0,0) 	
	center=Vector2D(0,0);	

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

	initLighting(); 
	setProjection(); 

	UpdateWindow(windowHnd);
} 


//----------------------------------------------------------
// SetIOD
// --------------------------------------------------------
void Screen3D::initLighting(){
	static const GLfloat lightModelAmbient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    static const GLfloat light0Diffuse[] = {0.9f, 0.9f, 0.9f, 0.9f};   
    static const GLfloat light0Direction[] = {0.1f, 1.0f, 0.4f, 0.0f};    
    static const GLfloat matSpecular[] = {1.0f, 1.0f, 1.0f, 1.0f};    
    static const GLfloat matShininess[] = {50.0f};    
    static const GLfloat whiteLight[] = {1.0f, 1.0f, 1.0f, 1.0f};    

	// Enable depth buffering for hidden surface removal.
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);

    // Cull back faces.
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    
    
    // Setup lighting model.
  	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lightModelAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0Diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, whiteLight);
    glLightfv(GL_LIGHT0, GL_POSITION, light0Direction);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

	// Common Material properties 
	glMaterialfv(GL_FRONT, GL_SPECULAR,matSpecular); 
	glMaterialfv(GL_FRONT, GL_SHININESS,matShininess);
	glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	UpdateWindow(windowHnd);
	
} 

//----------------------------------------------------------
// Rendering Context 
// --------------------------------------------------------
BOOL Screen3D::createRenderContext(void)
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
void Screen3D::deleteRenderContext(){
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


//----------------------------------------------------------
// Close Screen3D 
// --------------------------------------------------------
void Screen3D::close(void)
{
	cout << "close Screen3D"<<endl;

	if (windowHnd!=NULL) { 
		DestroyWindow(windowHnd);
		windowHnd=NULL;
		deleteRenderContext();
	}
}


// ------------------------------------------------------------------------------
// message Callback: Deals with Windows stuff
// ------------------------------------------------------------------------------
LRESULT CALLBACK Screen3D::messageCallback(HWND hwnd, UINT message,
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
void Screen3D::display(HWND hwnd) { 
		lastCycle=gTimer[0]-lastTime;
		lastTime=gTimer[0];
		setProjection(); 
	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);      
		(*updateGraphics)(0);
		SwapBuffers( hDeviceContext );
		ValidateRect( windowHnd, NULL );	
} 

// ------------------------------------------------------------------------------
// Calculate the positions of glvertices for making geometry display lists
// ------------------------------------------------------------------------------
int Screen3D::renderCircle(int listIndex, GLfloat lineWidth)
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
int Screen3D::renderRect(int listIndex,GLfloat width, GLfloat height, GLfloat lineWidth)
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




void Screen3D::drawSphere(Vector3D where,double size)
{
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
    glPushMatrix();

	glTranslated(where[0],where[1],where[2]);

    glEnable(GL_COLOR_MATERIAL);
    glColor3f(1.0, 1.0, 0.0);

    glutSolidSphere(size,32,32);
    glPopMatrix();
    glPopAttrib();
}

void Screen3D::drawBox(Vector3D size){ // Draws a cube at the current local coordinate  
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
    glPushMatrix();


    glEnable(GL_COLOR_MATERIAL);
    glColor3f(1.0, 1.0, 0.0);

    glScaled(size[0],size[1],size[2]); 
	glutSolidCube(1);
    glPopMatrix();
    glPopAttrib();
}



// ------------------------------------------------------------------------------
// Fast drawing routines for drawing simple shapes
// ------------------------------------------------------------------------------


void Screen3D::setColor(int color){ 
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
	case 10:
		setColor(darkgray);
		break;
	case 11:
		setColor(darkgreen);
		break;
	case 12:
		setColor(darkgray2);
		break;

	} 
} 

void Screen3D::drawDisk(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{
	glPushMatrix();
	glTranslatef(xpos,ypos,0.0f);
	glScalef(xsize,ysize,1.0f);
	glCallList(disklist);
	glPopMatrix();
}

void Screen3D::drawCircle(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{
	glPushMatrix();
	glTranslatef(xpos,ypos,0.0f);
	glScalef(xsize,ysize,1.0f);
	glCallList(circlelist);
	glPopMatrix();
}

void Screen3D::drawCircle(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos,int linewidth)
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
void Screen3D::drawSegment(Vector2D size, Vector2D pos,float startang, float endang) {
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
/// Draws a rectangle on the Screen3D.  
/// \param xsize width of the rectangle 
/// \param ysize height of the rectangle 
/// \param xpos position of the center of the rectangle 
/// \param ypos position of the center of the rectangle 
///////////////////////////////////////////////////////////////////////////// 
void Screen3D::drawRect(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{	
	glPushMatrix();
	glTranslatef(xpos,ypos,0.0f);
	glScalef(xsize,ysize,1.0f);
	glCallList(rectlist);
	glPopMatrix();
}


void Screen3D::drawBox(GLfloat xsize, GLfloat ysize, GLfloat xpos, GLfloat ypos)
{
	glPushMatrix();
	glScalef(xsize,ysize,1.0f);
	glTranslatef(xpos/xsize,ypos/ysize,0.0f);
	glCallList(boxlist);
	glPopMatrix();
}

void Screen3D::drawLine(GLfloat xposStart,GLfloat yposStart, GLfloat xposEnd,GLfloat yposEnd,GLfloat width)
{ 
	glPushMatrix();
	glBegin(GL_LINES);
		// glLineWidth(width);	/// Unfortunately, different line width are not supported under this version of OpenGL
		glVertex2f(xposStart,yposStart);
		glVertex2f(xposEnd,yposEnd);
	glEnd(); 
	glPopMatrix();
} 


void Screen3D::drawCircle(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(circlelist);
	glPopMatrix();
}

void Screen3D::drawDisk(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(disklist);
	glPopMatrix();
}

void Screen3D::drawRect(Vector2D size, Vector2D pos)
{	
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(rectlist);
	glPopMatrix();
}


void Screen3D::drawBox(Vector2D size, Vector2D pos)
{
	glPushMatrix();
	glTranslatef(pos[0],pos[1],0.0f);
	glScalef(size[0],size[1],1.0f);
	glCallList(boxlist);
	glPopMatrix();
}

void Screen3D::drawLine(Vector2D start,Vector2D end)
{ 
	glPushMatrix();
	glBegin(GL_LINES);
		glVertex2f(start[0],start[1]);
		glVertex2f(end[0],end[1]);
	glEnd();
	glPopMatrix();
} 

void Screen3D::drawArrow(Vector2D start,Vector2D end,double lengthHead)
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

void Screen3D::drawPlus(Vector2D size,Vector2D pos)
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


/*******************************************
initialize the geometric shape lists to
an unused glList ID
********************************************/

int Screen3D::initShapes()
{
	circlelist=glGenLists(1);
	disklist=glGenLists(1);
	rectlist=glGenLists(1);
	boxlist=glGenLists(1);
	renderCircle(circlelist,1.0f);
	renderRect(rectlist,1.0f,1.0f,1.0f);
	renderCircle(disklist,-1.0f);
	renderRect(boxlist,1.0f,1.0f,-1.0f);
	return 0;
	
}

/******************************************************
Print string s and point (x,y) workspace coordinates with magnification.
magnification=0 will print nothing.
******************************************************/
void Screen3D::print(string text, double x, double y, GLfloat magnification)
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
magnification=0 will print nothing.
******************************************************/
void Screen3D::printChar(char text, double x, double y, GLfloat magnification)
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
Clear Screen3D to black.
***************************************/

void Screen3D::clear(){
}

/**************************************
Get ans set of private size data members.
**************************************/
void Screen3D::getSizePixel(int &width, int &height)
{
	height=windowHeight;
	width=windowWidth;
}

// -------------------------------------------------
// Return workspace coordinate for center of Screen3D  
Vector2D Screen3D::getCenter(void){
	return center;
}


// -------------------------------------------------
// Set workspace center of workspace coordinates 
void Screen3D::setCenter(Vector2D cent){
	center=cent;
	reshape(windowWidth,windowHeight);
}

// --------------------------------------------------
// Return the size (in workspace coordinates) 
Vector2D Screen3D::getSize(void){ 
	return Vector2D(windowWidth*scale[0],windowHeight*scale[1]);
} 


// --------------------------------------------------
// Set size of Screen3D (in workspace coordinates) 
void Screen3D::setScale(Vector2D s){ 
	scale=s;
	reshape(windowWidth,windowHeight);
} 

/******************************************************
reshape: needs to be static, so that we can use C-style call
*******************************************************/
void Screen3D::reshape(int w,int h){ 
	windowWidth=w;
	windowHeight=h;
	setProjection();

}

/******************************************************
Keyboard function: Sets keypressed varaible 
*******************************************************/
void Screen3D::onKey(unsigned char k,int x, int y){ 
	keyPressed=1;
	key=k;
} 

/******************************************************
 Set Projection 
*******************************************************/
void Screen3D::setProjection(){	
   glViewport(0, 0, windowWidth, windowHeight);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity (); 
		gluPerspective(100.0,1,1,1000);

	/*	double clipNear=2.0; 
		double clipFar=100; 
		double viewportWidthcm=(double)windowWidth/94*2.54/2;
		double viewportHeightcm=(double)windowHeight/94*2.54;
        double fLeft =  ((-viewportWidthcm/2.0) ) * clipNear/MonitorDistance;  
        double fRight = ((viewportWidthcm/2.0) - XPos) * clipNear/MonitorDistance;        
        double fTop = (((viewportHeightcm/2.0)*clipNear/MonitorDistance));
        double fBottom = -fTop;
		
		
        glFrustum(eye[i].fLeft, eye[i].fRight, eye[i].fBottom, eye[i].fTop, 2, 100);  
        /* translate scene. Note you move the scene not the "camera" in openGL (which is always at 0,0).
                So here we translate the scene by the inverse of the eye position shift (i.e. for the left eye, the scene moves right). */          
        // This is for screen capture: 
        glMatrixMode(GL_MODELVIEW);
		glLoadIdentity ();
        gluLookAt (0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

} 