// -------------------------------------------------------------------------------
// Screen Object 
// Goal of rgl library: provide simplified drawing of polygons, simplified color selection, 
// and simplified initialization of glut.
// --------------------------------------------------------------------------------

#ifndef SCREEN_DEF
#define SCREEN_DEF
#include <windows.h>
#include <stdlib.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include "Vector2D.h" 
#include "Matrix2D.h" 


using namespace std;

typedef GLubyte Color_t[3];
#define SCREENNUMCOLORS 9    ///< Number Colors in color list 

/****************************************************
Screen class:
****************************************************8*/
class Screen {
public:
	Screen();
	~Screen();
	void init(HINSTANCE hThisInst,int xPos, int yPos, int width,int height,void (*displayFcn)(int));
	BOOL createRenderContext( void );
	void deleteRenderContext( void );

	void clear();
	void lock();
	void unlocked();

	void getSizePixel(int &height, int &width);				// Return the size of the screen in pixel 
	Vector2D getCenter(void);								// Return workspace coordinate for center of screen  
	void setCenter(Vector2D size);							// Set center of screen in workspace coordinates 
	Vector2D getSize(void);									// Return the size (in workspace coordinates) 
	Vector2D getScale(void);								// Scaling (size of one pixel in Workspace coordinates)
	void setScale(Vector2D scale);							// Scaling factor of pixel to coordinate system

	// Static callback functions 
	static void reshape(int w,int h);			// Static callback functions 
	static void display(HWND hwnd);	
	static void onKey(unsigned char k,int x, int y);// Static callback function 
	static void setProjection(); 

	// Drawing routines 

	void setColor(int theColor);
	void setColor(Color_t theColor){glColor3ubv(theColor);} 
	void drawCircle(GLfloat xsize,GLfloat ysize,GLfloat x,GLfloat y);
	void drawRing(GLfloat xsize,GLfloat ysize, GLfloat x,GLfloat y);
	void drawRing(GLfloat xsize,GLfloat ysize, GLfloat x,GLfloat y,int linewidth);
	void drawRect(GLfloat xsize,GLfloat ysize, GLfloat xpos,GLfloat ypos);
	void drawBox(GLfloat xsize,GLfloat ysize, GLfloat xpos,GLfloat ypos);
	void drawLine(GLfloat xposStart,GLfloat yposStart, GLfloat xposEnd,GLfloat yposEnd,GLfloat width=1);
	void print(string s, double x, double y, GLfloat magnification);
	void printChar(char s, double x, double y, GLfloat magnification);
	
	// Drawing using vectors 
	void drawCircle(Vector2D size, Vector2D pos);
	void drawRing(Vector2D size, Vector2D pos);
	void drawRect(Vector2D size, Vector2D pos);
	void drawBox(Vector2D size, Vector2D pos);
	void drawLine(Vector2D start, Vector2D end);
	void drawArrow(Vector2D start, Vector2D end, double headLength); 
	void drawPlus(Vector2D size,Vector2D pos);
	inline void print(string s, Vector2D pos, GLfloat magnification)
					{print(s,pos[0],pos[1],magnification);}
	void close();	
	


public: // colors and other static members 
	static HWND windowHnd;
	static WNDCLASSEX wcl;
	static LRESULT CALLBACK messageCallback(HWND, UINT, WPARAM, LPARAM);
	static HDC hDeviceContext; 
	static HGLRC hRenderContext;
	static void (* updateGraphics)(int);	
	static Color_t black;
	static Color_t white;
	static Color_t red;
	static Color_t green;
	static Color_t blue;
	static Color_t grey;
	static Color_t salmon;
	static Color_t yellow;
	static Color_t pale;
	static Color_t lightblue;

	static bool keyPressed;
	static char key;
	static double lastTime;
	static double lastCycle;

private: // Rendering privative shapes for display list and other private functions 
	int initShapes();
	int renderCircle(int listIndex,GLfloat lineWidth);
	int renderRect(int listIndex,GLfloat width, GLfloat height, GLfloat lineWidth);
public:
	static bool isLocked;

	static int circlelist;
	static int rectlist;
	static int ringlist;
	static int boxlist;

public:
	static Vector2D scale;			///< What is the size of a pixel in workspace coordinates?
	static Vector2D center;			///< What workspace coordinate is in the middle of the screen? 
	static int windowWidth;			///< What is the width of the window in pixel? 
	static int windowHeight;		///< What is the height of the window in pixel? 
};

#endif