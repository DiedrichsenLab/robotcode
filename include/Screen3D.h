/// --s-----------------------------------------------------------------------------
/// Screen Object 
/// Drawing one the screen using Glut 
/// j.diedrichsen 
////////////////////////////////////////////////
#ifndef SCREEN_DEF
#define SCREEN_DEF
#include <windows.h>
#include <stdlib.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include "Vector2D.h" 
#include "Vector3D.h" 

#include "Matrix2D.h" 


using namespace std;

typedef GLubyte Color_t[3];
#define SCREENNUMCOLORS 9    ///< Number Colors in color list 

enum Scr_color { 
	SCR_Black,
	SCR_WHITE,
	SCR_RED,
	SCR_GREEN,
	SCR_BLUE,
	SCR_GRAY,
	SCR_SALMON,
	SCR_YELLOW,
	SCR_PALE,
	SCR_LIGHTBLUE,
	SCR_DARKGRAY,
	SCR_DARKGREEN,
	SCR_DARKGRAY2,
}; 

///////////////////////////////////////////////////////////
/// \brief Window that is displayed to the Participant 
/// 
/// Screen implements a Window without a frame. It has a scale 
/// parameter, that influences the projection, so that all drawing commands can be performed in the 
/// Workspace of the robot, i.e. in cm. 
/// 
///////////////////////////////////////////////////////////
class Screen3D {
public:
	Screen3D();
	~Screen3D();
	void init(HINSTANCE hThisInst,int xPos, int yPos, int width,int height,void (*displayFcn)(int));  ///< Initializes the window
	void initLighting();						///< sets the lighting of  the scene 
	BOOL createRenderContext( void );			///< Rendering context is for openGL drawing commands 
	void deleteRenderContext( void );			///< Deleted 

	void clear();
	void lock();
	void unlocked();


	void getSizePixel(int &height, int &width);				///< Return the size of the Screen3D in pixel 
	Vector2D getCenter(void);								///< Return workspace coordinate for center of Screen3D  
	void setCenter(Vector2D size);							///< Set center of Screen3D in workspace coordinates 
	Vector2D getSize(void);									///< Return the size (in workspace coordinates) 
	Vector2D getScale(void);								///< Scaling (size of one pixel in Workspace coordinates)
	void setScale(Vector2D scale);							///< Scaling factor of pixel to coordinate system
	

	// Static callback functions 
	static void reshape(int w,int h);						///< Static callback functions 
	static void display(HWND hwnd);							///< Static callback functions: update Graphics 
	static void onKey(unsigned char k,int x, int y);		///< Static callback function 
	static void setProjection();							///< Sets the Projection matrix, depending on scale and center 

	// Drawing routines 

	void setColor(int theColor);								///< Sets the current drawing color (from list of colors) 
	void setColor(Color_t theColor){glColor3ubv(theColor);}		///< Sets the current drawing color (in RGB) 
	void drawSphere(Vector3D where,double size);				///< Render a solid sphere
	void drawBox(Vector3D size);								///< Render a box
	void drawDisk(GLfloat xsize,GLfloat ysize,GLfloat x,GLfloat y);		///< Draws a Rectangle
	void drawDisk(Vector2D size, Vector2D pos);								///< Draws a Circle
	void drawCircle(GLfloat xsize,GLfloat ysize, GLfloat x,GLfloat y);
	void drawCircle(GLfloat xsize,GLfloat ysize, GLfloat x,GLfloat y,int linewidth);
	void drawCircle(Vector2D size, Vector2D pos);
	void drawSegment(Vector2D size, Vector2D pos,float startang, float endang);		///< Draws a circular segment 
	void drawRect(GLfloat xsize,GLfloat ysize, GLfloat xpos,GLfloat ypos);		///< Draws a Rectangle
	void drawRect(Vector2D size, Vector2D pos);									///< Draws a Rectangle
	void drawBox(GLfloat xsize,GLfloat ysize, GLfloat xpos,GLfloat ypos);		///< Draws a Filled Rectangle
	void drawBox(Vector2D size, Vector2D pos);									///< Draws a Filled Rectangle
	void drawLine(GLfloat xposStart,GLfloat yposStart, GLfloat xposEnd,GLfloat yposEnd,GLfloat width=1); ///< Draws a Filled Rectangle
	void drawLine(Vector2D start, Vector2D end);								///< Draws a Filled Rectangle
	void drawArrow(Vector2D start, Vector2D end, double headLength);			///< Draws an Arrow 
	void drawPlus(Vector2D size,Vector2D pos);									///< Draws an Plus sign
	void print(string s, double x, double y, GLfloat magnification);			///< Prints a string on the Screen3D 
	inline void print(string s, Vector2D pos, GLfloat magnification)			///< Prints a string on the Screen3D 
					{print(s,pos[0],pos[1],magnification);}
	void printChar(char s, double x, double y, GLfloat magnification);			///< Prints a var on the Screen3D 
	void close();																///< Closes the window 
	


public: // colors and other static members 
	static HWND windowHnd;														///< Window Handle 
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
	static Color_t darkgray;
	static Color_t darkgreen;
	static Color_t darkgray2;

	static bool keyPressed;
	static char key;
	static double lastTime;
	static double lastCycle;

private: // Rendering privative shapes for display list and other private functions 
	int initShapes();																///< Initializes display lists for more complex, often used shapes 
	int renderCircle(int listIndex,GLfloat lineWidth);								///< Renderes a circle (for display Lists) 
	int renderRect(int listIndex,GLfloat width, GLfloat height, GLfloat lineWidth);	///< Renderes a Rectangle 
public:
	static bool isLocked;

	static int circlelist;
	static int rectlist;
	static int disklist;
	static int boxlist;

public:
	static Vector2D scale;			///< What is the size of a pixel in workspace coordinates?
	static Vector2D center;			///< What workspace coordinate is in the middle of the Screen3D? 
	static int windowWidth;			///< What is the width of the window in pixel? 
	static int windowHeight;		///< What is the height of the window in pixel? 
	static double windowWidthcm;		///< What is the width of the window in cm? 
	static double windowHeightcm;		///< What is the height of the window in cm? 

};

#endif