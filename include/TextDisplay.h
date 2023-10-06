///////////////////////////////////////////////////////////////////////
// class TextDisplay 
// provides the Experimenter text display with display & command line options 
// 
// To initialize the Text display use init
//		width: Width of the windows in pixel 
//		rows: How many text rows total
//		scrollrows: of these rows the lower x are scrolling 
//		columns: number of columns above the scroll rows
//		onRet: simple callback function to deal with user input 
// 
//   2007, Joern Diedrichsen 
//   j.diedrichsen@bangor.ac.uk
///////////////////////////////////////////////////////////////////////
#ifndef DEF_TEXTDISP
#define DEF_TEXTDISP
#include <stdlib.h> 
#include <GL/glut.h>
#if defined(WIN32) || defined(linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif
#include <windows.h>
#include <stdio.h>						// Header File For Standard Input/Output	( ADD )
#include "Vector2D.h"
#include <string>
using namespace std;
typedef GLubyte Color_t[3];

void glPrint(string text, int xpos, int ypos);

///////////////////////////////////////////////////////////////////////
/// \brief Provides the Window which which the Experimenter can commicate with the program  
///
/// The upper part of the text display is dedicated to the output 
/// the lower part to the input from the experimenter
///	\sa MyExperiment:parseCommands()
///////////////////////////////////////////////////////////////////////
class TextDisplay { 
public:
	TextDisplay();
	/// initialize the textdisplay 
	void init(HINSTANCE hInst, int xPos, int yPos, int width,int rows,int scrollrows, int columns,void (*onRet)(string)); 
	void setText(string text,int row,int column);		///< sets the text to a certain row / column
	void printVector3(string text,double *vec,int row,int column);  ///< prints out a 3D vector 		
	void printMatrix4(double *M,int row,int column);				///< prints a 4x4 Matrix 
	void printDouble(string text,double d,int row,int column);		///< prints a single double value 
	void scroll();													///< Scrolls the command line one up
	void print(string x);											///< give out unformated text at command line and scrolls up
	void lock();													///< No more keystrokes are accepted 
	void unlock();													///< allow input again
	static void display(HWND hwnd);									///< Display 
	static void onKey(unsigned char k,int x, int y);				///< on key callback 
	static int cursorCol;											///< Where is the cursor 
	static void resetOnRet(void(*onRet)(string));					///< callback on return 
	void close();
	
	
	static HWND windowHnd;
	static WNDCLASSEX wcl;
	static LRESULT CALLBACK messageCallback(HWND, UINT, WPARAM, LPARAM);
	static bool keyPressed;
	static char key;
private: 
	static int numRows;
	static int numScrollRows;
	static int numColumns;
	static int columnWidth;
	static int rowHeight;
	static string* text;
	static void reshape(int w,int h);
	static bool isLocked; 
	static Vector2D highlightPos;
	static Color_t highlightColor;
	static void drawHighlight();
	static int history;
	static int historySize;
	char buffer[80];
};

#endif 
