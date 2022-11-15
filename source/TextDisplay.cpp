#include "TextDisplay.h"
#include <stdlib.h>
#if defined(WIN32) || defined(linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif
#include <stdio.h>						// Header File For Standard Input/Output	( ADD )


using namespace std;
#include <windows.h>



/*******************************************************************************
 Declare static data members 
*******************************************************************************/
int TextDisplay::numRows;
int TextDisplay::numScrollRows;
int TextDisplay::numColumns;
int TextDisplay::columnWidth;
int TextDisplay::rowHeight;
HWND TextDisplay::windowHnd;
WNDCLASSEX TextDisplay::wcl;

bool TextDisplay::isLocked;
int TextDisplay::cursorCol;
Vector2D TextDisplay::highlightPos;
Color_t TextDisplay::highlightColor;
int TextDisplay::history;
int TextDisplay::historySize;
bool TextDisplay::keyPressed=0;
char TextDisplay::key;

string* TextDisplay::text;


void (*onReturn)(string) = NULL; 

int letterWidth=10;


/*******************************************************************************
 Constructor 
*******************************************************************************/
TextDisplay::TextDisplay() { 
 // 	cout << "Constructor"<<endl;
} 


char szWinNamw[] = "MyWin";


/*******************************************************************************
 init
*******************************************************************************/
void TextDisplay::init(HINSTANCE hThisInst, int xPos,int yPos, int width,int rows,int scrollrows,int columns, void (*onRet)(string)){ 
	int height = rows*22+10;			// Calculate the height based on a Helvetic 18 font 
	numRows=rows;			
	numScrollRows=scrollrows;
	numColumns=columns;
	columnWidth=width/numColumns;
	rowHeight=height/numRows;
	text=new string[numRows*numColumns];
	onReturn=onRet;							// Assign callback pointer 
	text[numRows-1]=">_";
	cursorCol=0;
	isLocked=false; 
	highlightPos=Vector2D(-1,-1);
	highlightColor[0]=0;
	highlightColor[1]=0;
	highlightColor[2]=0;
	historySize=0;

	wcl.cbSize = sizeof(WNDCLASSEX);
	wcl.hInstance = hThisInst;
	wcl.lpszClassName = "My Window";
	wcl.lpfnWndProc = messageCallback;
	wcl.style = 0;
	wcl.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wcl.hIconSm = LoadIcon(NULL, IDI_WINLOGO);
	wcl.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcl.lpszMenuName = NULL;
	wcl.cbClsExtra = 0;
	wcl.cbWndExtra = 0;
	wcl.hbrBackground = (HBRUSH) GetStockObject(WHITE_BRUSH);
	if(!RegisterClassEx(&wcl)) return;
	windowHnd = CreateWindow(
		"My Window",
		"TextDisplay",
		WS_OVERLAPPEDWINDOW,
		xPos,
		yPos,
		width,
		height,
		HWND_DESKTOP,
		NULL,
		hThisInst,
		NULL
	);
	ShowWindow(windowHnd, SW_SHOW);
	UpdateWindow(windowHnd);
}

LRESULT CALLBACK TextDisplay::messageCallback(HWND hwnd, UINT message,
							WPARAM wParam, LPARAM lParam){
	switch(message)	{
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_PAINT:
		display(hwnd);
		break;
	case WM_SIZE:
		reshape(LOWORD(lParam),HIWORD(lParam));
		break;
	case WM_CHAR: 
		onKey((unsigned char)wParam,0,0);
		InvalidateRect(hwnd,NULL,TRUE);
		UpdateWindow(windowHnd);
		break;
	default:
		return DefWindowProc(hwnd, message, wParam, lParam);
	}
	return 0;
}



/*******************************************************************************
 reshape
*******************************************************************************/
void TextDisplay::reshape(int w,int h) { 
	glViewport(0,0,(GLsizei) w,(GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0,(GLdouble) w,0.0,(GLdouble) h);
	columnWidth=w/numColumns;
	rowHeight=h/numRows;
} 

/*******************************************************************************
 Text Display Callback 
*******************************************************************************/
void TextDisplay::display(HWND hwnd) { 
	PAINTSTRUCT ps; 
    HDC hdc; 
	hdc = BeginPaint(hwnd, &ps); 

	for (int r=0;r<numRows;r++) { 
		for (int c=0;c<numColumns;c++) { 
			TextOut(hdc,c*columnWidth,rowHeight*r+6,text[r+c*numRows].c_str(),text[r+c*numRows].length());			
		} 
	} 
    EndPaint(hwnd, &ps); 	
} 

/******************************************************************************
	Set Text 
********************************************************************************/
void TextDisplay::setText(string t,int row,int col){ 
	if (col<numColumns &&  row < numRows) 
		text[row+col*numRows]=t;
} 

/******************************************************************************
	printVector3
********************************************************************************/

void TextDisplay::printDouble(string title,double d,int row,int col){ 
	if (col<numColumns &&  row < numRows) { 
			sprintf(buffer,"%s %2.2f ",title.c_str(),d);
			text[row+col*numRows]=buffer;
	} 
} 
void TextDisplay::printVector3(string title,double *vec,int row,int col){ 
	if (col<numColumns &&  row < numRows) { 
			sprintf(buffer,"%s %2.2f %2.2f %2.2f",title.c_str(),vec[0],vec[1],vec[2]);
			text[row+col*numRows]=buffer;
	} 
} 
/******************************************************************************
	printMatrix4 
********************************************************************************/
void TextDisplay::printMatrix4(double *M,int row,int col){ 
	for (int i=0;i<4;i++) { 
		if (col<numColumns &&  row+i < numRows) { 
			sprintf(buffer,"%2.2f %2.2f %2.2f %2.2f",M[i*4],M[i*4+1],M[i*4+2],M[i*4+3]);
			text[row+i+col*numRows]=buffer;
		} 
	} 
} 


/*******************************************************************************
 on Key: present new character in the last line, do any necessary line discipline,
present cursor at appropriate offset.
*******************************************************************************/
void TextDisplay::onKey(unsigned char k,int x, int y){ 
	int i;
	if (isLocked) {
		keyPressed=1;
		key=k;
		return;
	} 
	if (k==13) {
		for (i=numRows-numScrollRows;i<numRows-2;i++){ 
			text[i]=text[i+1];
		}
		text[numRows-2]=text[numRows-1].substr(0,text[numRows-1].length()-1);
		(*onReturn)(text[numRows-1].substr(1,text[numRows-1].length()-2));					// Call registered callback function
		text[numRows-1]=">_";
		cursorCol=0;
		history=0;
		historySize=(historySize++<numScrollRows)? historySize : numScrollRows ;
	} 
	else if (k==27) exit(0);
	else if (k==8)
	  {
	    if(cursorCol>0){
	      text[numRows-1]=text[numRows-1].substr(0,cursorCol)+text[numRows-1].substr(cursorCol+1,text[numRows-1].length()-1);
	      cursorCol-=1;
	    } 
	  }
	  else { 
	    string temp;
	    temp= text[numRows-1].substr(0,cursorCol+1);
	    (temp+=k);//+=text[numRows-1].substr(cursorCol+1,text[numRows-1].length()-1);
	    cursorCol+=1;
	    temp+=text[numRows-1].substr(cursorCol,text[numRows-1].length()-cursorCol);
	    text[numRows-1]=temp;
	}
} 

/*******************************************************************************
 Scroll: make the last line go up
*******************************************************************************/
void TextDisplay::scroll(){
	int i;
	for (i=numRows-numScrollRows;i<numRows-1;i++){ 
		text[i]=text[i+1];
	}
} 

/*******************************************************************************
 TextDisplay print: Gives a line of text in the end of the scrolling window
*******************************************************************************/
void TextDisplay::print(string format){
	text[numRows-1]=format;
	scroll();
	text[numRows-1]=">_";
	cursorCol=0;

	InvalidateRect(windowHnd,NULL,TRUE);
	UpdateWindow(windowHnd);
}

/*******************************************************************************
 Lock Do not allow for any new input 
*******************************************************************************/
void TextDisplay::lock(){
	text[numRows-1]="";
	isLocked=true;
}

/*******************************************************************************
 Unlock Do not allow for any new input 
*******************************************************************************/
void TextDisplay::unlock(){
	text[numRows-1]=">_";
	cursorCol=0;
	isLocked=false;
}


/*************************************************
Reset private data member onret to allow different behavior in different environments.
*************************************************/
void TextDisplay::resetOnRet(void(*onRet)(string))
{
  onReturn=onRet;

}

/*************************************************
Close 
*************************************************/
void TextDisplay::close(void)
{
	if (windowHnd!=NULL) { 
		DestroyWindow(windowHnd);
		windowHnd=NULL;
	}
}
