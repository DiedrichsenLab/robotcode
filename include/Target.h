//////////////////////////////////////////////////////////////
// Target.h
// Simple class that implements rectangles that can change color 
// and explode when they are hit. 
//   2007, Joern Diedrichsen 
//   j.diedrichsen@breversalangor.ac.uk
// extension: various shapes of targets (AR June 2011)
//////////////////////////////////////////////////////////////
#ifndef TARGET_DEF
#define TARGET_DEF


#include <math.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include "Screen.h" 
#include "Vector2D.h"

extern Screen gScreen;

// different shapes - more can be added, then draw() has to be extended
enum TargetShape
{
	SHAPE_RECT,				///< 0 rectangle = default
	SHAPE_BOX,				///< 1 box = filled rectangle
	SHAPE_CIRC,				///< 2 circle
	SHAPE_DISC,				///< 3 disc = filled circle
	SHAPE_PLUS,				///< 4 plus sign
	SHAPE_HEX,				///< 5 hexagon
	SHAPE_HEXF				///< 6 filled hexagon
	//SHAPE_ELLIPSE           //// 7 ellipse
};


//////////////////////////////////////////////////////////////
/// \brief Implements some routines that are useful for having a reachin target 
/// 
/// This can change the target in color and stores a position. The  
/// main feature is that it can explode visually 
//////////////////////////////////////////////////////////////
class Target{
public: 
//	Target();				///< constructor 
	Target(TargetShape s);	///< constructor with shape given
	Target(TargetShape s, int c);
	void draw();			///< draw on screen 
	void setColor(int c);	///< sets color from table 	
	int color;				///< Color (numtable entry)
	TargetShape shape;		///< shape of the target
	void setShape(TargetShape c);	///< sets shape
	Vector2D position;		///< position of target 
	Vector2D size;			///< size of target (mm)	
	void explode(double speed); ///< start explosion 
public: 
	double explodeState;	///< where in explosion are you? 
	double explodeSpeed;	///< explosion speed 
	bool isExplode;			///< is currently exploding? 
public: 
	static double colors[5][3];	
}; 

//////////////////////////////////////////////////////////////
/// Constructor doesn't do much, but sets the Target to 
/// be invisible 
//////////////////////////////////////////////////////////////
//Target::Target(){ 
//	isExplode=false; 
//	color=0; // invisible 
//	shape=SHAPE_RECT; // default shape is rectangle
//} 

//////////////////////////////////////////////////////////////
/// Constructor doesn't do much, but sets the Target to 
/// be invisible 
/// and sets the shape of it
//////////////////////////////////////////////////////////////
Target::Target(TargetShape s){ 
	isExplode=false; 
	color=0; // invisible 
	shape=s; // set the desired shape
}

Target::Target(TargetShape s, int c) {
	isExplode = false;
	color = c; // invisible 
	shape = s; // set the desired shape
}

//////////////////////////////////////////////////////////////
/// Draws the target and draws a number of lines if 
/// the target is exploding 
//////////////////////////////////////////////////////////////
void Target::draw(void){ 
	int i,j; 
	if (isExplode) {
		if (explodeState>6) { 
			isExplode=false; 
			setColor(0);
		} else {  
			gScreen.setColor(color); 
			j=(int)floor(explodeState); 
			for (i=0;i<explodeState/2;i++) { 
				switch(shape) {
				case SHAPE_RECT:
					gScreen.drawRect(size*(explodeState-i*2),position);
					break;
				case SHAPE_BOX: // when exploding, rectangles just look nicer
					gScreen.drawRect(size*(explodeState-i*2),position);
					break;			
				case SHAPE_CIRC:
					gScreen.drawCircle(size*(explodeState-i*2),position);
					break;
				case SHAPE_DISC: // when exploding, circles just look nicer
					gScreen.drawCircle(size*(explodeState-i*2),position);
					break;
				case SHAPE_PLUS:
					gScreen.drawPlus(size*(explodeState-i*2),position);
					break;
				case SHAPE_HEX:
					gScreen.drawHex(size*(explodeState-i*2),position);
					break;
				case SHAPE_HEXF: // when exploding, non filled just looks nicer
					gScreen.drawHex(size*(explodeState-i*2),position);
					break;




				}
			} 
			explodeState+=explodeSpeed;
		} 
	} else { 
		if (color>=0) {  		
			gScreen.setColor(color); 
			switch(shape) {
			case SHAPE_RECT:
				gScreen.drawRect(size,position);
				break;
			case SHAPE_BOX:
				gScreen.drawBox(size,position);
				break;			
			case SHAPE_CIRC:
				gScreen.drawCircle(size,position);
				break;
			case SHAPE_DISC:
				gScreen.drawDisk(size,position);
				break;			
			case SHAPE_PLUS:
				gScreen.drawPlus(size,position);
				break;
			case SHAPE_HEX:
				gScreen.drawHex(size,position);
				break;
			case SHAPE_HEXF:
				gScreen.drawHexF(size,position);
				break;
			}
		} 
	}
} 

//////////////////////////////////////////////////////////////
/// Starts an explosion
/// \param speed cm/ms of the explosion  
//////////////////////////////////////////////////////////////
void Target::explode(double speed){ 
	explodeState=1; 
	explodeSpeed=speed; 
	isExplode=true;
} 

//////////////////////////////////////////////////////////////
/// set color from a color list
/// \param c is the whished color 0: black 1: white 
/// \sa Screen 
//////////////////////////////////////////////////////////////
void Target::setColor(int c){ 
	color=c;
} 

//////////////////////////////////////////////////////////////
/// set shape
/// \param s is the whished shape 
/// \sa Screen 
//////////////////////////////////////////////////////////////
void Target::setShape(TargetShape s){ 
	shape=s;
} 
#endif
