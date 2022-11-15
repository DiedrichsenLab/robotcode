////////////////////////////////////////////////////////////////////////////
//
//  Matrix3D.h: A 3-d Matrix class based on a double number 
//  
//	Module Name:
//	
//	  Matrix3D.h
//	  
//		Description: 
//		
//		  Provides 3D matrix mathematics routines.
//		
//   2010, Nada Yousif
//   n.yousif@ucl.ac.uk  
////////////////////////////////////////////////////////////////////////////

#ifndef Matrix3D_H_
#define Matrix3D_H_
#include <iostream>
#include "Vector3D.h"
using namespace std;

////////////////////////////////////////////////////////////////////////////
/// \brief class that implements mathematical operations for a 2x2 matrix 
///  
/// Most necessary mathematical operations are defined over operator overloading.
/// Furthermore, there is a stream input and output operator.
///
////////////////////////////////////////////////////////////////////////////
class Matrix3D 
{
	friend ostream &operator<<(ostream &, const Matrix3D &);		///< Tab-delimited output 
	friend istream &operator>>(istream &, Matrix3D &);				///< Input from File 
public:
	inline Matrix3D(double = 0,double =0,double=0,double= 0,double = 0,double =0,double=0,double= 0,double= 0);		///< Create empty matrix - 9 elements
	inline Matrix3D operator + (const Matrix3D &) const;			///< Addition 
	inline Matrix3D operator - (const Matrix3D &) const;			///< Subtraction
	inline Matrix3D operator * (const double s) const;				///< Multiplication with scalar 
	inline Matrix3D operator * (const Matrix3D &) const;			///< Matrix Multiplication 
	inline Matrix3D operator / (const double s) const;				///< Devision by scalar 
	inline Matrix3D &operator = (const Matrix3D &);					///< assignment operator (memberwise copy) 
	inline double *operator [](const int i);						///< retrive a row or element 
	inline Vector3D operator* (Vector3D &src) const;				///< Matrix * vector multiplication
	inline Matrix3D getInverse() const;								///< Inverse 
	inline Matrix3D getTranspose();									///< Transpose 
public:															
	double x[3][3];													///< The four data members 
}; 


// ---------------------------------------------------------	
inline Matrix3D::Matrix3D(double a11,double a12,double a13,double a21,double a22,double a23,double a31,double a32,double a33) { 
	x[0][0]=a11;
	x[0][1]=a12;
	x[0][2]=a13;
	x[1][0]=a21;
	x[1][1]=a22;
	x[1][2]=a23;
	x[2][0]=a31;
	x[2][1]=a32;
	x[2][2]=a33;
}

// ---------------------------------------------------------	
inline Matrix3D Matrix3D::operator + (const Matrix3D &rhs) const{ 
	return(Matrix3D(x[0][0]+rhs.x[0][0],x[0][1]+rhs.x[0][1],x[0][2]+rhs.x[0][2],x[1][0]+rhs.x[1][0],x[1][1]+rhs.x[1][1],x[1][2]+rhs.x[1][2],x[2][0]+rhs.x[2][0],x[2][1]+rhs.x[2][1],x[2][2]+rhs.x[2][2]));
} 

// ---------------------------------------------------------	
inline Matrix3D Matrix3D::operator - (const Matrix3D &rhs) const{ 
	return(Matrix3D(x[0][0]-rhs.x[0][0],x[0][1]-rhs.x[0][1],x[0][2]-rhs.x[0][2],x[1][0]-rhs.x[1][0],x[1][1]-rhs.x[1][1],x[1][2]-rhs.x[1][2],x[2][0]-rhs.x[2][0],x[2][1]-rhs.x[2][1],x[2][2]-rhs.x[2][2]));
} 

// ---------------------------------------------------------	
inline Matrix3D Matrix3D::operator * (const double s) const{ 
	return(Matrix3D(x[0][0]*s,x[0][1]*s,x[0][2]*s,x[1][0]*s,x[1][1]*s,x[1][2]*s,x[2][0]*s,x[2][1]*s,x[2][2]*s));
} 

// ---------------------------------------------------------	
inline Matrix3D Matrix3D::operator * (const Matrix3D &rhs) const{ 
	return(Matrix3D(
		x[0][0]*rhs.x[0][0]+ x[0][1]*rhs.x[1][0]+x[0][2]*rhs.x[2][0],
		x[0][0]*rhs.x[0][1]+ x[0][1]*rhs.x[1][1]+x[0][2]*rhs.x[2][1],
		x[0][0]*rhs.x[0][2]+ x[0][1]*rhs.x[1][2]+x[0][2]*rhs.x[2][2],
		x[1][0]*rhs.x[0][0]+ x[1][1]*rhs.x[1][0]+x[1][2]*rhs.x[2][0],
		x[1][0]*rhs.x[0][1]+ x[1][1]*rhs.x[1][1]+x[1][2]*rhs.x[2][1],
		x[1][0]*rhs.x[0][2]+ x[1][1]*rhs.x[1][2]+x[1][2]*rhs.x[2][2],
		x[2][0]*rhs.x[0][0]+ x[2][1]*rhs.x[1][0]+x[2][2]*rhs.x[2][0],
		x[2][0]*rhs.x[0][1]+ x[2][1]*rhs.x[1][1]+x[2][2]*rhs.x[2][1],
		x[2][0]*rhs.x[0][2]+ x[2][1]*rhs.x[1][2]+x[2][2]*rhs.x[2][2]));
} 

// ---------------------------------------------------------	
inline Matrix3D Matrix3D::operator / (const double s) const{ 
	return(Matrix3D(x[0][0]/s,x[0][1]/s,x[0][2]/s,x[1][0]/s,x[1][1]/s,x[1][2]/s,x[2][0]/s,x[2][1]/s,x[2][2]/s));
} 

// ---------------------------------------------------------	
inline Matrix3D & Matrix3D::operator = (const Matrix3D &rhs){ 
	x[0][0]=rhs.x[0][0];
	x[0][1]=rhs.x[0][1];
	x[0][2]=rhs.x[0][2];
	x[1][0]=rhs.x[1][0];
	x[1][1]=rhs.x[1][1];
	x[1][2]=rhs.x[1][2];
	x[2][0]=rhs.x[2][0];
	x[2][1]=rhs.x[2][1];
	x[2][2]=rhs.x[2][2];
	return *this;
} 

// ---------------------------------------------------------	
// Index operator allows double indexing [i][j]
inline double * Matrix3D::operator[](const int i){
	return &x[i][0];
}

// ---------------------------------------------------------	
// Index operator allows double indexing [i][j]
inline Vector3D Matrix3D::operator * (Vector3D &src) const{
	return(Vector3D(x[0][0]*src[0]+x[0][1]*src[1]+x[0][2]*src[2],x[1][0]*src[0]+x[1][1]*src[1]+x[1][2]*src[2],x[2][0]*src[0]+x[2][1]*src[1]+x[2][2]*src[2]));
}

// ---------------------------------------------------------	
// Inversion of matrix, returns zero matrix if not sucessful
inline Matrix3D Matrix3D::getInverse() const{ 
	double det=x[0][0]*(x[1][1]*x[2][2]-x[2][1]*x[1][2])-x[0][1]*(x[1][0]*x[2][2]-x[2][0]*x[1][2])+x[0][2]*(x[1][0]*x[2][1]-x[2][0]*x[1][1]);
	if (det!=0) { 
		return(Matrix3D((x[1][1]*x[2][2]-x[2][1]*x[1][2]),-(x[2][2]*x[0][1]-x[2][1]*x[0][2]),(x[1][2]*x[0][1]-x[1][1]*x[0][2]),-(x[2][2]*x[1][0]-x[2][0]*x[1][2]),(x[2][2]*x[0][0]-x[2][0]*x[0][2]),-(x[1][2]*x[0][0]-x[1][0]*x[0][2]),(x[2][1]*x[1][0]-x[2][0]*x[1][1]),-(x[2][1]*x[0][0]-x[2][0]*x[0][1]),(x[1][1]*x[0][0]-x[1][0]*x[0][1]))/det);
	} else { 
		return(Matrix3D(0,0,0,0,0,0,0,0,0));
	} 
} 

// ---------------------------------------------------------	
// Inversion of matrix, returns zero matrix if not sucessful
inline Matrix3D Matrix3D::getTranspose(){ 
	return(Matrix3D(x[0][0],x[1][0],x[2][0],x[0][1],x[1][1],x[2][1],x[0][2],x[1][2],x[2][2]));
} 

// stream operators for Matrix 
// ---------------------------------------------------------	
// Stream output operator 
inline std::ostream &operator <<(std::ostream &out,const Matrix3D  &M) { 
	out << M.x[0][0] << "\t" << M.x[0][1] << "\t" << M.x[0][2] << "\t" << M.x[1][0] << "\t" << M.x[1][1] << "\t" << M.x[1][2] << "\t" << M.x[2][0] << "\t" << M.x[2][1] << "\t" << M.x[2][2];
	return out;
} 


// ---------------------------------------------------------	
// Stream input operator 
inline std::istream &operator >>(std::istream &in,Matrix3D &M) { 
	in >> M.x[0][0] >> M.x[0][1] >> M.x[0][2] >>  M.x[1][0] >> M.x[1][1] >> M.x[1][2] >>  M.x[2][0] >> M.x[2][1] >> M.x[2][2];
	return in;
} 


inline Matrix3D rotationMatrixRad(double ang,char axis){
	if (axis=='x'){
		return Matrix3D(1,0,0,0,cos(ang),-sin(ang),0,sin(ang),cos(ang));	///< Gives back a rotation
	} 
	else if (axis=='y'){
		return Matrix3D(cos(ang),0,sin(ang),0,1,0,-sin(ang),0,cos(ang));	///< Gives back a rotation
	}
	else if (axis=='z'){
		return Matrix3D(cos(ang),-sin(ang),0,sin(ang),cos(ang),0,0,0,1);	///< Gives back a rotation
	}
} 

#endif