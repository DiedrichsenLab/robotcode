////////////////////////////////////////////////////////////////////////////
//
//  Matrix2D.h: A 2-d Matrix class based on a double number 
//  
//	Module Name:
//	
//	  Matrix2D.h
//	  
//		Description: 
//		
//		  Provides 2D matrix mathematics routines.
//		
//   2007, Joern Diedrichsen 
//   j.diedrichsen@bangor.ac.uk  
////////////////////////////////////////////////////////////////////////////

#ifndef Matrix2D_H_
#define Matrix2D_H_
#include <iostream>
#include "Vector2D.h"
using namespace std;

////////////////////////////////////////////////////////////////////////////
/// \brief class that implements mathematical operations for a 2x2 matrix 
///  
/// Most necessary mathematical operations are defined over operator overloading.
/// Furthermore, there is a stream input and output operator.
///
////////////////////////////////////////////////////////////////////////////
class Matrix2D 
{
	friend ostream &operator<<(ostream &, const Matrix2D &);		///< Tab-delimited output 
	friend istream &operator>>(istream &, Matrix2D &);				///< Input from File 
public:
	inline Matrix2D(double = 0,double =0,double=0,double= 0);		///< Create empty matrix 
	inline Matrix2D operator + (const Matrix2D &) const;			///< Addition 
	inline Matrix2D operator - (const Matrix2D &) const;			///< Subtraction
	inline Matrix2D operator * (const double s) const;				///< Multiplication with scalar 
	inline Matrix2D operator * (const Matrix2D &) const;			///< Matrix Multiplication 
	inline Matrix2D operator / (const double s) const;				///< Devision by scalar 
	inline Matrix2D &operator = (const Matrix2D &);					///< assignment operator (memberwise copy) 
	inline double *operator [](const int i);						///< retrive a row or element 
	inline Vector2D operator* (Vector2D &src) const;				///< Matrix * vector multiplication
	inline Matrix2D getInverse() const;								///< Inverse 
	inline Matrix2D getTranspose();									///< Transpose 
public:															
	double x[2][2];													///< The four data members 
}; 


// ---------------------------------------------------------	
inline Matrix2D::Matrix2D(double a11,double a12,double a21,double a22) { 
	x[0][0]=a11;
	x[0][1]=a12;
	x[1][0]=a21;
	x[1][1]=a22;
}

// ---------------------------------------------------------	
inline Matrix2D Matrix2D::operator + (const Matrix2D &rhs) const{ 
	return(Matrix2D(x[0][0]+rhs.x[0][0],x[0][1]+rhs.x[0][1],x[1][0]+rhs.x[1][0],x[1][1]+rhs.x[1][1]));
} 

// ---------------------------------------------------------	
inline Matrix2D Matrix2D::operator - (const Matrix2D &rhs) const{ 
	return(Matrix2D(x[0][0]-rhs.x[0][0],x[0][1]-rhs.x[0][1],x[1][0]-rhs.x[1][0],x[1][1]-rhs.x[1][1]));
} 

// ---------------------------------------------------------	
inline Matrix2D Matrix2D::operator * (const double s) const{ 
	return(Matrix2D(x[0][0]*s,x[0][1]*s,x[1][0]*s,x[1][1]*s));
} 

// ---------------------------------------------------------	
inline Matrix2D Matrix2D::operator * (const Matrix2D &rhs) const{ 
	return(Matrix2D(
		x[0][0]*rhs.x[0][0]+ x[0][1]*rhs.x[1][0],
		x[0][0]*rhs.x[0][1]+ x[0][1]*rhs.x[1][1],
		x[1][0]*rhs.x[0][0]+ x[1][1]*rhs.x[1][0],
		x[1][0]*rhs.x[0][1]+ x[1][1]*rhs.x[1][1]));
} 



// ---------------------------------------------------------	
inline Matrix2D Matrix2D::operator / (const double s) const{ 
	return(Matrix2D(x[0][0]/s,x[0][1]/s,x[1][0]/s,x[1][1]/s));
} 

// ---------------------------------------------------------	
inline Matrix2D & Matrix2D::operator = (const Matrix2D &rhs){ 
	x[0][0]=rhs.x[0][0];
	x[0][1]=rhs.x[0][1];
	x[1][0]=rhs.x[1][0];
	x[1][1]=rhs.x[1][1];	
	return *this;
} 

// ---------------------------------------------------------	
// Index operator allows double indexing [i][j]
inline double * Matrix2D::operator[](const int i){
	return &x[i][0];
}

// ---------------------------------------------------------	
// Index operator allows double indexing [i][j]
inline Vector2D Matrix2D::operator * (Vector2D &src) const{
	return(Vector2D(x[0][0]*src[0]+x[0][1]*src[1],x[1][0]*src[0]+x[1][1]*src[1]));
}

// ---------------------------------------------------------	
// Inversion of matrix, returns zero matrix if not sucessful
inline Matrix2D Matrix2D::getInverse() const{ 
	double det=x[0][0]*x[1][1]-x[1][0]*x[0][1];
	if (det!=0) { 
		return(Matrix2D(x[1][1],-x[0][1],-x[1][0],x[0][0])/det);
	} else { 
		return(Matrix2D(0,0,0,0));
	} 
} 

// ---------------------------------------------------------	
// Inversion of matrix, returns zero matrix if not sucessful
inline Matrix2D Matrix2D::getTranspose(){ 
	return(Matrix2D(x[0][0],x[1][0],x[0][1],x[1][1]));
} 

// stream operators for Matrix 
// ---------------------------------------------------------	
// Stream output operator 
inline std::ostream &operator <<(std::ostream &out,const Matrix2D  &M) { 
	out << M.x[0][0] << "\t" << M.x[0][1] << "\t" << M.x[1][0] << "\t" << M.x[1][1];
	return out;
} 


// ---------------------------------------------------------	
// Stream input operator 
inline std::istream &operator >>(std::istream &in,Matrix2D &M) { 
	in >> M.x[0][0] >> M.x[0][1] >>  M.x[1][0] >> M.x[1][1];
	return in;
} 


inline Matrix2D rotationMatrixRad(double ang){
	return Matrix2D(cos(ang),sin(ang),-sin(ang),cos(ang));	///< Gives back a rotation
} 

inline Matrix2D outerprod(const Vector2D &lhs,const Vector2D &rhs ){ 
	return(Matrix2D(lhs.x[0]*rhs.x[0],lhs.x[0]*rhs.x[1],lhs.x[1]*rhs.x[0],lhs.x[1]*rhs.x[1]));
} 

#endif