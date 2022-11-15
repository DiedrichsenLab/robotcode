/*****************************************************************************

  vector2D.h: A 2-d vector class based on a double number 
  
	Module Name:
	
	  vector2D.h
	  
		Description: 
		
		  Provides 2D vector mathematics routines.
		  
*******************************************************************************/

#ifndef vector2D_H_
#define vector2D_H_
#include <math.h>
#include <iostream>
using namespace std; 

////////////////////////////////////////////////////////////////////////////
/// \brief class that implements mathematical operations for a 2x1 vector
///  
/// Most necessary mathematical operations are defined over operator overloading.
/// For some reason the stream operators seem not to work, as there seem to be conflicting 
/// definitions somewhere else 
/// 
////////////////////////////////////////////////////////////////////////////
class Vector2D 
{
public:
//	friend inline std::ostream & ostream::operator<<(std::ostream &, const Vector2D &);
//	friend inline std::istream & ostream::operator>>(std::istream &, Vector2D &);
public:
	inline Vector2D(double = 0,double =0);				///< Create empty 
	inline Vector2D(const Vector2D &);					///< Create by memberwise copy
	inline Vector2D operator + (const Vector2D &) const;		///< Add two vectors
	inline Vector2D operator - (const Vector2D &) const;	///< Subtract two vectors 
	inline Vector2D operator * (const double s) const;		///< Multiply vector by scalar
	inline Vector2D operator / (const double s) const;	///< Divide vector by scalar
	inline double operator * (const Vector2D &) const;		///< dot product
	inline double &operator[] (int i);					///< Index
	inline Vector2D &operator = (const Vector2D &);		///< Assignment by memberwise copy 
    inline Vector2D &operator -= (const Vector2D &v1);	///< minus equal
    inline Vector2D &operator += (const Vector2D &v1);	///< plus equal
    inline Vector2D &operator *= (const Vector2D &v1);	///< element by element multiplication 
    inline Vector2D &operator *= (double s);
    inline Vector2D &operator /= (const Vector2D &v1);	///< element by element division 
    inline Vector2D &operator /= (double s);
	inline double norm();								///< Length of the vector 
	friend inline double norm(Vector2D);				///< Length of the vector  
	inline operator double *() {return(x);}				

public: 
	double x[2];
};


// ---------------------------------------------------------	
// Constructor 
inline Vector2D::Vector2D(double a1,double a2) { 
	x[0]=a1;
	x[1]=a2;
}; 

// ---------------------------------------------------------	
// Constructor 
inline Vector2D::Vector2D(const Vector2D &rhs) { 
	x[0]=rhs.x[0];
	x[1]=rhs.x[1];
}; 

// ---------------------------------------------------------	
// + operator  
inline Vector2D Vector2D::operator + (const Vector2D &rhs) const{ 
	return(Vector2D(x[0]+rhs.x[0],x[1]+rhs.x[1]));
} 

// ---------------------------------------------------------	
// - operator
inline Vector2D Vector2D::operator - (const Vector2D &rhs) const{ 
	return(Vector2D(x[0]-rhs.x[0],x[1]-rhs.x[1]));
} 

// ---------------------------------------------------------	
// * operator  for scalar
inline Vector2D Vector2D::operator * (const double rhs) const{ 
	return(Vector2D(x[0]*rhs,x[1]*rhs));
} 

// ---------------------------------------------------------	
// / operator for scalar
inline Vector2D Vector2D::operator / (const double rhs) const{ 
	return(Vector2D(x[0]/rhs,x[1]/rhs));
} 

// ---------------------------------------------------------	
// = operator
inline Vector2D & Vector2D::operator = (const Vector2D &rhs){ 
	x[0]=rhs.x[0];x[1]=rhs.x[1];
	return *this;
} 


// ---------------------------------------------------------	
// dot product 
inline double Vector2D::operator * (const Vector2D &rhs) const{
	return x[0]*rhs.x[0]+x[1]*rhs.x[1];
}

// ---------------------------------------------------------	
// [] index operator
inline double &Vector2D::operator[] (int i) {
		return x[i];
}

// ---------------------------------------------------------	
// norm 
inline double Vector2D::norm() {
		return sqrt(x[0]*x[0]+x[1]*x[1]);
}

inline double norm(Vector2D x) { 
		return sqrt(x.x[0]*x.x[0]+x.x[1]*x.x[1]);
} 

// ---------------------------------------------------------	
// -= operator  
inline Vector2D & Vector2D::operator -= (const Vector2D &v1) { 
	x[0]-=v1.x[0];
	x[1]-=v1.x[1];
	return *this;
} 

// ---------------------------------------------------------	
// -= operator  
inline Vector2D &Vector2D::operator += (const Vector2D &v1){
	x[0]+=v1.x[0];
	x[1]+=v1.x[1];
	return *this;
}

// ---------------------------------------------------------	
// *= operator  
inline Vector2D &Vector2D::operator *= (const Vector2D &v1){
	x[0]*=v1.x[0];
	x[1]*=v1.x[1];
	return *this;
}

inline Vector2D &Vector2D::operator *= (double s){
	x[0]*=s;
	x[1]*=s;
	return *this;
}

// ---------------------------------------------------------	
// /= operator  
inline Vector2D &Vector2D::operator /= (const Vector2D &v1){
	x[0]/=v1.x[0];
	x[1]/=v1.x[1];
	return *this;
}

// ---------------------------------------------------------	
inline Vector2D &Vector2D::operator /= (double s){
	x[0]/=s;
	x[1]/=s;
	return *this;
}

/*// ---------------------------------------------------------	
// Stream output operator 
ostream &operator <<(std::ostream &out,const Vector2D  v) { 
	return out << v.x[0] << "\t" << v.x[1];
	
} 


// ---------------------------------------------------------	
// Stream input operator 
istream &operator >>(std::istream &in,Vector2D v) { 
	return in >> v.x[0] >> v.x[1];
} */

#endif