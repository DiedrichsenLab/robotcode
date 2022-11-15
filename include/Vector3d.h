/*****************************************************************************

  vector3D.h: A 3-d vector class based on a double number 
  
	Module Name:
	
	  vector3D.h
	  
		Description: 
		
		  Provides 3D vector mathematics routines.
		  
*******************************************************************************/

#ifndef vector3D_H_
#define vector3D_H_
#include <math.h>
#include <iostream>
using namespace std; 

////////////////////////////////////////////////////////////////////////////
/// \brief class that implements mathematical operations for a 3x1 vector
///  
/// Most necessary mathematical operations are defined over operator overloading.
/// For some reason the stream operators seem not to work, as there seem to be conflicting 
/// definitions somewhere else 
/// 
////////////////////////////////////////////////////////////////////////////
class Vector3D 
{
public:
//	friend inline std::ostream & ostream::operator<<(std::ostream &, const Vector3D &);
//	friend inline std::istream & ostream::operator>>(std::istream &, Vector3D &);
public:
	inline Vector3D(double = 0,double =0,double=0);				///< Create empty 
	inline Vector3D(const Vector3D &);					///< Create by memberwise copy
	inline Vector3D operator + (const Vector3D &) const;		///< Add two vectors
	inline Vector3D operator - (const Vector3D &) const;	///< Subtract two vectors 
	inline Vector3D operator * (const double s) const;		///< Multiply vector by scalar
	inline Vector3D operator / (const double s) const;	///< Divide vector by scalar
	inline double operator * (const Vector3D &) const;		///< dot product
	inline double &operator[] (int i);					///< Index
	inline Vector3D &operator = (const Vector3D &);		///< Assignment by memberwise copy 
    inline Vector3D &operator -= (const Vector3D &v1);	///< minus equal
    inline Vector3D &operator += (const Vector3D &v1);	///< plus equal
    inline Vector3D &operator *= (const Vector3D &v1);	///< element by element multiplication 
    inline Vector3D &operator *= (double s);
    inline Vector3D &operator /= (const Vector3D &v1);	///< element by element division 
    inline Vector3D &operator /= (double s);
	inline double norm();								///< Length of the vector 
	friend inline double norm(Vector3D);				///< Length of the vector  
	inline operator double *() {return(x);}
public: 
	double x[3];
};


// ---------------------------------------------------------	
// Constructor 
inline Vector3D::Vector3D(double a1,double a2,double a3) { 
	x[0]=a1;
	x[1]=a2;
	x[2]=a3;
}; 

// ---------------------------------------------------------	
// Constructor 
inline Vector3D::Vector3D(const Vector3D &rhs) { 
	x[0]=rhs.x[0];
	x[1]=rhs.x[1];
	x[2]=rhs.x[2];
}; 

// ---------------------------------------------------------	
// + operator  
inline Vector3D Vector3D::operator + (const Vector3D &rhs) const{ 
	return(Vector3D(x[0]+rhs.x[0],x[1]+rhs.x[1],x[2]+rhs.x[2]));
} 

// ---------------------------------------------------------	
// - operator
inline Vector3D Vector3D::operator - (const Vector3D &rhs) const{ 
	return(Vector3D(x[0]-rhs.x[0],x[1]-rhs.x[1],x[2]-rhs.x[2]));
} 

// ---------------------------------------------------------	
// * operator  for scalar
inline Vector3D Vector3D::operator * (const double rhs) const{ 
	return(Vector3D(x[0]*rhs,x[1]*rhs,x[2]*rhs));
} 

// ---------------------------------------------------------	
// / operator for scalar
inline Vector3D Vector3D::operator / (const double rhs) const{ 
	return(Vector3D(x[0]/rhs,x[1]/rhs,x[2]/rhs));
} 

// ---------------------------------------------------------	
// = operator
inline Vector3D & Vector3D::operator = (const Vector3D &rhs){ 
	x[0]=rhs.x[0];x[1]=rhs.x[1];x[2]=rhs.x[2];
	return *this;
} 


// ---------------------------------------------------------	
// dot product 
inline double Vector3D::operator * (const Vector3D &rhs) const{
	return x[0]*rhs.x[0]+x[1]*rhs.x[1]+x[2]*rhs.x[2];
}

// ---------------------------------------------------------	
// [] index operator
inline double &Vector3D::operator[] (int i) {
		return x[i];
}

// ---------------------------------------------------------	
// norm 
inline double Vector3D::norm() {
		return sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
}

inline double norm(Vector3D x) { 
		return sqrt(x.x[0]*x.x[0]+x.x[1]*x.x[1]+x.x[2]*x.x[2]);
} 

// ---------------------------------------------------------	
// -= operator  
inline Vector3D & Vector3D::operator -= (const Vector3D &v1) { 
	x[0]-=v1.x[0];
	x[1]-=v1.x[1];
	x[2]-=v1.x[2];
	return *this;
} 

// ---------------------------------------------------------	
// -= operator  
inline Vector3D &Vector3D::operator += (const Vector3D &v1){
	x[0]+=v1.x[0];
	x[1]+=v1.x[1];
	x[2]+=v1.x[2];
	return *this;
}

// ---------------------------------------------------------	
// *= operator  
inline Vector3D &Vector3D::operator *= (const Vector3D &v1){
	x[0]*=v1.x[0];
	x[1]*=v1.x[1];
	x[2]*=v1.x[2];
	return *this;
}

inline Vector3D &Vector3D::operator *= (double s){
	x[0]*=s;
	x[1]*=s;
	x[2]*=s;
	return *this;
}

// ---------------------------------------------------------	
// /= operator  
inline Vector3D &Vector3D::operator /= (const Vector3D &v1){
	x[0]/=v1.x[0];
	x[1]/=v1.x[1];
	x[2]/=v1.x[2];
	return *this;
}

// ---------------------------------------------------------	
inline Vector3D &Vector3D::operator /= (double s){
	x[0]/=s;
	x[1]/=s;
	x[2]/=s;
	return *this;
}

/*// ---------------------------------------------------------	
// Stream output operator 
ostream &operator <<(std::ostream &out,const Vector3D  v) { 
	return out << v.x[0] << "\t" << v.x[1];
	
} 


// ---------------------------------------------------------	
// Stream input operator 
istream &operator >>(std::istream &in,Vector3D v) { 
	return in >> v.x[0] >> v.x[1];
} */

#endif