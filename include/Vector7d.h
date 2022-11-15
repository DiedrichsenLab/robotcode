/*****************************************************************************

  vector7D.h: A 3-d vector class based on a double number 
  
	Module Name:
	
	  Vector7D.h
	  
		Description: 
		
		  Provides 3D vector mathematics routines.
		  
*******************************************************************************/

#ifndef vector7D_H_
#define vector7D_H_
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
class Vector7D 
{
public:
//	friend inline std::ostream & ostream::operator<<(std::ostream &, const Vector7D &);
//	friend inline std::istream & ostream::operator>>(std::istream &, Vector7D &);
public:
	inline Vector7D(double = 0,double =0,double=0,double = 0,double =0,double=0,double=0);				///< Create empty 
	inline Vector7D(const Vector7D &);					///< Create by memberwise copy
	inline Vector7D operator + (const Vector7D &) const;		///< Add two vectors
	inline Vector7D operator - (const Vector7D &) const;	///< Subtract two vectors 
	inline Vector7D operator * (const double s) const;		///< Multiply vector by scalar
	inline Vector7D operator / (const double s) const;	///< Divide vector by scalar
	inline double operator * (const Vector7D &) const;		///< dot product
	inline double &operator[] (int i);					///< Index
	inline Vector7D &operator = (const Vector7D &);		///< Assignment by memberwise copy 
    inline Vector7D &operator -= (const Vector7D &v1);	///< minus equal
    inline Vector7D &operator += (const Vector7D &v1);	///< plus equal
    inline Vector7D &operator *= (const Vector7D &v1);	///< element by element multiplication 
    inline Vector7D &operator *= (double s);
    inline Vector7D &operator /= (const Vector7D &v1);	///< element by element division 
    inline Vector7D &operator /= (double s);
	inline double norm();								///< Length of the vector 
	friend inline double norm(Vector7D);				///< Length of the vector  
	inline operator double *() {return(x);}
public: 
	double x[7];
};


// ---------------------------------------------------------	
// Constructor 
inline Vector7D::Vector7D(double a1,double a2,double a3,double a4,double a5,double a6,double a7) { 
	x[0]=a1;
	x[1]=a2;
	x[2]=a3;
	x[3]=a4;
	x[4]=a5;
	x[5]=a6;
	x[6]=a7;
}; 

// ---------------------------------------------------------	
// Constructor 
inline Vector7D::Vector7D(const Vector7D &rhs) { 
	x[0]=rhs.x[0];
	x[1]=rhs.x[1];
	x[2]=rhs.x[2];
	x[3]=rhs.x[3];
	x[4]=rhs.x[4];
	x[5]=rhs.x[5];
	x[6]=rhs.x[6];
}; 

// ---------------------------------------------------------	
// + operator  
inline Vector7D Vector7D::operator + (const Vector7D &rhs) const{ 
	return(Vector7D(x[0]+rhs.x[0],x[1]+rhs.x[1],x[2]+rhs.x[2],
					x[3]+rhs.x[3],x[4]+rhs.x[4],x[5]+rhs.x[5],x[6]+rhs.x[6]));
} 

// ---------------------------------------------------------	
// - operator
inline Vector7D Vector7D::operator - (const Vector7D &rhs) const{ 
	return(Vector7D(x[0]-rhs.x[0],x[1]-rhs.x[1],x[2]-rhs.x[2],
					x[3]-rhs.x[3],x[4]-rhs.x[4],x[5]-rhs.x[5],x[6]-rhs.x[6]));

} 

// ---------------------------------------------------------	
// * operator  for scalar
inline Vector7D Vector7D::operator * (const double rhs) const{ 
	return(Vector7D(x[0]*rhs,x[1]*rhs,x[2]*rhs,
					x[3]*rhs,x[4]*rhs,x[5]*rhs,x[6]*rhs));
} 

// ---------------------------------------------------------	
// / operator for scalar
inline Vector7D Vector7D::operator / (const double rhs) const{ 
	return(Vector7D(x[0]/rhs,x[1]/rhs,x[2]/rhs,
					x[3]/rhs,x[4]/rhs,x[5]/rhs,x[6]/rhs));
} 

// ---------------------------------------------------------	
// = operator
inline Vector7D & Vector7D::operator = (const Vector7D &rhs){ 
	x[0]=rhs.x[0];x[1]=rhs.x[1];x[2]=rhs.x[2];
	x[3]=rhs.x[3];x[4]=rhs.x[4];x[5]=rhs.x[5];x[6]=rhs.x[6];
	return *this;
} 


// ---------------------------------------------------------	
// dot product 
inline double Vector7D::operator * (const Vector7D &rhs) const{
	return x[0]*rhs.x[0]+x[1]*rhs.x[1]+x[2]*rhs.x[2]
			+x[3]*rhs.x[3]+x[4]*rhs.x[4]+x[5]*rhs.x[5]+x[6]*rhs.x[6];
}

// ---------------------------------------------------------	
// [] index operator
inline double &Vector7D::operator[] (int i) {
		return x[i];
}

// ---------------------------------------------------------	
// norm 
inline double Vector7D::norm() {
		return sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+
					x[3]*x[3]+x[4]*x[4]+x[5]*x[5]+x[6]*x[6]);
}

inline double norm(Vector7D x) { 
		return sqrt(x.x[0]*x.x[0]+x.x[1]*x.x[1]+x.x[2]*x.x[2]+
					x.x[3]*x.x[3]+x.x[4]*x.x[4]+x.x[5]*x.x[5]+x.x[6]*x.x[6]);
} 

// ---------------------------------------------------------	
// -= operator  
inline Vector7D & Vector7D::operator -= (const Vector7D &v1) { 
	x[0]-=v1.x[0];
	x[1]-=v1.x[1];
	x[2]-=v1.x[2];
	x[3]-=v1.x[3];
	x[4]-=v1.x[4];
	x[5]-=v1.x[5];
	x[6]-=v1.x[6];
	return *this;
} 

// ---------------------------------------------------------	
// -= operator  
inline Vector7D &Vector7D::operator += (const Vector7D &v1){
	x[0]+=v1.x[0];
	x[1]+=v1.x[1];
	x[2]+=v1.x[2];
	x[3]+=v1.x[3];
	x[4]+=v1.x[4];
	x[5]+=v1.x[5];
	x[6]+=v1.x[6];
	return *this;
}

// ---------------------------------------------------------	
// *= operator  
inline Vector7D &Vector7D::operator *= (const Vector7D &v1){
	x[0]*=v1.x[0];
	x[1]*=v1.x[1];
	x[2]*=v1.x[2];
	x[3]*=v1.x[3];
	x[4]*=v1.x[4];
	x[5]*=v1.x[5];
	x[6]*=v1.x[6];
	return *this;
}

inline Vector7D &Vector7D::operator *= (double s){
	x[0]*=s;
	x[1]*=s;
	x[2]*=s;
	x[3]*=s;
	x[4]*=s;
	x[5]*=s;
	x[6]*=s;
	return *this;
}

// ---------------------------------------------------------	
// /= operator  
inline Vector7D &Vector7D::operator /= (const Vector7D &v1){
	x[0]/=v1.x[0];
	x[1]/=v1.x[1];
	x[2]/=v1.x[2];
	x[3]/=v1.x[3];
	x[4]/=v1.x[4];
	x[5]/=v1.x[5];
	x[6]/=v1.x[6];
	return *this;
}

// ---------------------------------------------------------	
inline Vector7D &Vector7D::operator /= (double s){
	x[0]/=s;
	x[1]/=s;
	x[2]/=s;
	x[3]/=s;
	x[4]/=s;
	x[5]/=s;
	x[6]/=s;
	return *this;
}

/*// ---------------------------------------------------------	
// Stream output operator 
ostream &operator <<(std::ostream &out,const Vector7D  v) { 
	return out << v.x[0] << "\t" << v.x[1];
	
} 


// ---------------------------------------------------------	
// Stream input operator 
istream &operator >>(std::istream &in,Vector7D v) { 
	return in >> v.x[0] >> v.x[1];
} */

#endif