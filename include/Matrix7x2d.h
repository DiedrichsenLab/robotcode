////////////////////////////////////////////////////////////////////////////
//
//  Matrix3D.h: A 7x2-d Matrix class based on a double number 
//  
//	Module Name:
//	
//	  Matrix3D.h
//	  
//		Description: 
//		
//		  Provides 3D matrix mathematics routines.
//	
//	
//   2010, Nada Yousif
//   n.yousif@ucl.ac.uk  
//
//	 Modified by N Roach from 3D Mat routines, 2011
//
////////////////////////////////////////////////////////////////////////////

#ifndef Matrix7x2D_H_
#define Matrix7x2D_H_
#include <iostream>
#include "Vector7D.h"
#include "Vector2D.h"
#include "Matrix2D.h"
using namespace std;

////////////////////////////////////////////////////////////////////////////
/// \brief class that implements mathematical operations for a 7x2 matrix 
///  
/// Most necessary mathematical operations are defined over operator overloading.
/// Furthermore, there is a stream input and output operator.
///
////////////////////////////////////////////////////////////////////////////
class Matrix7x2D 
{
public:
	inline Matrix7x2D(double = 0,double =0,double=0,double= 0,double = 0,double =0,double=0,double= 0,double= 0,double = 0,double =0,double=0,double= 0,double= 0);		///< Create empty matrix - 14 elements
	inline Matrix7x2D TraceMult(void);		///< 7x7 Matrix (L) by Transposed 7x2 Matrix mult
	inline Matrix7x2D operator + (const Matrix7x2D &) const;			///< Addition 
	inline Matrix7x2D operator - (const Matrix7x2D &) const;			///< Subtraction
	inline Matrix7x2D operator * (const double s) const;				///< Multiplication with scalar 
	inline Vector7D operator* (Vector2D &src) const;				///<  2D vector * matrix mult
	inline Vector2D operator* (Vector7D &src) const;				///<  7D vector * matrix mult
	inline Matrix7x2D operator / (const double s) const;				///< Devision by scalar 
	inline Matrix7x2D &operator = (const Matrix7x2D &);					///< assignment operator (memberwise copy) 
	inline double *operator [](const int i);						///< retrive a row or element 

public:															
	double x[7][2];													///< The four data members 
	double L[7][7];													///< An additional 7x7 matrix for the additional Learning op (3DoM)
}; 


// ---------------------------------------------------------	
inline Matrix7x2D::Matrix7x2D(double a11,double a12,double a21,double a22,double a31,double a32,double a41,double a42,double a51,double a52,double a61,double a62,double a71,double a72) { 
	x[0][0]=a11;
	x[0][1]=a12;
	x[1][0]=a21;
	x[1][1]=a22;
	x[2][0]=a31;
	x[2][1]=a32;
	x[3][0]=a41;
	x[3][1]=a42;
	x[4][0]=a51;
	x[4][1]=a52;
	x[5][0]=a61;
	x[5][1]=a62;
	x[6][0]=a71;
	x[6][1]=a72;

	int i=0;
	int j=0;
	for (i = 0; i < 7; i++) 
	for (j = 0; j < 7; j++) 
		L[i][j]=0; //zero L on matrix init
}

// ---------------------------------------------------------	
inline Matrix7x2D Matrix7x2D::operator + (const Matrix7x2D &rhs) const{ 
	return(Matrix7x2D(x[0][0]+rhs.x[0][0]
					 ,x[0][1]+rhs.x[0][1]
					 ,x[1][0]+rhs.x[1][0]
					 ,x[1][1]+rhs.x[1][1]
					 ,x[2][0]+rhs.x[2][0]
					 ,x[2][1]+rhs.x[2][1]
					 ,x[3][0]+rhs.x[3][0]
					 ,x[3][1]+rhs.x[3][1]
					 ,x[4][0]+rhs.x[4][0]
					 ,x[4][1]+rhs.x[4][1]
					 ,x[5][0]+rhs.x[5][0]
					 ,x[5][1]+rhs.x[5][1]
					 ,x[6][0]+rhs.x[6][0]
					 ,x[6][1]+rhs.x[6][1]));
} 

// ---------------------------------------------------------	
inline Matrix7x2D Matrix7x2D::operator - (const Matrix7x2D &rhs) const{ 
	return(Matrix7x2D(x[0][0]-rhs.x[0][0]
					 ,x[0][1]-rhs.x[0][1]
					 ,x[1][0]-rhs.x[1][0]
					 ,x[1][1]-rhs.x[1][1]
					 ,x[2][0]-rhs.x[2][0]
					 ,x[2][1]-rhs.x[2][1]
					 ,x[3][0]-rhs.x[3][0]
					 ,x[3][1]-rhs.x[3][1]
					 ,x[4][0]-rhs.x[4][0]
					 ,x[4][1]-rhs.x[4][1]
					 ,x[5][0]-rhs.x[5][0]
					 ,x[5][1]-rhs.x[5][1]
					 ,x[6][0]-rhs.x[6][0]
					 ,x[6][1]-rhs.x[6][1]));
} 

// ---------------------------------------------------------	
inline Matrix7x2D Matrix7x2D::operator * (const double s) const{ 
	return(Matrix7x2D(x[0][0]*s
					 ,x[0][1]*s
					 ,x[1][0]*s
					 ,x[1][1]*s
					 ,x[2][0]*s
					 ,x[2][1]*s
					 ,x[3][0]*s
					 ,x[3][1]*s
					 ,x[4][0]*s
					 ,x[4][1]*s
					 ,x[5][0]*s
					 ,x[5][1]*s
					 ,x[6][0]*s
					 ,x[6][1]*s));
} 

// ---------------------------------------------------------	

inline Matrix7x2D & Matrix7x2D::operator = (const Matrix7x2D &rhs){ 
	x[0][0]=rhs.x[0][0];
	x[0][1]=rhs.x[0][1];
	x[1][0]=rhs.x[1][0];
	x[1][1]=rhs.x[1][1];
	x[2][0]=rhs.x[2][0];
	x[2][1]=rhs.x[2][1];
	x[3][0]=rhs.x[3][0];
	x[3][1]=rhs.x[3][1];
	x[4][0]=rhs.x[4][0];
	x[4][1]=rhs.x[4][1];
	x[5][0]=rhs.x[5][0];
	x[5][1]=rhs.x[5][1];
	x[6][0]=rhs.x[6][0];
	x[6][1]=rhs.x[6][1];

	return *this;
} 

// ---------------------------------------------------------	
// Index operator allows double indexing [i][j]
inline double * Matrix7x2D::operator[](const int i){
	return &x[i][0];
}

// ---------------------------------------------------------	
// 7x7 Matrix (L) by Transposed 7x2 Matrix mult
inline Matrix7x2D Matrix7x2D::TraceMult(void){

	Matrix7x2D outmat(0,0,0,0,0,0,0,0,0,0,0,0,0,0);
	Matrix7x2D outmat_sw(0,0,0,0,0,0,0,0,0,0,0,0,0,0);

		  int i = 0;
	      int j = 0;
		  int k = 0;

		  for(i = 0; i < 7; i++) 
		  {
             for( j = 0; j < 2; j++)
			 {
               for( k = 0; k < 7; k++) 
				{
                  outmat.x[i][j] +=  L[i][k] * x[k][j];
				  
				}
			 }

		  }
	
		  outmat_sw[0][1]=outmat[0][0];
		  outmat_sw[1][1]=outmat[1][0];
		  outmat_sw[2][1]=outmat[2][0];
		  outmat_sw[3][1]=outmat[3][0];
		  outmat_sw[4][1]=outmat[4][0];
		  outmat_sw[5][1]=outmat[5][0];
		  outmat_sw[6][1]=outmat[6][0];
		  outmat_sw[0][0]=outmat[0][1];
		  outmat_sw[1][0]=outmat[1][1];
		  outmat_sw[2][0]=outmat[2][1];
		  outmat_sw[3][0]=outmat[3][1];
		  outmat_sw[4][0]=outmat[4][1];
		  outmat_sw[5][0]=outmat[5][1];
		  outmat_sw[6][0]=outmat[6][1];
		  
		  
	return(outmat_sw);
}

// ---------------------------------------------------------	
// 7D vector matrix mult
inline Vector7D Matrix7x2D::operator * (Vector2D &src) const{

	//return(Vector7D(x[0][0]*src[0]+x[0][1]*src[1],x[1][0]*src[0]+x[1][1]*src[1]));

		 Vector7D outvect(0,0,0,0,0,0,0);

		  int i = 0;
	      int j = 0;
	
		  for(i = 0; i < 7; i++) 
		  {
             for( j = 0; j < 2; j++)
			 {        
                  outvect.x[i] +=  x[i][j] * src[j];
			 }

		  }
		  
		return(outvect);

}

// ---------------------------------------------------------	
// 2D vector matrix mult
inline Vector2D Matrix7x2D::operator * (Vector7D &src) const{

	
		Vector2D outvect(0,0);

		  int i = 0;
	      int j = 0;
	
		 
             for( j = 0; j < 2; j++)
			 {
				  for(i = 0; i < 7; i++) 
				  {
        
					outvect.x[j] +=  x[i][j] * src[i];
				  }

			 }


	return(outvect);
}



#endif