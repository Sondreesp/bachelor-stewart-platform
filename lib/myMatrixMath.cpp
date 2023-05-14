
#include "myMatrixMath.h"
#include <iostream>
#include <math.h>



/* Returns the legth of a vector */

double myMatrixMath::vecNorm(double vec[3]){

	return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

}

double myMatrixMath::vecNorm6x6(double vec[6]){
  return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2] + vec[3]*vec[3] + vec[4]*vec[4] + vec[5]*vec[5]);
}

/* Set a matrix to zero */
int myMatrixMath::zeroMatrix(double *Matrix, int N,int M){

  for (int i= 0; i<N; i++ ){
    for(int j = 0; j<M; j++ ){
        Matrix[i*3+j] = 0;
    }
  }

return 1;
}

int myMatrixMath::zeroVector(double vec[], int N){
	for(int i = 0;i<N;i++){
		*(vec + i)=0.0;
	}
return 1;
}




int myMatrixMath::zeroMatrix(double Matrix[3][3]){
	Matrix[0][0] = 0;
	Matrix[1][0] = 0;
	Matrix[2][0] = 0;

	Matrix[0][1] = 0;
	Matrix[1][1] = 0;
	Matrix[2][1] = 0;

	Matrix[0][2] = 0;
	Matrix[1][2] = 0;
	Matrix[2][2] = 0;
return 1;
}

int myMatrixMath::eye3by3(double Matrix[3][3]){
	
	zeroMatrix(Matrix);
	Matrix[0][0] = 1;
	Matrix[1][1] = 1;
	Matrix[2][2] = 1;

return 1;
}	


/* Matrix multiplier, for 3 by 3 matrices  */
void myMatrixMath::MatMatMult3by3(double ResultMat[3][3], double const Mat1[3][3], double const Mat2[3][3])
{

ResultMat[0][0] = Mat1[0][0]*Mat2[0][0] + Mat1[0][1]*Mat2[1][0] + Mat1[0][2]*Mat2[2][0];

ResultMat[1][0] = Mat1[1][0]*Mat2[0][0] + Mat1[1][1]*Mat2[1][0] + Mat1[1][2]*Mat2[2][0];
ResultMat[2][0] = Mat1[2][0]*Mat2[0][0] + Mat1[2][1]*Mat2[1][0] + Mat1[2][2]*Mat2[2][0];

ResultMat[0][1] = Mat1[0][0]*Mat2[0][1] + Mat1[0][1]*Mat2[1][1] + Mat1[0][2]*Mat2[2][1];
ResultMat[1][1] = Mat1[1][0]*Mat2[0][1] + Mat1[1][1]*Mat2[1][1] + Mat1[1][2]*Mat2[2][1];
ResultMat[2][1] = Mat1[2][0]*Mat2[0][1] + Mat1[2][1]*Mat2[1][1] + Mat1[2][2]*Mat2[2][1];

ResultMat[0][2] = Mat1[0][0]*Mat2[0][2] + Mat1[0][1]*Mat2[1][2] + Mat1[0][2]*Mat2[2][2];
ResultMat[1][2] = Mat1[1][0]*Mat2[0][2] + Mat1[1][1]*Mat2[1][2] + Mat1[1][2]*Mat2[2][2];
ResultMat[2][2] = Mat1[2][0]*Mat2[0][2] + Mat1[2][1]*Mat2[1][2] + Mat1[2][2]*Mat2[2][2];

}

/* Matrix Vector Multiplier, for 3 by 3 matrices  */
void myMatrixMath::MatVecMult3by3(double ResultVec[3] ,double const Mat1[3][3], double const Vec[3])
{
ResultVec[0] = Mat1[0][0]*Vec[0] + Mat1[0][1]*Vec[1] + Mat1[0][2]*Vec[2];
ResultVec[1] = Mat1[1][0]*Vec[0] + Mat1[1][1]*Vec[1] + Mat1[1][2]*Vec[2];
ResultVec[2] = Mat1[2][0]*Vec[0] + Mat1[2][1]*Vec[1] + Mat1[2][2]*Vec[2];
}

void myMatrixMath::MatVecMult6by6(double ResultVec[6] ,double const Mat1[6][6], double const Vec[6]){

ResultVec[0] = Mat1[0][0]*Vec[0] + Mat1[0][1]*Vec[1] + Mat1[0][2]*Vec[2] + Mat1[0][3]*Vec[3] + Mat1[0][4]*Vec[4] + Mat1[0][5]*Vec[5];
ResultVec[1] = Mat1[1][0]*Vec[0] + Mat1[1][1]*Vec[1] + Mat1[1][2]*Vec[2] + Mat1[1][3]*Vec[3] + Mat1[1][4]*Vec[4] + Mat1[1][5]*Vec[5];
ResultVec[2] = Mat1[2][0]*Vec[0] + Mat1[2][1]*Vec[1] + Mat1[2][2]*Vec[2] + Mat1[2][3]*Vec[3] + Mat1[2][4]*Vec[4] + Mat1[2][5]*Vec[5];
ResultVec[3] = Mat1[3][0]*Vec[0] + Mat1[3][1]*Vec[1] + Mat1[3][2]*Vec[2] + Mat1[3][3]*Vec[3] + Mat1[3][4]*Vec[4] + Mat1[3][5]*Vec[5];
ResultVec[4] = Mat1[4][0]*Vec[0] + Mat1[4][1]*Vec[1] + Mat1[4][2]*Vec[2] + Mat1[4][3]*Vec[3] + Mat1[4][4]*Vec[4] + Mat1[4][5]*Vec[5];
ResultVec[5] = Mat1[5][0]*Vec[0] + Mat1[5][1]*Vec[1] + Mat1[5][2]*Vec[2] + Mat1[5][3]*Vec[3] + Mat1[5][4]*Vec[4] + Mat1[5][5]*Vec[5];

}

/* Matrix Vector Multiplier, for 3 by 3 matrices  */
void myMatrixMath::MatTransVecMult3by3(double ResultVec[3] ,double const Mat1[3][3], double const Vec[3])
{
ResultVec[0] = Mat1[0][0]*Vec[0] + Mat1[1][0]*Vec[1] + Mat1[2][0]*Vec[2];
ResultVec[1] = Mat1[0][1]*Vec[0] + Mat1[1][1]*Vec[1] + Mat1[2][1]*Vec[2];
ResultVec[2] = Mat1[0][2]*Vec[0] + Mat1[1][2]*Vec[1] + Mat1[2][2]*Vec[2];
}


/* Matrix Multiplied By Constant */
void myMatrixMath::MatConstMult3by3(double ResultMat[3][3] ,double const Mat1[3][3], double const Constant)
{
ResultMat[0][0] = Mat1[0][0]*Constant;
ResultMat[1][0] = Mat1[1][0]*Constant;
ResultMat[2][0] = Mat1[2][0]*Constant;

ResultMat[0][1] = Mat1[0][1]*Constant;
ResultMat[1][1] = Mat1[1][1]*Constant;
ResultMat[2][1] = Mat1[2][1]*Constant;

ResultMat[0][2] = Mat1[0][2]*Constant;
ResultMat[1][2] = Mat1[1][2]*Constant;
ResultMat[2][2] = Mat1[2][2]*Constant;
}


void myMatrixMath::MatMatAdd3by3(double ResultMat[3][3], double Mat1[3][3], double Mat2[3][3])
{
ResultMat[0][0] = Mat1[0][0] + Mat2[0][0];
ResultMat[1][0] = Mat1[1][0] + Mat2[1][0];
ResultMat[2][0] = Mat1[2][0] + Mat2[2][0];

ResultMat[0][1] = Mat1[0][1] + Mat2[0][1];
ResultMat[1][1] = Mat1[1][1] + Mat2[1][1];
ResultMat[2][1] = Mat1[2][1] + Mat2[2][1];

ResultMat[0][2] = Mat1[0][2] + Mat2[0][2];
ResultMat[1][2] = Mat1[1][2] + Mat2[1][2];
ResultMat[2][2] = Mat1[2][2] + Mat2[2][2];
}

void myMatrixMath::copyMatrix3by3(double MatOut[3][3], double MatInn[3][3]){

MatOut[0][0] = MatInn[0][0];
MatOut[0][1] = MatInn[0][1];
MatOut[0][2] = MatInn[0][2];

MatOut[1][0] = MatInn[1][0];
MatOut[1][1] = MatInn[1][1];
MatOut[1][2] = MatInn[1][2];

MatOut[2][0] = MatInn[2][0];
MatOut[2][1] = MatInn[2][1];
MatOut[2][2] = MatInn[2][2];

}




void myMatrixMath::VecVecAdd3(double ResVec[3], double Vec1[3], double Vec2[3]){
  
  ResVec[0] = Vec1[0]+Vec2[0];
  ResVec[1] = Vec1[1]+Vec2[1];
  ResVec[2] = Vec1[2]+Vec2[2];

}


void myMatrixMath::VecVecAdd6(double ResVec[6], double Vec1[6], double Vec2[6]){
  
  ResVec[0] = Vec1[0]+Vec2[0];
  ResVec[1] = Vec1[1]+Vec2[1];
  ResVec[2] = Vec1[2]+Vec2[2];

  ResVec[3] = Vec1[3]+Vec2[3];
  ResVec[4] = Vec1[4]+Vec2[4];
  ResVec[5] = Vec1[5]+Vec2[5];

}


void myMatrixMath::VecVecSub3(double ResVec[3], double Vec1[3], double Vec2[3]){
  
  ResVec[0] = Vec1[0]-Vec2[0];
  ResVec[1] = Vec1[1]-Vec2[1];
  ResVec[2] = Vec1[2]-Vec2[2];

}


void myMatrixMath::VecVecSub6(double ResVec[6], double Vec1[6], double Vec2[6]){
  
  ResVec[0] = Vec1[0]-Vec2[0];
  ResVec[1] = Vec1[1]-Vec2[1];
  ResVec[2] = Vec1[2]-Vec2[2];

  ResVec[3] = Vec1[3]-Vec2[3];
  ResVec[4] = Vec1[4]-Vec2[4];
  ResVec[5] = Vec1[5]-Vec2[5];

}

int myMatrixMath::VecVecDiv(double *ResVec, double *Vec1, double *Vec2, int Size){
	for(int i=0;i<Size;i++){
		if(!(*(Vec2+i)==0)){
			*(ResVec+i) = *(Vec1+i)/ *(Vec2+i);
		}
		else
		{
			printf("divide by zero in VecVecDiv \n");
			return 0;
		}
	}
	return 1;
}


void myMatrixMath::PrintVec(double *Vec, int Size){
  for(int i = 0; i<Size-1; i++){
    printf("%lf, ",Vec[i]);
  }
  printf("%lf;\n ",Vec[Size-1]);
}

void myMatrixMath::Print3by3(double Matrix[3][3]){

for(int i=0;i<3;i++){
  for(int j=0;j<3;j++){
    printf("%lf, ",Matrix[i][j]);
  }printf(";\n");
}
printf("\n\n");

}


int myMatrixMath::getRotation(double Matrix[3][3], int axis, double angle){

  double cs = cos(angle);
  double si = sin(angle);

  switch(axis)
  {
    case 1:
	   Matrix[0][0] = 1; Matrix[0][1] =  0; Matrix[0][2] =   0; 
	   Matrix[1][0] = 0; Matrix[1][1] = cs; Matrix[1][2] = -si;
	   Matrix[2][0] = 0; Matrix[2][1] = si; Matrix[2][2] =  cs;
	   return 1;
    case 2:
	   Matrix[0][0] = cs;  Matrix[0][1] = 0; Matrix[0][2] = si;
	   Matrix[1][0] =  0;  Matrix[1][1] = 1; Matrix[1][2] =  0;
	   Matrix[2][0] = -si; Matrix[2][1] = 0; Matrix[2][2] = cs;
	   return 1;
    case 3:
	   Matrix[0][0] = cs; Matrix[0][1] = -si; Matrix[0][2] = 0;
	   Matrix[1][0] = si; Matrix[1][1] =  cs; Matrix[1][2] = 0;
	   Matrix[2][0] =  0; Matrix[2][1] =   0; Matrix[2][2] = 1;
	   return 1;
    default:
	   printf("Problem with getRotation, axis was: %d \n",axis);
	   return 0;
  }
return 1;
}





/*Get the rotational matrix of an unknown rotational axis */
int myMatrixMath::expMRodriguez(double Rot[3][3], double w1, double w2, double w3, double dt) {

  
    double f1, f2;    
    double normw0 = sqrt(w1 * w1 + w2 * w2 + w3 * w3);
    double O[3][3] = {0};

    double I[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    double m2[3][3] = {0};
    double m3[3][3] = {0};
    double m4[3][3] = {0};
    double m5[3][3] = {0};

    O[0][0] = 0;
    O[0][1] = -w3;
    O[0][2] = w2;
    O[1][0] = w3;
    O[1][1] = 0;
    O[1][2] = -w1;
    O[2][0] = -w2;
    O[2][1] = w1;
    O[2][2] = 0;

    if (normw0 >= 0.00000001) {
         f1 = (sin(normw0 * dt)) / (normw0);
         f2 = (1 - cos(normw0 * dt)) / (normw0 * normw0);
    };
    if (normw0 <= 0.00000001) {
         f1 = 0;
         f2 = 0;
    };

    myMatrixMath::MatConstMult3by3(m2 ,O, f1);
    
     myMatrixMath::MatMatMult3by3(m3 ,O, O);
    
     myMatrixMath::MatConstMult3by3(m4 ,m3, f2);

     myMatrixMath::MatMatAdd3by3(m5, m2, m4);

     myMatrixMath::MatMatAdd3by3(Rot, I, m5);


    return 1;
}
