#ifndef myMatrixMath_h
#define myMatrixMath_h

#include <math.h>
#include <stdio.h>

namespace myMatrixMath{

/* Calculate the norm of a R³ vector */
double vecNorm(double vec[3]);	

double vecNorm6x6(double vec[6]);
	
/* Set a matrix to zero */
int zeroMatrix(double *Matrix, int N,int M);

int zeroVector(double vec[], int N);

int zeroMatrix(double Matrix[3][3]);


int eye3by3(double Matrix[3][3]);


/* Matrix multiplier, for 3 by 3 matrices  */
void MatMatMult3by3(double ResultMat[3][3], double const Mat1[3][3], double const Mat2[3][3]);

/* Matrix Vector Multiplier, for 3 by 3 matrices  */
void MatVecMult3by3(double ResultVec[3] ,double const Mat1[3][3], double const Vec[3]);

/* Matrix Vector Multiplier, for 6 by 6 matrices  */
void MatVecMult6by6(double ResultVec[6] ,double const Mat1[6][6], double const Vec[6]);

/* Matrix Vector Multiplier, for 3 by 3 matrices  */
void MatTransVecMult3by3(double ResultVec[3] ,double const Mat1[3][3], double const Vec[3]);


/* Matrix Multiplied By Constant */
void MatConstMult3by3(double ResultMat[3][3] ,double const Mat1[3][3], double const Constant);


void MatMatAdd3by3(double ResultMat[3][3], double Mat1[3][3], double Mat2[3][3]);


void copyMatrix3by3(double MatOut[3][3], double MatInn[3][3]);


void VecVecAdd3(double ResVec[3], double Vec1[3], double Vec2[3]);

void VecVecAdd6(double ResVec[6], double Vec1[6], double Vec2[6]);

void VecVecSub3(double ResVec[3], double Vec1[3], double Vec2[3]);

void VecVecSub6(double ResVec[6], double Vec1[6], double Vec2[6]);

int VecVecDiv(double *ResVec, double *Vec1, double *Vec2, int Size);

void PrintVec(double Vec[3], int Size);

void Print3by3(double Matrix[3][3]);


int getRotation(double Matrix[3][3], int axis, double angle);



/*Get the rotational matrix of an unknown rotational axis */
int expMRodriguez(double Rot[3][3], double w1, double w2, double w3, double dt);

/*Returns the inverse of a 6 by 6 array, listed in seperate file due to size*/
int myInvMat6x6(double invM[6][6], double M[6][6]);

}
#endif
