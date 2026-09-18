#pragma once
#include"allnecehead.h"
#define LOOPMAX     1000000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

//int LD(Matrix * Q, Matrix * L, Matrix * D);
//
///* integer gauss transformation ----------------------------------------------*/
//void gauss(Matrix * L, Matrix * Z, int i, int j);
///* permutations --------------------------------------------------------------*/
//void perm(Matrix * L, Matrix * D, int j, double del, Matrix * Z);
///* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
//void reduction(Matrix * L, Matrix * D, Matrix * Z);
///* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
//int search(int m, Matrix * L, Matrix * D, Matrix * zs, Matrix * zn, Matrix * s);
//
void MatrixMultiply(int m1, int n1, int m2, int n2,const double M1[], const double M2[], double M3[]);
int MatrixInv(int n, double a[], double b[]);
int lambdafuh(int n, int m, const double* a, const double* Q, double* F_, double* s);
int lambda(int m,Matrix * amb, Matrix * Q_amb, Matrix * F_, Matrix * s);


