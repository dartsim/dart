/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef UTILS_UTILSMATH_H
#define UTILS_UTILSMATH_H

// Standard Libraries
#include <vector>
#include <cmath>
#include <ctime>
#include <climits>
#include <cassert>
#include <iostream>
using namespace std;
// External Libraries
#include <Eigen/Dense>
using namespace Eigen;
// Local Headers
#include "Misc.h"

namespace utils {

    // a cross b = (CR*a) dot b
    /* const Matd CR(2,2,0.0,-1.0,1.0,0.0); */
    const Matrix2d CR( (Matrix2d() << 0.0, -1.0, 1.0, 0.0).finished() );

    inline int delta(int i, int j) {
        if(i == j) return 1;
        return 0;
    }

    inline int sgn(double a){
        if(a<0) return -1;
        else if(a==0) return 0;
        else return 1;
    }

    inline double sqr(double x){
        return x*x;
    }

    inline double Tsinc(double theta){
        return 0.5-sqrt(theta)/48;
    }

    inline bool isZero(double theta){
        return(fabs(theta)<EPSILON);
    }

    inline double asinh(double X){
        return log(X + sqrt(X * X + 1));
    }

    inline double acosh(double X){
        return log(X + sqrt(X * X - 1));
    }

    inline double atanh(double X){
        return log((1 + X)/(1 - X))/ 2;
    }

    inline double asech(double X){
        return log((sqrt(-X * X + 1) + 1) / X);
    }

    inline double acosech(double X){
        return log((sgn(X) * sqrt(X * X + 1) +1) / X);
    }

    inline double acotanh(double X){
        return log((X + 1) / (X - 1)) / 2;
    }

    inline double round(double x){
        return floor(x+0.5);
    }

    inline double round2(double x){
        int gintx = (int)floor(x);
        if(x-gintx<0.5) return (double)gintx;
        else return (double)(gintx+1);
    }

    inline bool isEqual(double x, double y){
        return (fabs(x - y) < EPSILON);
    }
	
    // check if it is an integer
    inline bool isInt(double x){
        if(isEqual(round(x), x)) return true;
        else return false;
    }

    inline double random( double min, double max ){
        return min + ((double)rand()/(RAND_MAX + 1.0)) * (max-min);
    }

    inline unsigned seedRand(){
        time_t now = time(0);
        unsigned char *p = (unsigned char *)&now;
        unsigned seed = 0;
        size_t i;

        for(i=0; i<sizeof(now); i++)
            seed = seed * (UCHAR_MAX + 2U ) + p[i];

        srand( seed );
        return seed;
    }

    //inline VectorXd xformHom(const MatrixXd& M, const VectorXd& _v) { ///< homogeneous transformation of the vector _v with the last value treated a 1
    //    int n = _v.size();
    //    assert(M.rows() == n + 1);
    //    VectorXd augV(n + 1);
    //    augV.head(n) = _v;
    //    augV(n) = 1.0;

    //    augV = M * augV;
    //    VectorXd ret = augV.head(n);
    //    return ret;
    //}

    inline Vector3d xformHom(const Matrix4d& _W, const Vector3d& _x) { ///< homogeneous transformation of the vector _v with the last value treated a 1
        //Vector4d x(_x(0), _x(1), _x(2), 1.0);
        //Vector4d y = _W * x;
        //return Vector3d(y(0), y(1), y(2));
        return _W.topLeftCorner(3,3)*_x + _W.col(3).head(3);
    }

    inline Vector3d xformHomDir(const Matrix4d& _W, const Vector3d& _v) { ///< homogeneous transformation of the vector _v treated as a direction: last value 0
        return _W.topLeftCorner(3,3)*_v;
    }

    inline Matrix3d makeSkewSymmetric(const Vector3d& v){
        Matrix3d result = Matrix3d::Zero();
		
        result(0, 1) = -v(2);
        result(1, 0) =  v(2);
        result(0, 2) =  v(1);
        result(2, 0) = -v(1);
        result(1, 2) = -v(0);
        result(2, 1) =  v(0);

        return result;
    }

    inline Vector3d fromSkewSymmetric(const Matrix3d& m) {
        if (fabs(m(0, 0)) > EPSILON || fabs(m(1, 1)) > EPSILON || fabs(m(2, 2)) > EPSILON) {
            cout << "Not skew symmetric matrix" << endl;
            cerr << m <<endl;
            return Vector3d::Zero();
        }
        Vector3d ret = Vector3d::Zero();
        ret(0) = m(2,1);
        ret(1) = m(0,2);
        ret(2) = m(1,0);

        return ret;
    }

} // namespace utils

#endif // #ifndef UTILS_UTILSMATH_H


