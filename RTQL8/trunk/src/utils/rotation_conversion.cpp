#include "rotation_conversion.h"

// Standard Libraries
#include <cstdio>
#include <iostream>
using namespace std;

namespace utils {
  namespace rot_conv {
    Quaterniond matrix_to_quat(Matrix3d& mat) {
      return Quaterniond(mat);
    }

    Matrix3d quat_to_matrix(Quaterniond& q) {
      return Matrix3d(q);
    }

    Quaterniond exp_to_quat(Vector3d& v) {
      double mag = v.norm();
      if(mag > 1e-10){
        Quaterniond q(AngleAxisd(mag, v / mag));
        return q;
      }
      else{
        Quaterniond q(1, 0, 0, 0);
        return q;
      }
    }

    Vector3d quat_to_exp(Quaterniond& q) {
      AngleAxisd aa(q);
      Vector3d v = aa.axis();
      return v*aa.angle();
    }

// Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
// coordinate system transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
    Vector3d matrix_to_euler(Matrix3d& m, RotationOrder order) {
      double x, y, z;

      if(order==XYZ) {
        if(m(2, 0) > (1.0-EPSILON)) {
          cout << "North Pole" << endl;
          x = atan2(m(0, 1), m(0, 2));
          y = -M_PI / 2.0;
          z = 0.0;
        }
        if(m(2, 0) < -(1.0-EPSILON)) {
          cout << "South Pole" << endl;
          x = atan2(m(0, 1), m(0, 2));
          y = M_PI / 2.0;
          z = 0.0;
        }
        x = atan2(m(2, 1), m(2, 2));
        y = -asin(m(2, 0));
        z = atan2(m(1, 0), m(0, 0));
        return Vector3d(x,y,z);	// order of return is the order of input
      }

      if(order==ZYX) {
        if(m(0, 2) > (1.0-EPSILON)) {
          cout << "North Pole" << endl;
          z = atan2(m(1, 0), m(1, 1));
          y = M_PI / 2.0;
          x = 0.0;
        }
        if(m(0, 2) < -(1.0-EPSILON)) {
          cout << "South Pole" << endl;
          z = atan2(m(1, 0), m(1, 1));
          y = -M_PI / 2.0;
          x = 0.0;
        }
        z = -atan2(m(0, 1), m(0, 0));
        y = asin(m(0, 2));
        x = -atan2(m(1, 2), m(2, 2));
        return Vector3d(z,y,x);	// order of return is the order of input
      }

      if(order==YZX) {
        if(m(0, 1) > (1.0-EPSILON)) {
          cout << "North Pole" << endl;
          y = atan2(m(1, 2), m(1, 0));
          z = -M_PI / 2.0;
          x = 0.0;
        }
        if(m(0, 1) < -(1.0-EPSILON)) {
          cout << "South Pole" << endl;
          y = atan2(m(1, 2), m(1, 0));
          z = M_PI / 2.0;
          x = 0.0;
        }
        y = atan2(m(0, 2), m(0, 0));
        z = -asin(m(0, 1));
        x = atan2(m(2, 1), m(1, 1));
        return Vector3d(y,z,x);	// order of return is the order of input
      }

      if(order==XZY) {
        if(m(1, 0) > (1.0-EPSILON)) {
          cout << "North Pole" << endl;
          x = -atan2(m(0, 2), m(0, 1));
          z = M_PI / 2.0;
          y = 0.0;
        }
        if(m(1, 0) < -(1.0-EPSILON)) {
          cout << "South Pole" << endl;
          x = -atan2(m(0, 2), m(0, 1));
          z = -M_PI / 2.0;
          y = 0.0;
        }
        x = -atan2(m(1, 2), m(1, 1));
        z = asin(m(1, 0));
        y = -atan2(m(2, 0), m(0, 0));
        return Vector3d(x,z,y);	// order of return is the order of input
      }

      if(order==YXZ){
        if(m(2, 1) > (1.0-EPSILON)) {
          cout << "North Pole" << endl;
          y = atan2(m(0, 2), m(0, 0));
          x = M_PI / 2.0;
          z = 0.0;
        }
        if(m(2, 1) < -(1.0-EPSILON)) {
          cout << "South Pole" << endl;
          y = atan2(m(0, 2), m(0, 0));
          x = -M_PI / 2.0;
          z = 0.0;
        }
        y = -atan2(m(2, 0), m(2, 2));
        x = asin(m(2, 1));
        z = -atan2(m(0, 1), m(1, 1));
        return Vector3d(y, x, z);	// order of return is the order of input
      }

      if(order==ZXY){
        if(m(1, 2) > (1.0-EPSILON)) {
          cout << "North Pole" << endl;
          z = -atan2(m(0, 1), m(0, 0));
          x = -M_PI / 2.0;
          y = 0.0;
        }
        if(m(1, 2) < -(1.0-EPSILON)) {
          cout << "South Pole" << endl;
          z = -atan2(m(0, 1), m(0, 0));
          x = M_PI / 2.0;
          y = 0.0;
        }
        z = atan2(m(1, 0), m(1, 1));
        x = -asin(m(1, 2));
        y = atan2(m(0, 2), m(2, 2));
        return Vector3d(z,x,y);	// order of return is the order of input
      }
      printf("Matrix_to_Euler - Do not support rotation order %d. Make sure letters are in lowercase\n", order);
      return Vector3d(0.0, 0.0, 0.0);
    }

    Matrix3d euler_to_matrix(Vector3d& v, RotationOrder order) {
      if(order==XYZ){
        return euler_to_matrixz(v[2])*euler_to_matrixy(v[1])*euler_to_matrixx(v[0]);
      }
      if(order==ZYX){
        return euler_to_matrixx(v[0])*euler_to_matrixy(v[1])*euler_to_matrixz(v[2]);
      }
      if(order==YZX){
        return euler_to_matrixx(v[0])*euler_to_matrixz(v[2])*euler_to_matrixy(v[1]);
      }
      if(order==XZY){
        return euler_to_matrixy(v[1])*euler_to_matrixz(v[2])*euler_to_matrixx(v[0]);
      }
      if(order==YXZ){
        return euler_to_matrixz(v[2])*euler_to_matrixx(v[0])*euler_to_matrixy(v[1]);
      }
      if(order==ZXY){
        return euler_to_matrixy(v[1])*euler_to_matrixx(v[0])*euler_to_matrixz(v[2]);
      }

      printf("euler_to_matrix - Do not support rotation order %d. Make sure letters are in lowercase\n", order);
      return Matrix3d::Zero();
    }

    Matrix3d euler_to_matrixx(double x) {
      Matrix3d mat = Matrix3d::Zero();
      double cosangle = cos(x);
      double sinangle = sin(x);
      mat(0, 0) = 1.0; 
      mat(1, 1) = cosangle; 
      mat(1, 2) = -sinangle; 
      mat(2, 1) = sinangle; 
      mat(2, 2) = cosangle; 
      return mat;
    }

    Matrix3d euler_to_matrixy(double y) {
      Matrix3d mat = Matrix3d::Zero();
      double cosangle = cos(y);
      double sinangle = sin(y);
      mat(1, 1) = 1.0; 
      mat(2, 2) = cosangle; 
      mat(2, 0) = -sinangle; 
      mat(0, 2) = sinangle; 
      mat(0, 0) = cosangle; 
      return mat;
    }

    Matrix3d euler_to_matrixz(double z) {
      Matrix3d mat = Matrix3d::Zero();
      double cosangle = cos(z);
      double sinangle = sin(z);
      mat(2, 2) = 1.0; 
      mat(0, 0) = cosangle; 
      mat(0, 1) = -sinangle; 
      mat(1, 0) = sinangle; 
      mat(1, 1) = cosangle; 
      return mat;
    }

  } // namespace rot_conv
} // namespace utils
