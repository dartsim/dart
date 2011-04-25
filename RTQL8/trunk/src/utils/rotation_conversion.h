#ifndef _ROTATIONCONVERSION_
#define _ROTATIONCONVERSION_

// External Libraries
#include <Eigen/Dense>
using namespace Eigen;
// Local Headers
#include "utils/misc.h"

namespace utils {
  namespace rot_conv {

    enum RotationOrder {XYZ, XZY, YZX, YXZ, ZXY, ZYX};

    Quaterniond matrix_to_quat(Matrix3d& m);	// forms the Quaterniond from a rotation matrix
    Quaterniond exp_to_quat(Vector3d& v);
    Vector3d quat_to_exp(Quaterniond& q);
    Matrix3d quat_to_matrix(Quaterniond& q);

    // Note: xyz order means matrix is Rz*Ry*Rx i.e a point as transformed as Rz*Ry*Rx(p)
    // coord sys transformation as in GL will be written as glRotate(z); glRotate(y); glRotate(x)
    Vector3d matrix_to_euler(Matrix3d& m, RotationOrder _order);
    Matrix3d euler_to_matrix(Vector3d& v, RotationOrder _order);

    Matrix3d euler_to_matrixx(double x);
    Matrix3d euler_to_matrixy(double y);
    Matrix3d euler_to_matrixz(double z);

    // Note: retrieves the derivative matrix given a quaternion
    // This is not functionality that is part of Eigen
    Matrix3d get_derivative_matrix(const Quaterniond& q, int el);
    Matrix3d get_derivative_matrix(const Quaterniond& q, int el1, int el2);

  } // namespace rot_conv
} // namespace utils

#endif
