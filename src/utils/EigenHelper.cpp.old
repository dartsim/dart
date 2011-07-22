/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#include "EigenHelper.h"
using namespace Eigen;

namespace eigenhelper {
    Vector3d xformHom(const Matrix4d& m, const Vector3d& v) {
        Vector4d x(v(0), v(1), v(2), 1.0);
        Vector4d y = m * x;
        return Vector3d(y(0), y(1), y(2));
    }

    Vector3d xformHomDir(const Matrix4d& m, const Vector3d& v) {
        return m.topLeftCorner(3,3)*v;
    }
  
} // namespace eigenhelper
