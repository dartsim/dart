/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#include "EigenHelper.h"

namespace eigenhelper {
    Eigen::Vector3d xform(const Eigen::Matrix4d& m, const Eigen::Vector3d& v) {
        Eigen::Vector4d x(v(0), v(1), v(2), 1.0);
        Eigen::Vector4d y = m * x;
        return Eigen::Vector3d(y(0), y(1), y(2));
    }
  
} // namespace eigenhelper
