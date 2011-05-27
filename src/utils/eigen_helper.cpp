#include "eigen_helper.h"

namespace eigenhelper {
  Vector3d xform(const Matrix4d& m, const Vector3d& v) {
    Vector4d x(v(0), v(1), v(2), 1.0);
    Vector4d y = m * x;
    return Vector3d(y(0), y(1), y(2));
  }
  
} // namespace eigenhelper
