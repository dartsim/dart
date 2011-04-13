// Delete me

#ifndef SRC_MODEL3D_PLANE_H
#define SRC_MODEL3D_PLANE_H

#include <Eigen/Dense>
using namespace Eigen;

namespace model3d {

  class Plane {
  public:
    // Constructor
    Plane(Vector3d _normal = vl_0, double _constant = 0.0);

    // Get signed distance to inPoint
    inline double getSignedDistance(const Vector3d &inPoint);
    // Get two vectors that together with mNormal form a basis for the plane
    inline void getBasisVector(Vector3d &inU, Vector3d &inV);

    inline Vector3d getNormal();
    inline double getConst();

    // Convert a point from world space to plane space (3D -> 2D)
    // assume the point is on this plane
    inline Vector2d convertWorldToPlane(const Vector3d &inPoint);
    // Convert a point from plane space to world space (2D -> 3D)
    inline Vector3d convertPlaneToWorld(const Vector2d &inPoint);

    // Get matrix that converts a point from plane space to world space (2D -> 3D)
    // v3D = mat * (v2D , 1)
    inline Mat3d getPlaneToWorldMatrix();

    // Transform a plane by the inverse of inInverseMatrix
    inline Plane transformedByInverse(const Mat4d &inInverseMatrix);

  protected:
    // Plane equation: mNormal.Dot(point) + mConstant == 0
    Vector3d mNormal;
    double mConstant;

    Vector3d mBasisU;
    Vector3d mBasisV;

    inline void setBasisVectors();
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_PLANE_H

