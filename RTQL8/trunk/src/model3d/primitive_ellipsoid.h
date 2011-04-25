#ifndef SRC_MODEL3D_PRIMITIVE_ELLIPSOID_H
#define SRC_MODEL3D_PRIMITIVE_ELLIPSOID_H

#include "primitive.h"

namespace model3d {

  class PrimitiveEllipsoid : public Primitive {
  public:
    PrimitiveEllipsoid(Vector3d _dim, double Mass); 
	
    void draw(Vector4d& _col, bool _default = true);
  private:
    void calMassTensor();
    void calVolume();
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_PRIMITIVE_ELLIPSOID_H


