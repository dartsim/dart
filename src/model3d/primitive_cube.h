#ifndef SRC_MODEL3D_PRIMITIVE_CUBE_H
#define SRC_MODEL3D_PRIMITIVE_CUBE_H

#include "primitive.h"

namespace model3d {

  class PrimitiveCube : public Primitive {
  public:
    PrimitiveCube(Vector3d _dim, double _mass);
	
    virtual void draw(Renderer::OpenGLRenderInterface* RI, const Vector4d& _col, bool _default = true);
  private:
    void calMassTensor();
    void calVolume();
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_PRIMITIVE_CUBE_H

