#ifndef SRC_MODEL3D_TRFM_ROTATE_QUAT_H
#define SRC_MODEL3D_TRFM_ROTATE_QUAT_H

#include "transformation.h"

namespace model3d {

  class TrfmRotateQuat: public Transformation {
  public:
    TrfmRotateQuat(Dof *w, Dof *x, Dof *y, Dof *z, char *_name = NULL);
	
    void applyGLTransform(Renderer::OpenGLRenderInterface* RI);
    void evalTransform();
    Matrix4d getDeriv(Dof *);
    Matrix4d getDeriv2(Dof *, Dof *);
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_TRFM_ROTATE_QUAT_H

