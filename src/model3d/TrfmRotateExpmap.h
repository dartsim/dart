#ifndef SRC_MODEL3D_TRFM_ROTATE_EXPMAP_H
#define SRC_MODEL3D_TRFM_ROTATE_EXPMAP_H

#include "Transformation.h"
//#include "moremath.h"
#include <cmath>
#include <cassert>
using namespace std;

namespace model3d {

  class TrfmRotateExpMap: public Transformation{
  public:
    TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, char* name = NULL);

    Matrix4d getInvTransform();

    void applyGLTransform(Renderer::OpenGLRenderInterface* RI);
    void evalTransform();
    Matrix4d getDeriv(Dof *);
    Matrix4d getDeriv2(Dof *q1, Dof *q2);
	
  private:
    // some math utility functions
    double get_dq_dv(int i, int j, double theta, Vector3d v, Vector3d vhat);
    double get_dq_dv_approx(int i, int j, double theta, Vector3d v);
    double get_dq_dv_dv(int i, int j, int k, double theta, Vector3d vhat);
    double get_dq_dv_dv_approx(int i, int j, int k, double theta, Vector3d v);
  };
} // namespace model3d

#endif // SRC_MODEL3D_TRFM_ROTATE_EXPMAP_H
