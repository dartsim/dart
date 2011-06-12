#ifndef SRC_MODEL3D_TRFM_TRANSLATE_H
#define SRC_MODEL3D_TRFM_TRANSLATE_H

#include <cassert>
using namespace std;
#include "Transformation.h"

namespace model3d {
  class TrfmTranslate : public Transformation {
  public:
    TrfmTranslate(Dof *x, Dof *y, Dof *z, char *_name = NULL);
	
    Matrix4d getInvTransform();
    void applyTransform(Vector3d&);
    void applyTransform(Matrix4d&);
    void applyInvTransform(Vector3d&);
    void applyInvTransform(Matrix4d&);

    void applyGLTransform(Renderer::OpenGLRenderInterface* RI);
    void evalTransform();
	
    Matrix4d getDeriv(Dof *q);
    void applyDeriv(Dof* q, Vector3d& v);
    void applyDeriv(Dof* q, Matrix4d& m);

    Matrix4d getDeriv2(Dof *q1, Dof *q2){ return Matrix4d(); } // TODO::placeholder to run code
//    void applyDeriv2(Dof* q1, Dof* q2, Vector3d& v);
//    void applyDeriv2(Dof* q1, Dof* q2, Matrix4d& m);
  };

  class TrfmTranslateX : public Transformation {
  public:
    TrfmTranslateX(Dof *x, char *_name = NULL);
	
    Matrix4d getInvTransform();
    void applyTransform(Vector3d&);
    void applyTransform(Matrix4d&);
    void applyInvTransform(Vector3d&);
    void applyInvTransform(Matrix4d&);

    void applyGLTransform(Renderer::OpenGLRenderInterface* RI);
    void evalTransform();
	
    Matrix4d getDeriv(Dof *q);
    void applyDeriv(Dof* q, Vector3d& v);
    void applyDeriv(Dof* q, Matrix4d& m);

//    Matrix4d getDeriv2(Dof *q1, Dof *q2);
//    void applyDeriv2(Dof* q1, Dof* q2, Vector3d& v);
//    void applyDeriv2(Dof* q1, Dof* q2, Matrix4d& m);
  };

  class TrfmTranslateY : public Transformation {
  public:
    TrfmTranslateY(Dof *y, char *_name = NULL);
	
    Matrix4d getInvTransform();
    void applyTransform(Vector3d&);
    void applyTransform(Matrix4d&);
    void applyInvTransform(Vector3d&);
    void applyInvTransform(Matrix4d&);

    void applyGLTransform(Renderer::OpenGLRenderInterface* RI);
    void evalTransform();
	
    Matrix4d getDeriv(Dof *q);
    void applyDeriv(Dof* q, Vector3d& v);
    void applyDeriv(Dof* q, Matrix4d& m);

 //   Matrix4d getDeriv2(Dof *q1, Dof *q2);
 //   void applyDeriv2(Dof* q1, Dof* q2, Vector3d& v);
 //   void applyDeriv2(Dof* q1, Dof* q2, Matrix4d& m);
  };

  class TrfmTranslateZ : public Transformation {
  public:
    TrfmTranslateZ(Dof *z, char *_name = NULL);
	
    Matrix4d getInvTransform();
    void applyTransform(Vector3d&);
    void applyTransform(Matrix4d&);
    void applyInvTransform(Vector3d&);
    void applyInvTransform(Matrix4d&);

    void applyGLTransform(Renderer::OpenGLRenderInterface* RI);
    void evalTransform();
	
    Matrix4d getDeriv(Dof *q);
    void applyDeriv(Dof* q, Vector3d& v);
    void applyDeriv(Dof* q, Matrix4d& m);

//    Matrix4d getDeriv2(Dof *q1, Dof *q2);
//    void applyDeriv2(Dof* q1, Dof* q2, Vector3d& v);
//    void applyDeriv2(Dof* q1, Dof* q2, Matrix4d& m);
  };
} // namespace model3d

#endif // #ifndef SRC_MODEL3D_TRFM_TRANSLATE_H

