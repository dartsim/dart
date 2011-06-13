/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_TRANSFORMATION_H
#define MODEL3D_TRANSFORMATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <vector>
using namespace std;
#include "utils/Misc.h" // For M_PI
#include "renderer/OpenGLRenderInterface.h"

namespace model3d {
#define MAX_TRANSFORMATION_NAME 182

  class Joint;
  class Dof;

#ifndef _AXIS
#define _AXIS
#define _X 0
#define _Y 1
#define _Z 2
#endif

  class Transformation {
  public:
    enum TransFormType {
      T_ROTATEX,
      T_ROTATEY,
      T_ROTATEZ,
      T_ROTATEEXPMAP,
      T_ROTATEQUAT,
      T_TRANSLATE,
      T_TRANSLATEX,
      T_TRANSLATEY,
      T_TRANSLATEZ
    };

  public:
    Transformation();
    virtual ~Transformation();

    TransFormType getType() { return mType; }

    int getModelIndex() { return mModelIndex; }
    void setModelIndex(int _idx) { mModelIndex = _idx; }
	
    Joint* getJoint() { return mJoint; }
    void setJoint(Joint *_joint) { mJoint = _joint; }

    int getNumDofs() { return mDofs.size(); }
    Dof* getDof(int i) { return mDofs[i]; }
	
    bool getVariable() { return mVariable; }
    void setVariable(bool _var) { mVariable = _var; }

    void setDirty() { isDirty = true; }
    Matrix4d getTransform();
	
    bool isPresent(Dof *d);	// true if d is present in the dof list
    virtual Matrix4d getInvTransform();
	
    virtual void applyTransform(Vector3d& _v);
    virtual void applyTransform(Matrix4d& _m);
    virtual void applyInvTransform(Vector3d& _v);
    virtual void applyInvTransform(Matrix4d& _m);

    // TODO: old code has implementations (but what are they?)
    virtual void applyDeriv(Dof* q, Vector3d& v);
    virtual void applyDeriv(Dof* q, Matrix4d& m);
    virtual void applyDeriv2(Dof* q1, Dof* q2, Vector3d& v);
    virtual void applyDeriv2(Dof* q1, Dof* q2, Matrix4d& m);

	virtual void applyGLTransform(Renderer::OpenGLRenderInterface* RI) = 0;	// apply transform in GL
    virtual void evalTransform() = 0;	// computes and stores in above
    virtual Matrix4d getDeriv(Dof *q) = 0;	// get derivative wrt to a dof
    virtual Matrix4d getDeriv2(Dof *q1, Dof *q2) = 0;	// get derivative wrt to 2 dofs present in a transformation

  protected:
    vector<Dof*> mDofs;	// collection of Dofs
    TransFormType mType;
    int mModelIndex;	// position in the model transform vector
    char mName[MAX_TRANSFORMATION_NAME];

    Joint* mJoint;	// Transformation associated with
    bool mVariable;	// true when it is a variable and included int he model
    Matrix4d mTransform;	// transformation matrix will be stored here
	
    bool isDirty;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace model3d

#endif // #ifndef MODEL3D_TRANSFORMATION_H

