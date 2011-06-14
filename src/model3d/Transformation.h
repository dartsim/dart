/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_TRANSFORMATION_H
#define MODEL3D_TRANSFORMATION_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Renderer {
    class OpenGLRenderInterface;
} // namespace Renderer

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
        Eigen::Matrix4d getTransform();
	
        bool isPresent(Dof *d);	// true if d is present in the dof list
        virtual Eigen::Matrix4d getInvTransform();
	
        virtual void applyTransform(Eigen::Vector3d& _v);
        virtual void applyTransform(Eigen::Matrix4d& _m);
        virtual void applyInvTransform(Eigen::Vector3d& _v);
        virtual void applyInvTransform(Eigen::Matrix4d& _m);

        // TODO: old code has implementations (but what are they?)
        virtual void applyDeriv(Dof* q, Eigen::Vector3d& v);
        virtual void applyDeriv(Dof* q, Eigen::Matrix4d& m);
        virtual void applyDeriv2(Dof* q1, Dof* q2, Eigen::Vector3d& v);
        virtual void applyDeriv2(Dof* q1, Dof* q2, Eigen::Matrix4d& m);

	virtual void applyGLTransform(Renderer::OpenGLRenderInterface* RI) = 0;	// apply transform in GL
        virtual void evalTransform() = 0;	// computes and stores in above
        virtual Eigen::Matrix4d getDeriv(Dof *q) = 0;	// get derivative wrt to a dof
        virtual Eigen::Matrix4d getDeriv2(Dof *q1, Dof *q2) = 0;	// get derivative wrt to 2 dofs present in a transformation

    protected:
        std::vector<Dof *> mDofs;	// collection of Dofs
        TransFormType mType;
        int mModelIndex;	// position in the model transform vector
        char mName[MAX_TRANSFORMATION_NAME];

        Joint *mJoint;	// Transformation associated with
        bool mVariable;	// true when it is a variable and included int he model
        Eigen::Matrix4d mTransform;	// transformation matrix will be stored here
	
        bool isDirty;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

} // namespace model3d

#endif // #ifndef MODEL3D_TRANSFORMATION_H

