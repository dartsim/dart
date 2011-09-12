/*
    RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
    All rights reserved.

    Author  Sehoon Ha
    Date    06/12/2011
*/

#ifndef KINEMATICS_TRANSFORMATION_H
#define KINEMATICS_TRANSFORMATION_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "renderer/RenderInterface.h"

namespace kinematics {
#define MAX_TRANSFORMATION_NAME 182

    class Joint;
    class Dof;

    enum AxisType{
        A_X=0,
        A_Y=1,
        A_Z=2
    };

    class Transformation {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

        inline TransFormType getType() const { return mType; }

        inline int getSkelIndex() const { return mSkelIndex; }
        inline void setSkelIndex(int _idx) { mSkelIndex = _idx; }

        inline Joint* getJoint() const { return mJoint; }
        inline void setJoint(Joint *_joint) { mJoint = _joint; }

        inline int getNumDofs() const { return mDofs.size(); }
        inline Dof* getDof(int i) const { return mDofs[i]; }

        inline bool getVariable() const { return mVariable; }
        inline void setVariable(bool _var) { mVariable = _var; }

        inline void setDirty() { mDirty = true; }
        Eigen::Matrix4d getTransform();

        bool isPresent(const Dof *d) const;	// true if d is present in the dof list
        virtual Eigen::Matrix4d getInvTransform();

        virtual void applyTransform(Eigen::Vector3d& _v);
        virtual void applyTransform(Eigen::Matrix4d& _m);
        virtual void applyInvTransform(Eigen::Vector3d& _v);
        virtual void applyInvTransform(Eigen::Matrix4d& _m);

        virtual void applyDeriv(const Dof* _q, Eigen::Vector3d& _v);
        virtual void applyDeriv(const Dof* _q, Eigen::Matrix4d& _m);
        virtual void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Vector3d& _v);
        virtual void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Matrix4d& _m);

        virtual void applyGLTransform(renderer::RenderInterface* _ri) const = 0;	// apply transform in GL
        virtual void computeTransform() = 0;	// computes and stores in above
        virtual Eigen::Matrix4d getDeriv(const Dof *_q) = 0;	// get derivative wrt to a dof
        virtual Eigen::Matrix4d getSecondDeriv(const Dof *_q1, const Dof *_q2) = 0;	// get derivative wrt to 2 dofs present in a transformation

    protected:
        std::vector<Dof *> mDofs;	// collection of Dofs
        TransFormType mType;
        int mSkelIndex;	// position in the model transform vector
        char mName[MAX_TRANSFORMATION_NAME];

        Joint *mJoint;	// Transformation associated with
        bool mVariable;	// true when it is a variable and included int he model
        Eigen::Matrix4d mTransform;	// transformation matrix will be stored here

        bool mDirty;
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_TRANSFORMATION_H

