/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KINEMATICS_TRANSFORMATION_H
#define KINEMATICS_TRANSFORMATION_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace renderer { class RenderInterface; };

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
            T_ROTATEAXIS,
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

        inline char* getName() { return mName; }

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

