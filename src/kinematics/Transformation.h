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

#ifndef DART_KINEMATICS_TRANSFORMATION_H
#define DART_KINEMATICS_TRANSFORMATION_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "kinematics/System.h"

namespace renderer { class RenderInterface; }

namespace kinematics {

#define MAX_TRANSFORMATION_NAME 182

class Joint;
class Dof;

enum AxisType {
    A_X = 0,
    A_Y = 1,
    A_Z = 2
};

class Transformation : public System {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief
    enum TransFormType {
        T_ROTATEX,
        T_ROTATEY,
        T_ROTATEZ,
        T_ROTATEEXPMAP,
        T_ROTATEQUAT,
        T_TRANSLATE,
        T_TRANSLATEX,
        T_TRANSLATEY,
        T_TRANSLATEZ,
        T_ROTATEAXIS
    };

public:
    /// @brief Constructor.
    Transformation();

    /// @brief Destructor.
    virtual ~Transformation();

    /// @brief
    TransFormType getType() const { return mType; }

    /// @brief
    char* getName() { return mName; }

    /// @brief
    int getSkelIndex() const { return mSkelIndex; }

    /// @brief
    void setSkelIndex(int _idx) { mSkelIndex = _idx; }

    /// @brief
    Joint* getJoint() const { return mJoint; }

    /// @brief
    void setJoint(Joint *_joint) { mJoint = _joint; }

    /// @brief
    // TODO: What about changing the function name getVariable() to
    // isVariable() instead.
    bool isVariable() const { return mVariable; }

    /// @brief
    void setVariable(bool _var) { mVariable = _var; }

    /// @brief
    void setDirty() { mDirty = true; }

    /// @brief
    Eigen::Matrix4d getTransform();

    /// @brief true if d is present in the dof list.
    bool isPresent(const Dof *d) const;

    /// @brief
    virtual Eigen::Matrix4d getInvTransform();

    /// @brief
    virtual void applyTransform(Eigen::Vector3d& _v);

    /// @brief
    virtual void applyTransform(Eigen::Matrix4d& _m);

    /// @brief
    virtual void applyInvTransform(Eigen::Vector3d& _v);

    /// @brief
    virtual void applyInvTransform(Eigen::Matrix4d& _m);

    /// @brief
    virtual void applyDeriv(const Dof* _q, Eigen::Vector3d& _v);

    /// @brief
    virtual void applyDeriv(const Dof* _q, Eigen::Matrix4d& _m);

    /// @brief
    virtual void applySecondDeriv(const Dof* _q1, const Dof* _q2,
                                  Eigen::Vector3d& _v);

    /// @brief
    virtual void applySecondDeriv(const Dof* _q1, const Dof* _q2,
                                  Eigen::Matrix4d& _m);

    /// @brief Apply transform in GL.
    virtual void applyGLTransform(renderer::RenderInterface* _ri) const = 0;

    /// @brief Computes and stores in above.
    virtual void computeTransform() = 0;

    /// @brief Get derivative wrt to a dof.
    virtual Eigen::Matrix4d getDeriv(const Dof *_q) const = 0;

    /// @brief Get derivative wrt to 2 dofs present in a transformation.
    virtual Eigen::Matrix4d getSecondDeriv(const Dof *_q1,
                                           const Dof *_q2) const = 0;

    /// @brief Local Jacobian
    /// @return \f$ J \in R^{6 \times n} \f$ where \f$ n \f$ is a number of
    /// Dofs.
    // TODO: Use this instead of getDeriv
    virtual Eigen::MatrixXd getJacobian() const = 0;

protected:
    /// @brief
    TransFormType mType;

    /// @brief Position in the model transform vector.
    int mSkelIndex;

    /// @brief
    char mName[MAX_TRANSFORMATION_NAME];

    /// @brief Transformation associated with.
    Joint *mJoint;

    /// @brief true when it is a variable and included int he model.
    bool mVariable;

    /// @brief transformation matrix will be stored here.
    Eigen::Matrix4d mTransform;

    /// @brief
    bool mDirty;
};

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_TRANSFORMATION_H

