/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
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

/*! \mainpage Dynamic Animation and Robotics Toolkits

DART is an open source library for developing kinematic and dynamic
applications in robotics and computer animation. DART features two
main components: a multibody dynamic simulator developed by Georgia
Tech Graphics Lab and a variety of motion planning algorithms
developed by Georgia Tech Humanoid Robotics Lab. This document focuses
only on the dynamic simulator.

The multibody dynamics simulator in DART is designed to aid
development of motion control algorithms. DART uses generalized
coordinates to represent articulated rigid body systems and computes
Lagrange’s equations derived from D’Alembert’s principle to describe
the dynamics of motion. In contrast to many popular physics engines
which view the simulator as a black box, DART provides full access to
internal kinematic and dynamic quantities, such as mass matrix,
Coriolis and centrifugal force, transformation matrices and their
derivatives, etc. DART also provides efficient computation of Jacobian
matrices for arbitrary body points and coordinate frames.

The contact and collision are handled using an implicit time-stepping,
velocity-based LCP (linear-complementarity problem) to guarantee
non-penetration, directional friction, and approximated Coulombs
friction cone conditions. The LCP problem is solved efficiently by
Lemke's algorithm. For the collision detection, DART directly uses FCL
package developed by UNC Gamma Lab.

In addition, DART supports various joint types (ball-and-socket,
universal, hinge, and prismatic joints) and arbitrary meshes. DART
also provides two explicit integration methods: first-order
Runge-Kutta and fourth-order Runge Kutta.

*/

#ifndef DART_KINEMATICS_BODYNODE_H
#define DART_KINEMATICS_BODYNODE_H

#include <vector>
#include <Eigen/Dense>

#include "utils/Deprecated.h"
#include "math/EigenHelper.h"
#include "math/UtilsMath.h"

namespace renderer { class RenderInterface; }

namespace kinematics
{

#define MAX_NODE3D_NAME 128

class Marker;
class Dof;
class Transformation;
class Shape;
class Skeleton;
class Joint;

/// @brief BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
/// connected and have a set of core functions for calculating derivatives.
/// Mostly automatically constructed by FileInfoSkel.
/// @see FileInfoSkel.
class BodyNode {
public:
    // We need this aligned allocator because we have Matrix4d as members in
    // this class.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    /// @brief Default constructor. The name can be up to 128.
    BodyNode(const char *_name = NULL);

    /// @brief Default destructor.
    virtual ~BodyNode();

    /// @brief Initialize the vector memebers with proper sizes.
    void init();

    /// @brief
    void setWorldTransform(const Eigen::Matrix4d& _W) { mW = _W; }

    /// @brief Transformation from the local coordinates of this body node to
    /// the world coordinates
    Eigen::Matrix4d getWorldTransform() const { return mW; }

    /// @brief Transformation from the world coordinates to the local
    /// coordinates of this body node
    Eigen::Matrix4d getWorldInvTransform() const { return mW.inverse(); }

    /// @brief Transformation from the local coordinates of this body node to
    /// the local coordinates of its parent
    Eigen::Matrix4d getLocalTransform() const { return mT; }

    /// @briefTransformation from the local coordinates of the parent node to
    /// the local coordinates of this body node
    Eigen::Matrix4d getLocalInvTransform() const { return mT.inverse(); }

    /// @brief Given a 3D vector lp in the local coordinates of this body node.
    /// @return The world coordinates of this vector
    Eigen::Vector3d evalWorldPos(const Eigen::Vector3d& _lp) const;

    /// @brief Set up the list of dependent dofs.
    void setDependDofList();

    /// @brief Test whether this dof is dependent or not.
    /// @warning You may want to use getNumDependentDofs / getDependentDof for
    /// efficiency.
    bool dependsOn(int _dofIndex) const;

    /// @brief The number of the dofs by which this node is affected.
    int getNumDependentDofs() const { return mDependentDofs.size(); }

    /// @brief Return an dof index from the array index (< getNumDependentDofs).
    int getDependentDof(int _arrayIndex) { return mDependentDofs[_arrayIndex]; }

    /// @brief Render the entire subtree rooted at this body node.
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true,
              int _depth = 0) const;

    /// @brief Render the markers
    void drawMarkers(renderer::RenderInterface* _ri = NULL,
                     const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                     bool _useDefaultColor = true) const;

    /// @brief
    void setName(const char* _name) { strcpy(mName, _name); }

    /// @brief
    char* getName() { return mName; }

    /// @brief
    void setLocalCOM(const Eigen::Vector3d& _off) { mCOMLocal = _off; }

    /// @brief
    const Eigen::Vector3d& getLocalCOM() const { return mCOMLocal; }

    /// @brief
    Eigen::Vector3d getWorldCOM() const { return evalWorldPos(mCOMLocal); }

    /// @brief
    void setSkel(Skeleton* _skel) { mSkel = _skel; }

    /// @brief
    Skeleton* getSkel() const { return mSkel; }

    /// @brief
    void setSkelIndex(int _idx) { mSkelIndex = _idx; }

    /// @brief
    int getSkelIndex() const { return mSkelIndex; }

    /// @brief
    BodyNode* getParentNode() const { return mParentNode; }

    /// @brief
    void setMass(double _mass) { mMass = _mass; }

    /// @brief
    double getMass() const { return mMass; }

    /// @brief
    void setLocalInertia(double _Ixx, double _Iyy, double _Izz,
                         double _Ixy, double _Ixz, double _Iyz)
    {
        mI(0,0) = _Ixx; mI(0,1) = _Ixy; mI(0,2) = _Ixz;
        mI(1,0) = _Ixy; mI(1,1) = _Iyy; mI(1,2) = _Iyz;
        mI(2,0) = _Ixz; mI(2,1) = _Iyz; mI(2,2) = _Izz;
    }

    /// @brief
    void setLocalInertia(const Eigen::Matrix3d& _inertia)
    { mI = _inertia; }

    /// @brief
    const Eigen::Matrix3d& getLocalInertia() const { return mI; }

    /// @brief
    const Eigen::Matrix3d& getWorldInertia() const { return mIc; }

    /// @brief
    void addMarker(Marker *_h) { mMarkers.push_back(_h); }

    /// @brief
    int getNumMarkers() const { return mMarkers.size(); }

    /// @brief
    Marker* getMarker(int _idx) const { return mMarkers[_idx]; }

    /// @brief
    void addShape(Shape *_p) { mVizShapes.push_back(_p); mColShapes.push_back(_p); }

    /// @brief
    int getNumShapes() const { return mVizShapes.size(); }

    /// @brief
    Shape* getShape(int _idx) const { return mVizShapes[_idx]; }

    /// @brief
    void addVisualizationShape(Shape *_p) { mVizShapes.push_back(_p); }

    /// @brief
    int getNumVisualizationShapes() const { return mVizShapes.size(); }

    /// @brief
    Shape* getVisualizationShape(int _idx) const { return mVizShapes[_idx]; }

    /// @brief
    void addCollisionShape(Shape *_p) { mColShapes.push_back(_p); }

    /// @brief
    int getNumCollisionShapes() const { return mColShapes.size(); }

    /// @brief
    Shape* getCollisionShape(int _idx) const { return mColShapes[_idx]; }

    /// @brief
    void addChildJoint(Joint *_c) { mJointsChild.push_back(_c); }

    /// @brief
    int getNumChildJoints() const { return mJointsChild.size(); }

    /// @brief
    Joint* getChildJoint(int _idx) const { return mJointsChild[_idx]; }

    /// @brief
    Joint* getParentJoint() const { return mParentJoint; }

    /// @brief
    void setParentJoint(Joint* _p);

    /// @brief
    void setColliding(bool _colliding) { mColliding = _colliding; }

    /// @brief
    bool getColliding() const { return mColliding; }

    // wrapper functions for joints
    /// @brief
    BodyNode* getChildNode(int _idx) const;

    /// @brief
    int getNumLocalDofs() const;

    /// @brief
    Dof* getDof(int _idx) const;

    /// @brief
    bool isPresent(const Dof* _q);

    /// @brief
    bool getCollideState() const { return mCollidable; }

    /// @brief
    void setCollideState(bool _c) { mCollidable = _c; }

    /// @brief
    const Matrix4d& getDerivLocalTransform(int _index) const;

    /// @brief
    const Matrix4d& getDerivWorldTransform(int _index) const;

    /// @brief Update the first derivatives of the transformations
    void updateFirstDerivatives();

    //--------------------------------------------------------------------------
    // Sub-functions for kinematics
    //--------------------------------------------------------------------------
    /// @brief Update local transformations and world transformations.
    /// T(i-1,i), W(i)
    void updateTransform();

    Eigen::Matrix4d getLocalDeriv(Dof *_q) const; ///< First derivative of the local transformation w.r.t. the input dof.
    Eigen::Matrix3d getInertia() const { return mIc; } ///< Superseded by getWorldInertia()
    Eigen::Matrix4d getMassTensor() const; ///< Computes the "mass tensor" in lagrangian dynamics from the inertia matrix.
    const MatrixXd& getJacobianLinear() const;
    const MatrixXd& getJacobianAngular() const;
    void evalVelocity(const Eigen::VectorXd &_qDotSkel); ///< Evaluates the velocity of the COM in the world frame.
    void evalOmega(const Eigen::VectorXd &_qDotSkel);    ///< Evaluates the Omega in the world frame.
    Eigen::Vector3d mVel;    ///< Linear velocity in the world frame
    Eigen::Vector3d mOmega;  ///< Angular velocity in the world frame
    void evalJacLin(); ///< Evaluate linear Jacobian of this body node (num cols == num dependent dofs)
    void evalJacAng(); ///< Evaluate angular Jacobian of this body node (num cols == num dependent dofs)

protected:
    /// @brief Name
    char mName[MAX_NODE3D_NAME];

    /// @brief Index in the model
    int mSkelIndex;

    /// @brief Visual geometry of this body node
    std::vector<Shape*> mVizShapes;

    /// @brief Collision geometry of this body node
    std::vector<Shape*> mColShapes;

    /// @brief List of joints that link to child nodes
    std::vector<Joint*> mJointsChild;

    /// @brief Joint connecting to parent node
    Joint* mParentJoint;

    /// @brief Parent node
    BodyNode *mParentNode;

    /// @brief List of markers associated
    std::vector<Marker*> mMarkers;

    /// @brief whether the node is currently in collision with another node
    bool mColliding;

    /// @brief A list of dependent dof indices
    std::vector<int> mDependentDofs;

    /// @brief keep track of the root translation DOFs only if they are the
    /// first ones
    int mNumRootTrans;

    /// @brief Pointer to the model this body node belongs to.
    Skeleton* mSkel;

    //--------------------------------------------------------------------------
    // TRANSFORMATIONS
    //--------------------------------------------------------------------------
    /// @brief Local transformation from parent to itself
    Eigen::Matrix4d mT;

    /// @brief Global transformation.
    Eigen::Matrix4d mW;

    double mMass; ///< Mass of this node; zero if no primitive
    Eigen::Vector3d mCOMLocal; ///< COM of this body node in its local coordinate frame.
    Eigen::Matrix3d mI;  ///< Inertia matrix in the body frame; defaults to Shape's inertia matrix
    Eigen::Matrix3d mIc; ///< Inertia matrix in the world frame = R*Ibody*RT; updated by evalTransform
    EIGEN_V_MAT4D mTq;   ///< Partial derivative of local transformation wrt local dofs; each element is a 4x4 matrix
    EIGEN_V_MAT4D mWq;   ///< Partial derivative of world transformation wrt all dependent dofs; each element is a 4x4 matrix
    Eigen::MatrixXd mJv; ///< Linear Jacobian; Cartesian_linear_velocity of the COM = mJv * generalized_velocity
    Eigen::MatrixXd mJw; ///< Angular Jacobian; Cartesian_angular_velocity = mJw * generalized_velocity

private:
    /// @brief A unique ID of this node globally.
    int mID;

    /// @brief Counts the number of nodes globally.
    static int msBodyNodeCount;

    /// @brief Indicating whether this node is collidable.
    bool mCollidable;
};

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_BODYNODE_H

