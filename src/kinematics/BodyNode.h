/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/07/2011
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

#ifndef KINEMATICS_BODYNODE_H
#define KINEMATICS_BODYNODE_H

#include <vector>
#include <Eigen/Dense>
#include "utils/EigenHelper.h"
#include "utils/Deprecated.h"

namespace renderer { class RenderInterface; }

namespace kinematics {
#define MAX_NODE3D_NAME 128

    class Marker;
    class Dof;
    class Transformation;
    class Shape;
    class Skeleton;
    class Joint;

    /**
       @brief BodyNode class represents a single node of the skeleton.

       BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
       connected and have a set of core functions for calculating derivatives.
       Mostly automatically constructed by FileInfoSkel. @see FileInfoSkel.
    */
    class BodyNode {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // we need this aligned allocator because we have Matrix4d as members in this class

        BodyNode(const char *_name = NULL); ///< Default constructor. The name can be up to 128
        virtual ~BodyNode(); ///< Default destructor

        void init(); ///< Initialize the vector memebers with proper sizes
        void updateTransform(); ///< Update transformations w.r.t. the current dof values in Dof*
        void updateFirstDerivatives();  ///< Update the first derivatives of the transformations 
        void evalJacLin(); ///< Evaluate linear Jacobian of this body node (num cols == num dependent dofs)
        void evalJacAng(); ///< Evaluate angular Jacobian of this body node (num cols == num dependent dofs)

        inline void setWorldTransform(const Eigen::Matrix4d& _W) { mW = _W; }

        inline Eigen::Matrix4d getWorldTransform() const { return mW; } ///< Transformation from the local coordinates of this body node to the world coordinates
        inline Eigen::Matrix4d getWorldInvTransform() const { return mW.inverse(); } ///< Transformation from the world coordinates to the local coordinates of this body node
        inline Eigen::Matrix4d getLocalTransform() const { return mT; } ///< Transformation from the local coordinates of this body node to the local coordinates of its parent
        inline Eigen::Matrix4d getLocalInvTransform() const { return mT.inverse(); } ///< Transformation from the local coordinates of the parent node to the local coordinates of this body node

        Eigen::Vector3d evalWorldPos(const Eigen::Vector3d& _lp); ///< Given a 3D vector lp in the local coordinates of this body node, return the world coordinates of this vector

        Eigen::Matrix4d getLocalDeriv(Dof *_q) const; ///< First derivative of the local transformation w.r.t. the input dof

        /* void setDependDofMap(int _numDofs); ///< set up the dof dependence map for this node */
        /* bool dependsOn(int _dofIndex) const { return mDependsOnDof[_dofIndex]; } ///< NOTE: not checking index range */
        void setDependDofList(); ///< Set up the list of dependent dofs 
        bool dependsOn(int _dofIndex) const; ///< Test whether this dof is dependent or not \warning{You may want to use getNumDependentDofs / getDependentDof for efficiency}
        inline int getNumDependentDofs() const { return mDependentDofs.size(); } ///< The number of the dofs by which this node is affected
        inline int getDependentDof(int _arrayIndex) { return mDependentDofs[_arrayIndex]; } ///< Return an dof index from the array index (< getNumDependentDofs)

        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(), bool _useDefaultColor = true, int _depth = 0) const ;    ///< Render the entire subtree rooted at this body node
        void drawMarkers(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const ;    ///< Render the markers

        inline void setName(const char* _name) { strcpy(mName, _name); }
        inline char* getName() { return mName; }

        inline void setLocalCOM(const Eigen::Vector3d& _off) { mCOMLocal = _off; }
        inline Eigen::Vector3d getLocalCOM() const { return mCOMLocal; }
        inline Eigen::Vector3d getWorldCOM() { return evalWorldPos(mCOMLocal); }
        
        inline void setSkel(Skeleton* _skel) { mSkel = _skel; }
        inline Skeleton* getSkel() const { return mSkel; }

        inline void setSkelIndex(int _idx) { mSkelIndex = _idx; }
        inline int getSkelIndex() const { return mSkelIndex; }
        
        inline BodyNode* getParentNode() const { return mNodeParent; }

        inline void setMass(double _mass) { mMass = _mass; }
        inline double getMass() const { return mMass; }

        inline void setLocalInertia(double _Ixx, double _Iyy, double _Izz,
                                    double _Ixy, double _Ixz, double _Iyz) {
            mI(0,0) = _Ixx; mI(0,1) = _Ixy; mI(0,2) = _Ixz;
            mI(1,0) = _Ixy; mI(1,1) = _Iyy; mI(1,2) = _Iyz;
            mI(2,0) = _Ixz; mI(2,1) = _Iyz; mI(2,2) = _Izz;
        }
        inline void setLocalInertia(const Eigen::Matrix3d& _inertia) { mI = _inertia; }
        inline Eigen::Matrix3d getLocalInertia() const { return mI; }
        inline Eigen::Matrix3d getWorldInertia() const { return mIc; }
        DEPRECATED inline Eigen::Matrix3d getInertia() const { return mIc; } ///< Superseded by getWorldInertia()
        Eigen::Matrix4d getMassTensor(); ///< Computes the "mass tensor" in lagrangian dynamics from the inertia matrix

        inline void addMarker(Marker *_h) { mMarkers.push_back(_h); }
        inline int getNumMarkers() const { return mMarkers.size(); }
        inline Marker* getMarker(int _idx) const { return mMarkers[_idx]; }
        
        inline void setShape(Shape *_p) {
            mVizShape = _p;
            mColShape = _p;
        }
        inline Shape* getShape() const { return mVizShape; }
        inline void setVisualizationShape(Shape *_p) { mVizShape = _p; }
        inline Shape* getVisualizationShape() { return mVizShape; }
        inline void setCollisionShape(Shape *_p) { mColShape = _p; }
        inline Shape* getCollisionShape() const { return mColShape; }

        inline void addChildJoint(Joint *_c) { mJointsChild.push_back(_c); }
        inline int getNumChildJoints() { return mJointsChild.size(); }
        inline Joint* getChildJoint(int _idx) const { return mJointsChild[_idx]; }
        inline Joint* getParentJoint() const { return mJointParent; }
        void setParentJoint(Joint *_p);

        inline void setColliding(bool _colliding) { mColliding = _colliding; }
        inline bool getColliding() { return mColliding; }

        // wrapper functions for joints
        BodyNode* getChildNode(int _idx) const;
        int getNumLocalDofs() const;
        Dof* getDof(int _idx) const;
        bool isPresent(Dof *_q);

        Eigen::Matrix4d getDerivLocalTransform(int _index) const;
        Eigen::Matrix4d getDerivWorldTransform(int _index) const;
        Eigen::MatrixXd getJacobianLinear() const;
        Eigen::MatrixXd getJacobianAngular() const;
        
        inline bool getCollideState() const { return mCollidable; }
        inline void setCollideState(bool _c) { mCollidable = _c; }

    protected:
        
        char mName[MAX_NODE3D_NAME]; ///< Name
        int mSkelIndex;    ///< Index in the model

        Shape *mVizShape;  ///< Visual geometry of this body node
        Shape *mColShape;  ///< Collision geometry of this body node
        std::vector<Joint *> mJointsChild; ///< List of joints that link to child nodes
        Joint *mJointParent;    ///< Joint connecting to parent node
        BodyNode *mNodeParent;      ///< Parent node
        std::vector<Marker *> mMarkers; ///< List of markers associated
        bool mColliding; ///< whether the node is currently in collision with another node

        std::vector<int> mDependentDofs; ///< A list of dependent dof indices 
        int mNumRootTrans;  ///< keep track of the root translation DOFs only if they are the first ones

        double mMass; ///< Mass of this node; zero if no primitive
        Eigen::Vector3d mCOMLocal; ///< COM of this body node in its local coordinate fram
        Skeleton *mSkel; ///< Pointer to the model this body node belongs to

        // transformations
        Eigen::Matrix4d mT; ///< Local transformation from parent to itself
        Eigen::Matrix4d mW; ///< Global transformation
        Eigen::Matrix3d mI;	///< Inertia matrix in the body frame; defaults to Shape's inertia matrix
        Eigen::Matrix3d mIc;    ///< Inertia matrix in the world frame = R*Ibody*RT; updated by evalTransform

        // first derivatives
        EIGEN_V_MAT4D mTq;  ///< Partial derivative of local transformation wrt local dofs; each element is a 4x4 matrix
        EIGEN_V_MAT4D mWq;  ///< Partial derivative of world transformation wrt all dependent dofs; each element is a 4x4 matrix
        Eigen::MatrixXd mJv; ///< Linear Jacobian; Cartesian_linear_velocity of the COM = mJv * generalized_velocity
        Eigen::MatrixXd mJw; ///< Angular Jacobian; Cartesian_angular_velocity = mJw * generalized_velocity


    private:
        int mID; ///< A unique ID of this node globally 
        static int msBodyNodeCount; ///< Counts the number of nodes globally
        bool mCollidable; ///< Indicating whether this node is collidable
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_BODYNODE_H

