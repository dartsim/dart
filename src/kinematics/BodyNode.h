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

#ifndef KINEMATICS_BODYNODE_H
#define KINEMATICS_BODYNODE_H

#include <vector>
#include <Eigen/Dense>
#include "renderer/RenderInterface.h"
#include "utils/EigenHelper.h"

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
        inline int getNumDependentDofs() const { return mDependentDofs.size(); } ///< The number of the dofs which this node is affected
        inline int getDependentDof(int _arrayIndex) { return mDependentDofs[_arrayIndex]; } ///< Return an dof index from the array index (< getNumDependentDofs)

        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(), bool _useDefaultColor = true, int _depth = 0) const ;    ///< Render the entire subtree rooted at this body node
        void drawMarkers(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const ;    ///< Render the markers

        inline char* getName() { return mName; }

        inline void setLocalCOM(const Eigen::Vector3d& _off) { mCOMLocal = _off; }
        inline Eigen::Vector3d getLocalCOM() const { return mCOMLocal; }
        inline Eigen::Vector3d getWorldCOM() { return evalWorldPos(mCOMLocal); }
        
        inline void setSkel(Skeleton* _skel) { mSkel = _skel; }
        inline Skeleton* getSkel() const { return mSkel; }

        inline void setSkelIndex(int _idx) { mSkelIndex = _idx; }
        inline int getSkelIndex() const { return mSkelIndex; }
        
        inline BodyNode* getParentNode() const { return mNodeParent; }
        inline double getMass() const { return mMass; }

        inline void addMarker(Marker *_h) { mMarkers.push_back(_h); }
        inline int getNumMarkers() const { return mMarkers.size(); }
        inline Marker* getMarker(int _idx) const { return mMarkers[_idx]; }

        inline void setShape(Shape *_p) { mShape = _p; }
        inline Shape* getShape() const { return mShape; }
        
        inline void addChildJoint(Joint *_c) { mJointsChild.push_back(_c); }
        inline int getNumChildJoints() { return mJointsChild.size(); }
        inline Joint* getChildJoint(int _idx) const { return mJointsChild[_idx]; }
        inline Joint* getParentJoint() const { return mJointParent; }
        void setParentJoint(Joint *_p);

        // wrapper functions for joints
        BodyNode* getChildNode(int _idx) const;
        int getNumLocalDofs() const;
        Dof* getDof(int _idx) const;
        bool isPresent(Dof *_q);

        Eigen::Matrix4d getDerivLocalTransform(int index) const;
        Eigen::Matrix4d getDerivWorldTransform(int index) const;
        Eigen::MatrixXd getJacobianLinear() const;
        Eigen::MatrixXd getJacobianAngular() const;
        
    protected:
        
        char mName[MAX_NODE3D_NAME]; ///< Name
        int mSkelIndex;    ///< Index in the model

        Shape *mShape;  ///< Geometry of this body node
        std::vector<Joint *> mJointsChild; ///< List of joints that link to child nodes
        Joint *mJointParent;    ///< Joint connecting to parent node
        BodyNode *mNodeParent;      ///< Parent node
        std::vector<Marker *> mMarkers; ///< List of markers associated

        std::vector<int> mDependentDofs; ///< A list of dependent dof indices 
        int mNumRootTrans;  ///< keep track of the root translation DOFs only if they are the first ones

        double mMass; ///< Mass of this node; zero if no primitive
        Eigen::Vector3d mCOMLocal; ///< COM of this body node in its local coordinate frame
        Skeleton *mSkel; ///< Pointer to the model this body node belongs to

        // transformations
        Eigen::Matrix4d mT; ///< Local transformation from parent to itself
        Eigen::Matrix4d mW; ///< Global transformation
        Eigen::Matrix3d mIc;    ///< Inertia matrix in the world frame = R*Ibody*RT; updated by evalTransform

        // first derivatives
        EIGEN_V_MAT4D mTq;  ///< Partial derivative of local transformation wrt local dofs; each element is a 4x4 matrix
        EIGEN_V_MAT4D mWq;  ///< Partial derivative of world transformation wrt all dependent dofs; each element is a 4x4 matrix
        Eigen::MatrixXd mJv; ///< Linear Jacobian; Cartesian_linear_velocity of the COM = mJv * generalized_velocity
        Eigen::MatrixXd mJw; ///< Angular Jacobian; Cartesian_angular_velocity = mJw * generalized_velocity


    private:
        int mID; ///< A unique ID of this node globally 
        static int msBodyNodeCount; ///< Counts the number of nodes globally
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_BODYNODE_H

