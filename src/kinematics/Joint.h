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

#ifndef KINEMATICS_JOINT_H
#define KINEMATICS_JOINT_H

#include <vector>
#include <Eigen/Dense>
#include "utils/UtilsRotation.h"

namespace kinematics {

    class Dof;
    class BodyNode;
    class Transformation;

    class Joint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // we need this aligned allocator because we have Matrix4d as members in this class

        enum JointType {
            J_UNKNOWN, 
            J_FREEEULER, 
            J_FREEEXPMAP, 
            J_BALLEULER, 
            J_BALLEXPMAP, 
            J_HINGE, 
            J_UNIVERSAL, 
            J_TRANS
        };
	
        Joint(BodyNode* _bParent, BodyNode* _bChild, const char *_name=NULL);
        virtual ~Joint();

        bool isPresent (const Dof *_d) const;	///< true if d is present in the dof list for the joint
        int getDofLocalIndex (int _dofSkelIndex) const; ///< get local index of the dof at this joint; if the dof is not presented at this joint, return -1
        int getFirstDofIndex() const;    ///< index of the first local dof (dof 0) of the joint in the full skel dof list
        int getFirstRotDofIndex() const;    ///< index of the first rot local dof (dof 0) of the joint in the full skel dof list
        int getFirstTransDofIndex() const;    ///< index of the first trans local dof (dof 0) of the joint in the full skel dof list

        Eigen::Matrix4d getLocalTransform(); ///< computes and returns the local transformation from NodeIn to NodeOut
        void applyTransform(Eigen::Vector3d& _v); ///< apply the local transformation to a vector _v
        void applyTransform(Eigen::Matrix4d& _m); ///< apply the local transformation to a matrix _m

        void computeRotationJac(Eigen::MatrixXd *_J, Eigen::MatrixXd *_Jdot, const Eigen::VectorXd *_qdot);   ///< compute the relative angular velocity jacobian i.e. w_rel = J*\dot{q_local}
        utils::rotation::RotationOrder getEulerOrder(); ///< Rotation order for the euler rotation if hinge, universal or ball euler joint
        Eigen::Vector3d getAxis(unsigned int _i);    ///< returns the i th axis of rotation accordingly when R = R2*R1*R0 (i \in {0,1,2})
	
        Eigen::Matrix4d getDeriv(const Dof* _q); ///< returns the derivative of the local transformation w.r.t. _q. Note: this function assumes _q belongs to this joint. Is _q is not in this joint, the local transformation is returned.
        void applyDeriv(const Dof* _q, Eigen::Vector3d& _v); ///< apply the derivative w.r.t. _q for vector _v. Note: if _q doesn't belong to this joint, applyTransform(_v) is used in effect.
        void applyDeriv(const Dof* _q, Eigen::Matrix4d& _m); ///< apply the derivative w.r.t. _q for matrix _m. Note: if _q doesn't belong to this joint, applyTransform(_m) is used in effect.
	
        Eigen::Matrix4d getSecondDeriv(const Dof* _q1, const Dof* _q2); ///< returns the second derivative of the local transformation w.r.t. _q1 and _q2
        void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Vector3d& _v); ///< apply the second derivative to _v
        void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Matrix4d& _m); ///< apply the second derivative to _m

        void addTransform(Transformation *_t, bool _isVariable = true); ///< add _t to mTransforms; _isVariable indicates whether this transformation is a degree of freedom; also sets the joint type automatically
        inline int getNumTransforms() const {return mTransforms.size();}
        inline Transformation* getTransform(int i) const {return mTransforms[i];}

        inline int getNumDofs() const {return mDofs.size();}
        inline Dof* getDof(int _idx) const {return mDofs[_idx];}

        inline int getNumDofsTrans(){return mNumDofsTrans;}
        inline int getNumDofsRot(){return mNumDofsRot;}
        inline const std::vector<int>& getRotTransformIndices(){return mRotTransformIndex;}
	
        inline BodyNode* getParentNode() const {return mNodeParent;}
        inline BodyNode* getChildNode() const {return mNodeChild;}
	
        inline void setSkelIndex(int _idx){mSkelIndex= _idx;}
        inline int getSkelIndex() const {return mSkelIndex;}

		inline JointType getJointType(){return mType;}

        inline const char *getName(){return mName;}
        inline void setName(const char *_name){
            strcpy(mName, _name);
        }
        
        void updateStaticTransform();

    protected:
        BodyNode *mNodeParent; ///< parent node
        BodyNode *mNodeChild; ///< child node
        int mSkelIndex;	///< unique dof id in model
		JointType mType;	///< type of joint e.g. ball, hinge etc., initialized as J_UNKNOWN
        char mName[512];    ///< name the joint so that joint can be retrieved by name

        std::vector<Transformation*> mTransforms;	///< transformations for mNodeChild
        std::vector<Dof*> mDofs;	///< associated dofs

        Eigen::Matrix4d mStaticTransform; ///< The product of all static transforms before the first variable one. Precalculated for efficiency.
        int mFirstVariableTransformIndex; ///< index of the first variable transform (rotation or translation)
        int mNumDofsRot;    ///< number of DOFs for corresponding to rotation
        int mNumDofsTrans;   ///< number of DOFs for corresponding to translation
        std::vector<int> mRotTransformIndex; ///< indices of the rotation transforms

        void addDof(Dof *_d); ///< add _d to mDofs 
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_JOINT_H

