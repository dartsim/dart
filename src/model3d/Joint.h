/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef MODEL3D_JOINT_H
#define MODEL3D_JOINT_H

#include <vector>
#include <Eigen/Dense>
#include "utils/RotationConversion.h"

namespace model3d {

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
        int getLocalIndex (int _dofSkelIndex) const; ///< get local index of the dof at this joint; if the dof is not presented at this joint, return -1

        Eigen::Matrix4d getLocalTransform(); ///< computes and returns the local transformation from NodeIn to NodeOut
        void applyTransform(Eigen::Vector3d& _v); ///< apply the local transformation to a vector _v
        void applyTransform(Eigen::Matrix4d& _m); ///< apply the local transformation to a matrix _m

        void computeRotationJac(Eigen::MatrixXd *_J, Eigen::MatrixXd *_Jdot, Eigen::VectorXd *_qdot);   ///< compute the relative angular velocity jacobian i.e. w_rel = J*\dot{q_local}
        utils::rot_conv::RotationOrder getEulerOrder(); ///< Rotation order for the euler rotation if hinge, universal or ball euler joint
        Eigen::Vector3d getAxis(unsigned int _i);    ///< returns the i th axis of rotation accordingly when R = R2*R1*R0 (i \in {0,1,2})
	
        Eigen::Matrix4d getDeriv(const Dof* _q); ///< returns the derivative of the local transformation w.r.t. _q. Note: this function assumes _q belongs to this joint. Is _q is not in this joint, the local transformation is returned.
        void applyDeriv(const Dof* _q, Eigen::Vector3d& _v); ///< apply the derivative w.r.t. _q for vector _v. Note: if _q doesn't belong to this joint, applyTransform(_v) is used in effect.
        void applyDeriv(const Dof* _q, Eigen::Matrix4d& _m); ///< apply the derivative w.r.t. _q for matrix _m. Note: if _q doesn't belong to this joint, applyTransform(_m) is used in effect.
	
        Eigen::Matrix4d getSecondDeriv(const Dof* _q1, const Dof* _q2); ///< returns the second derivative of the local transformation w.r.t. _q1 and _q2
        void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Vector3d& _v); ///< apply the second derivative to _v
        void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Matrix4d& _m); ///< apply the second derivative to _m

        void addTransform(Transformation *_t, bool _isVariable = true); ///< add _t to mTransforms; _isVariable indicates whether this transformation is a degree of freedom; also sets the joint type automatically
        int getNumTransforms() const {return mTransforms.size();}
        Transformation* getTransform(int i) const {return mTransforms[i];}

        int getNumDofs() const {return mDofs.size();}
        Dof* getDof(int _idx) const {return mDofs[_idx];}
	
        BodyNode* getParentNode() const {return mNodeParent;}
        BodyNode* getChildNode() const {return mNodeChild;}
	
        void setSkelIndex(int _idx){mSkelIndex= _idx;}
        int getSkelIndex() const {return mSkelIndex;}

		JointType getJointType(){return mType;}

        inline const char *getName(){return mName;}
        inline void setName(const char *_name){
            strcpy(mName, _name);
        }

    protected:
        BodyNode *mNodeParent; ///< parent node
        BodyNode *mNodeChild; ///< child node
        int mSkelIndex;	///< unique dof id in model
		JointType mType;	///< type of joint e.g. ball, hinge etc., initialized as J_UNKNOWN
        char mName[512];    ///< name the joint so that joint can be retrieved by name

        std::vector<Transformation*> mTransforms;	///< transformations for mNodeChild
        std::vector<Dof*> mDofs;	///< associated dofs
        
        int mNumDofsRot;    ///< number of DOFs for corresponding to rotation
        int mNumDofsTrans;   ///< number of DOFs for corresponding to translation
        std::vector<int> mRotTransIndex; ///< indices of the rotation dofs 

        void addDof(Dof *_d); ///< add _d to mDofs 
    };

} // namespace model3d

#endif // #ifndef MODEL3D_JOINT_H

