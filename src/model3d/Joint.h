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

namespace model3d {

    class Dof;
    class BodyNode;
    class Transformation;

    class Joint {
    public:
        enum JointType {
			UNKNOWN,
            FREE,
            BALLEULER,
            BALLQUAT,
            BALLEXPMAP,
            HINGE,
            UNIVERSAL
        };
	
        Joint(BodyNode* _bIn, BodyNode* _bOut);
        virtual ~Joint();

        bool isPresent (const Dof *_d) const;	///< true if d is present in the dof list
        int getLocalIndex (int _dofModelIndex) const; ///< get local index of the dof at this joint; if the dof is not presented at this joint, return -1

        Eigen::Matrix4d getLocalTransform(); ///< computes and returns the local transformation from NodeIn to NodeOut
        void applyTransform(Eigen::Vector3d& _v); ///< apply the local transformation to a vector _v
        void applyTransform(Eigen::Matrix4d& _m); ///< apply the local transformation to a matrix _m
	
        Eigen::Matrix4d getDeriv(const Dof* _q); ///< returns the derivative of the local transformation w.r.t. _q. Note: this function assumes _q belongs to this joint. Is _q is not in this joint, the local transformation is returned.
        void applyDeriv(const Dof* _q, Eigen::Vector3d& _v); ///< apply the derivative w.r.t. _q for vector _v. Note: if _q doesn't belong to this joint, applyTransform(_v) is used in effect.
        void applyDeriv(const Dof* _q, Eigen::Matrix4d& _m); ///< apply the derivative w.r.t. _q for matrix _m. Note: if _q doesn't belong to this joint, applyTransform(_m) is used in effect.
	
        Eigen::Matrix4d getSecondDeriv(const Dof* _q1, const Dof* _q2); ///< returns the second derivative of the local transformation w.r.t. _q1 and _q2
        void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Vector3d& _v); ///< apply the second derivative to _v
        void applySecondDeriv(const Dof* _q1, const Dof* _q2, Eigen::Matrix4d& _m); ///< apply the second derivative to _m

        void addTransform(Transformation *_t, bool _isVariable = true); ///< add _t to mTransforms; _isVariable indicates whether this transformation is a degree of freedom
        int getNumTransforms() const {return mTransforms.size();}
        Transformation* getTransform(int i) const {return mTransforms[i];}

        int getNumDofs() const {return mDofs.size();}
        Dof* getDof(int _idx) const {return mDofs[_idx];}
	
        BodyNode* getNodeIn() const {return mNodeIn;}
        BodyNode* getNodeOut() const {return mNodeOut;}
	
        void setModelIndex(int _idx){mModelIndex= _idx;}
        int getModelIndex() const {return mModelIndex;}

		JointType getJointType(){return mType;}

    protected:
        BodyNode *mNodeIn; ///< parent node
        BodyNode *mNodeOut; ///< child node
        int mModelIndex;	///< unique dof id in model
		JointType mType;	///< type of joint e.g. ball, hinge etc., initialized as UNKNOWN

        std::vector<Transformation*> mTransforms;	///< transformations for mNodeOut
        std::vector<Dof*> mDofs;	///< associated dofs
        
        void addDof(Dof *_d); ///< add _d to mDofs 
    };

} // namespace model3d

#endif // #ifndef MODEL3D_JOINT_H

