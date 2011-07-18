/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author        Sehoon Ha
  Date      06/07/2011
*/

#ifndef MODEL3D_BODYNODE_H
#define MODEL3D_BODYNODE_H

#include <vector>
#include <Eigen/Dense>
#include "utils/EigenArrayHelper.h"
#include "renderer/RenderInterface.h"

namespace model3d {
#define MAX_NODE3D_NAME 128

    class Marker;
    class Dof;
    class Transformation;
    class Primitive;
    class Skeleton;
    class Joint;

    /**
       @brief BodyNode class represents a single node of the skeleton.

       BodyNode is a basic element of the skeleton. BodyNodes are hierarchically
       connected and have a set of core functions for calculating derivatives.
       Mostly automatically constructed by FileInfoModel. @see FileInfoModel.
    */
    class BodyNode {
    public:      
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // we need this aligned allocator because we have Matrix4d as memebers in this class
        
        BodyNode(char *_name = NULL); ///< Default constructor. The name can be up to 128
        virtual ~BodyNode(); ///< Default destructor

        void init(); ///< Initialize the vector memebers with proper sizes
        void updateTransform(); ///< Update transformations w.r.t. the current dof values in Dof*
        void updateDerivatives();

        Eigen::Matrix4d getWorldTransform() const { return mTransWorld; } ///< Transformation from the local coordinates of this body node to the world coordinates
        Eigen::Matrix4d getWorldInvTransform() const { return mTransWorld.inverse(); } ///< Transformation from the world coordinates to the local coordiantes of this body node
        Eigen::Matrix4d getLocalTransform() const { return mTransLocal; } ///< Transformation from the local coordiantes of this body node to the local coordiantes of its parent
        Eigen::Matrix4d getLocalInvTransform() const { return mTransLocal.inverse(); } ///< Transformation from the local coordinates of the parent node to the local coordiantes of this body node

        Eigen::Vector3d evalWorldPos(const Eigen::Vector3d& _lp); ///< Given a 3D vector lp in the local coordinates of this body node, return the world coordinates of this vector

        /* void setDependDofMap(int _numDofs); ///< set up the dof dependence map for this node */
        /* bool dependsOn(int _dofIndex) const { return mDependsOnDof[_dofIndex]; } ///< NOTE: not checking index range */
        void setDependDofList(); ///< Set up the list of dependent dofs 
        bool dependsOn(int _dofIndex) const; ///< Test whether this dof is depedant or not \warning{You may want to use getNumDependantDofs / getDependantDof for efficiency}
        int getNumDependantDofs() const { return mDependantDofs.size(); } ///< The number of the dofs which this node is affected
        int getDependantDof(int _arrayIndex) { return mDependantDofs[_arrayIndex]; } ///< Return an dof index from the array index (< getNumDependantDofs)

        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(), bool _useDefaultColor = true, int _depth = 0) const ;    ///< Render the entire subtree rooted at this body node
        void drawHandles(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const ;    ///< Render the handles

        char* getName() { return mName; }

        void setOffset(const Eigen::Vector3d& _off) { mOffset = _off; }
        Eigen::Vector3d getOffset() const { return mOffset; }
        
        void setModel(Skeleton* _model) { mModel = _model; }
        Skeleton* getModel() const { return mModel; }

        void setModelIndex(int _idx) { mModelIndex = _idx; }
        int getModelIndex() const { return mModelIndex; }
        
        BodyNode* getNodeIn() const { return mNodeIn; }
        double getMass() const { return mMass; }

        void addHandle(Marker *_h) { mHandles.push_back(_h); }
        int getNumHandles() const { return mHandles.size(); }
        Marker* getHandle(int _idx) const { return mHandles[_idx]; }

        void setPrimitive(Primitive *_p) { mPrimitive = _p; }
        Primitive* getPrimitive() const { return mPrimitive; }
        
        void addJointOut(Joint *_c) { mJointOut.push_back(_c); }
        int getNumJoints() { return mJointOut.size(); }
        Joint* getJointOut(int _idx) const { return mJointOut[_idx]; }
        void setJointIn(Joint *_p);
        Joint* getJointIn() const { return mJointIn; }

        // wrapper functions for joints
        BodyNode* getNodeOut(int _idx) const;
        int getNumDofs() const;
        Dof* getDof(int _idx) const;
        bool isPresent(Dof *_q);

        Eigen::Matrix4d getLocalDeriv(Dof *_q) const;

    protected:
        void evalJC(); ///< Evaluate linear Jacobian of this body node wrt dependent dofs
        void evalJW(); ///< Evaluate angular Jacobian of this body node wrt dependent dofs
        
        char mName[MAX_NODE3D_NAME]; ///< Name
        int mModelIndex;    ///< Index in the model

        Primitive *mPrimitive;  ///< Geometry of this body node
        std::vector<Joint *> mJointOut; ///< List of joints that link to child nodes
        Joint *mJointIn;    ///< Joint connecting to parent node
        BodyNode *mNodeIn;      ///< Parent node
        std::vector<Marker *> mHandles; ///< List of handles associated

        // transformations
        Eigen::Matrix4d mTransLocal; ///< Local transformation from parent to itself
        Eigen::Matrix4d mTransWorld; ///< Global transformation

        std::vector<int> mDependantDofs; ///< A list of dependant dof indices 

        double mMass; ///< Mass of this node; zero if no primitive
        Eigen::Vector3d mOffset; ///< Origin of this body node in its parent's coordinate frame
        Skeleton *mModel; ///< Pointer to the model this body node belongs to

    private:
        int mID; ///< A unique ID of this node globally 
        static int msBodyNodeCount; ///< Counts the number of nodes globally
    public:
        std::vector<Eigen::MatrixXd> mTq; ///< Partial derivative of local transformation wrt local dofs; each element is a 4x4 matrix
        std::vector<Eigen::MatrixXd> mWq; ///< Partial derivative of world transformation wrt all dependent dofs; each element is a 4x4 matrix
        Eigen::MatrixXd mJC; ///< Linear Jacobian; Cartesian_linear_velocity = mJC * generalized_velocity
        Eigen::MatrixXd mJW; ///< Angular Jacobian; Cartesian_angular_velocity = mJW * generalized_velocity
    };

} // namespace model3d

#endif // #ifndef MODEL3D_BODYNODE_H

