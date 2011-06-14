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

namespace Renderer {
    class OpenGLRenderInterface;
} // namespace Renderer

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
        void updateTransform(); ///< update transformations w.r.t. the current dof values in Dof*

        Eigen::Matrix4d getWorldTransform() const { return W; } ///< transformation from the local coordinates of this body node to the world coordinates
        Eigen::Matrix4d getWorldInvTransform() const { return W.inverse(); } ///< transformation from the world coordinates to the local coordiantes of this body node
        Eigen::Matrix4d getLocalTransform() const { return T; } ///< transformation from the local coordiantes of this node to the local coordiantes of its parent
        Eigen::Matrix4d getLocalInvTransform() const { return T.inverse(); } ///< transformation from the local coordinates of the parent node to the local coordiantes of this node

        Eigen::Vector3d evalWorldPos(Eigen::Vector3d& lp); ///< given a 3D vector lp in the local coordinates of this node, return the world coordinates of this vector

        void setDependDofMap(int _numDofs); ///< set up the dof dependence map for this node
        bool dependsOn(int _dofIndex) const { return dependsOnDof[_dofIndex]; } ///< NOTE: not checking index range

        void draw(Renderer::OpenGLRenderInterface *RI, const Eigen::Vector4d& _color, bool _default, int depth = 0);    ///< render the entire bodylink subtree rooted here
        void drawHandles(Renderer::OpenGLRenderInterface *RI, const Eigen::Vector4d& _color, bool _default);    ///< render the handles

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
        Marker* getHandle(int i) const { return mHandles[i]; }

        void setPrimitive(Primitive *_p) { mPrimitive = _p; }
        Primitive* getPrimitive() const { return mPrimitive; }
        
        void addJointOut(Joint *_c) { mJointOut.push_back(_c); }
        int getNumJoints() { return mJointOut.size(); }
        Joint* getJointOut(int i) { return mJointOut[i]; }
        void setJointIn(Joint *_p);
        Joint* getJointIn() { return mJointIn; }

        // wrapper functions for joints
        BodyNode* getNodeOut(int i) const;
        int getNumDofs() const;
        Dof* getDof(int i);
        bool isPresent(Dof *q);

    protected:
        char mName[MAX_NODE3D_NAME]; ///< name
        int mModelIndex;    ///< location in the model

        Primitive *mPrimitive;  ///< body geometry
        std::vector<Joint *> mJointOut; ///< list of joints that link to children
        Joint *mJointIn;    ///< joint to connect to parent
        BodyNode *mNodeIn;      ///< parent node
        std::vector<Marker *> mHandles; ///< list of handles associated

        // transformations
        Eigen::Matrix4d T; ///< local transformation from parent to itself
        Eigen::Matrix4d W; ///< global transformation

        bool *dependsOnDof; ///< map to answer the question whether the bodylink depends on the asked dof or not.

        double mMass; ///< mass of this node; if it has no primitive associated with, its mass is zero
        Eigen::Vector3d mOffset; ///< origin of this node in its parent's coordinate frame
        Skeleton *mModel; ///< the model this node belongs to

    private:
        int mID; ///< a unique ID of this node globally 
        static int msBodyNodeCount; ///< counts the numbe of nodes globally
    };

} // namespace model3d

#endif // #ifndef MODEL3D_BODYNODE_H

