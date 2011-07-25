/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_SKELETON_H
#define MODEL3D_SKELETON_H

#include <vector>
#include <Eigen/Dense>
#include "renderer/RenderInterface.h"

namespace model3d {

    class Transformation;
    class Marker;
    class Joint;
    class BodyNode;
    class Dof;

    class Skeleton {
    public:
        Eigen::VectorXd mCurrState; 
        BodyNode* mRoot;
        int mNumDofs;
        int mNumNodes;
        int mNumHandles;

        Skeleton();
        virtual ~Skeleton();

        virtual BodyNode* createBodyNode(const char* const name = NULL);
        void addHandle(Marker *h);
        void addNode(BodyNode *b);
        void addJoint(Joint *_j);
        void addDof(Dof *d);
        void addTransform(Transformation *t);
	
        // init the model after parsing
        void initSkel();
	
        // inline access functions
        inline int getNumDofs() { return mDofs.size(); }
        inline int getNumNodes() { return mNodes.size(); }
        inline int getNumHandles() { return mHandles.size(); }
        inline Dof* getDof(int i) { return mDofs[i]; }
        inline BodyNode* getNode(int i) { return mNodes[i]; }
        inline BodyNode* getRoot() { return mRoot; }
        BodyNode* getNode(const char* const name);
        int getNodeIndex(const char* const name);
        inline Marker* getHandle(int i) { return mHandles[i]; }
        inline double getMass() { return mMass; }
        inline int getNumJoints(){return mJoints.size();}

        void setState(const Eigen::VectorXd&, bool bCalcTrans = true, bool bCalcDeriv = true);
        void setState(const std::vector<double>&, bool bCalcTrans = true, bool bCalcDeriv = true);

        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const;
        void drawHandles(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true ) const;
	
        void setPose(const Eigen::VectorXd& _pose);
        void setPose(const std::vector<double>& _pose);
        void getPose(Eigen::VectorXd& _pose);
        void getPose(std::vector<double>& _pose);

    protected:
        std::vector<Marker*> mHandles;
        std::vector<Dof*> mDofs;
        std::vector<Transformation*> mTransforms;
        std::vector<BodyNode*> mNodes;
        std::vector<Joint*> mJoints;

        double mMass;
    };

} // namespace model3d

#endif // #ifndef MODEL3D_SKELETON_H

