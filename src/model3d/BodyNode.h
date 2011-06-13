/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author		Sehoon Ha
  Date		06/07/2011
*/

#ifndef MODEL3D_BODYNODE_H
#define MODEL3D_BODYNODE_H

#include <vector>
#include <Eigen/Dense>

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
        BodyNode(char *_name = NULL); ///< Default constructor. The name can be up to 128
        virtual ~BodyNode(); ///< Default destructor

        // functions from Node
        void init(); ///< Initialize the properties and the derivatives
        void update(Eigen::VectorXd&); ///< update all values w.r.t. the given joint angles.
        void evalSecondOrder(const Eigen::VectorXd&);
        Eigen::MatrixXd evalM(Eigen::VectorXd&);
        Eigen::VectorXd evalC(Eigen::VectorXd&);
        Eigen::VectorXd evalWorldPos(Eigen::VectorXd& lp); 
        Eigen::MatrixXd evalDpDq(Eigen::VectorXd& lp);	
        void evalSecDpDq(Eigen::VectorXd&, std::vector<Eigen::MatrixXd>&);
        Eigen::VectorXd evalMomenta(Eigen::VectorXd&,Eigen::VectorXd&);
        Eigen::MatrixXd evalP(Eigen::VectorXd&);
        Eigen::MatrixXd evaldLdq(Eigen::VectorXd&, Eigen::VectorXd&);
        double evalG();
        Eigen::VectorXd evaldGdq();

        Eigen::VectorXd evalCOM() { return mCOM; }
        void draw();
        double getMass() const;

        // Jie's helper functions
        void evalJC();
        void evalJW();
        void evalJCq(int dofIndex);
        void evalJWq(int dofIndex);

        // ori functions
        // Transformation
        Eigen::MatrixXd getWorldTransform() { return W; }
  
        // Inverse Transformation
        Eigen::MatrixXd getWorldInvTransform() { return W.inverse(); }
        Eigen::MatrixXd getLocalInvTransform() { return T.inverse(); }
	
        void draw(Renderer::OpenGLRenderInterface *RI, const Eigen::Vector4d& _color,
                  bool _default, int depth = 0);	// render the entire bodylink subtree rooted here
        void drawHandles(Renderer::OpenGLRenderInterface *RI, const Eigen::Vector4d& _color,
                         bool _default);	// render the handles

        char* getName() { return mName; }
        Eigen::Vector3d getOffset() { return Eigen::Vector3d(mOffset[0],mOffset[1],mOffset[2]); }
        void setOffset(const Eigen::Vector3d& _off) { mOffset = _off; }
        int getModelIndex() { return mModelIndex; }
        void setModelIndex(int _idx) { mModelIndex = _idx; }
        BodyNode* getNodeIn() { return mNodeIn; }
        void setSkel(Skeleton* skel) { mSkel = skel; }

        void addHandle(Marker *h) { mHandles.push_back(h); }
        std::vector<Marker*> clearHandles();
        void removeHandle(Marker *h);
        int getNumHandles() const { return mHandles.size(); }
        Marker* getHandle(int i) const { return mHandles[i]; }
	
        Primitive* getPrimitive() const { return mPrimitive; }
        void setPrimitive(Primitive *_p) { mPrimitive = _p; }

        void addJointOut(Joint *_c) { mJointOut.push_back(_c); }
        int getNumJoints() { return mJointOut.size(); }
        Joint* getJointOut(int i) { return mJointOut[i]; }
        Joint* getJointIn() { return mJointIn; }
        void setJointIn(Joint *_p);

        // wrapper functions for joints
        BodyNode* getNodeOut(int i) const;
        int getNumDofs() const;
        Dof* getDof(int i);
        bool isPresent(Dof *q);
        Eigen::Matrix4d getLocalTrans() const;
        Eigen::Matrix4d getLocalDeriv(Dof *q) const;
        Eigen::Matrix4d getLocalDeriv2(Dof *q1, Dof *q2) const;

    public:
        char mName[MAX_NODE3D_NAME];
        int mModelIndex;	// location in the model

        Primitive *mPrimitive;	// body geometry
        std::vector<Joint *> mJointOut;	// list of joints that link to children
        Joint *mJointIn;	// joint to connect to parent
        BodyNode *mNodeIn;		// parent node
        std::vector<Marker *> mHandles;	// list of handles associated
        Skeleton *mSkel;

        // transformation
        Eigen::MatrixXd T; // local transformation from parent to itself
        std::vector<Eigen::MatrixXd> Tq;
        std::vector<std::vector<Eigen::MatrixXd> > Tqq;
        Eigen::MatrixXd W; // global transformation
        std::vector<Eigen::MatrixXd> Wq;
        std::vector<std::vector<Eigen::MatrixXd> > Wqq;

        // Jacobian
        Eigen::MatrixXd mJC;
        Eigen::MatrixXd mJW;
        std::vector<Eigen::MatrixXd> mJCq;
        std::vector<Eigen::MatrixXd> mJWq;
	
        Eigen::MatrixXd mK1;
        Eigen::MatrixXd mK2Qd;
        Eigen::MatrixXd mJWqQd;
        Eigen::MatrixXd mJCqQd;

        Eigen::VectorXd mCOM;
        bool *dependsOnDof;	// map to answer the question whether the bodylink depends on the asked dof or not.

        double mMass;
        Eigen::VectorXd mOffset;
        Skeleton *mModel;

    private:
        int mID;
        static int msBodyNodeCount;
    };

} // namespace model3d

#endif // #ifndef MODEL3D_BODYNODE_H

