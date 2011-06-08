/*
RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
All rights reserved.

Author		Sehoon Ha
Date		06/07/2011
*/

#ifndef SRC_MODEL3D_BODYNODE_H
#define SRC_MODEL3D_BODYNODE_H

#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

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
    void update(VectorXd&); ///< update all values w.r.t. the given joint angles.
    void evalSecondOrder(const VectorXd&);
    MatrixXd evalM(VectorXd&);
    VectorXd evalC(VectorXd&);
    VectorXd evalWorldPos(VectorXd& lp); 
    MatrixXd evalDpDq(VectorXd& lp);	
    void evalSecDpDq(VectorXd&, vector<MatrixXd>&);
    VectorXd evalMomenta(VectorXd&,VectorXd&);
    MatrixXd evalP(VectorXd&);
    MatrixXd evaldLdq(VectorXd&, VectorXd&);
    double evalG();
    VectorXd evaldGdq();

    VectorXd evalCOM() { return mCOM; }
    void draw();
    double getMass() const;

    // Jie's helper functions
    void evalJC();
    void evalJW();
    void evalJCq(int dofIndex);
    void evalJWq(int dofIndex);

    // ori functions
    // Transformation
    MatrixXd getWorldTransform() { return W; }
  
    // Inverse Transformation
    MatrixXd getWorldInvTransform() { return W.inverse(); }
    MatrixXd getLocalInvTransform() { return T.inverse(); }
	
    void draw(Renderer::OpenGLRenderInterface* RI, const Vector4d& _color,
              bool _default, int depth = 0);	// render the entire bodylink subtree rooted here
    void drawHandles(Renderer::OpenGLRenderInterface* RI, const Vector4d& _color,
                     bool _default);	// render the handles

    char* getName() { return mName; }
    Vector3d getOffset() { return Vector3d(mOffset[0],mOffset[1],mOffset[2]); }
    void setOffset(const Vector3d& _off) { mOffset = _off; }
    int getModelIndex() { return mModelIndex; }
    void setModelIndex(int _idx) { mModelIndex = _idx; }
    BodyNode* getNodeIn() { return mNodeIn; }
    void setSkel(Skeleton* skel) { mSkel = skel; }

    void addHandle(Marker *h) { mHandles.push_back(h); }
    vector<Marker*> clearHandles();
    void removeHandle(Marker *h);
    int getNumHandles() const { return mHandles.size(); }
    Marker* getHandle(int i) const { return mHandles[i]; }
	
    Primitive* getPrimitive() const { return mPrimitive; }
    void setPrimitive(Primitive *_p) { mPrimitive = _p; }

    void addJointOut(Joint* _c) { mJointOut.push_back(_c); }
    int getNumJoints() { return mJointOut.size(); }
    Joint* getJointOut(int i) { return mJointOut[i]; }
    Joint* getJointIn() { return mJointIn; }
    void setJointIn(Joint *_p);

    // wrapper functions for joints
    BodyNode* getNodeOut(int i) const;
    int getNumDofs() const;
    Dof* getDof(int i);
    bool isPresent(Dof* q);
    Matrix4d getLocalTrans() const;
    Matrix4d getLocalDeriv(Dof* q) const;
    Matrix4d getLocalDeriv2(Dof* q1, Dof* q2) const;

  public:
    char mName[MAX_NODE3D_NAME];
    int mModelIndex;	// location in the model

    Primitive *mPrimitive;	// body geometry
    vector<Joint *> mJointOut;	// list of joints that link to children
    Joint *mJointIn;	// joint to connect to parent
    BodyNode* mNodeIn;		// parent node
    vector<Marker *> mHandles;	// list of handles associated
    Skeleton* mSkel;

    // transformation
    MatrixXd T; // local transformation from parent to itself
    vector<MatrixXd> Tq;
    vector<vector<MatrixXd> > Tqq;
    MatrixXd W; // global transformation
    vector<MatrixXd> Wq;
    vector<vector<MatrixXd> > Wqq;

    // Jacobian
    MatrixXd mJC;
    MatrixXd mJW;
    vector<MatrixXd> mJCq;
    vector<MatrixXd> mJWq;
	
    MatrixXd mK1;
    MatrixXd mK2Qd;
    MatrixXd mJWqQd;
    MatrixXd mJCqQd;

    VectorXd mCOM;
    bool *dependsOnDof;	// map to answer the question whether the bodylink depends on the asked dof or not.

    double mMass;
    VectorXd mOffset;
    Skeleton* mModel;

 private:
    int mID;
    static int msBodyNodeCount;
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_BODYNODE_H

