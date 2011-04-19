#ifndef SRC_MODEL3D_BODYNODE_H
#define SRC_MODEL3D_BODYNODE_H

#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

#include "bodynode.h"

namespace model3d {
#define MAX_NODE3D_NAME 128

  class Marker;
  class Dof;
  class Transformation;
  class Primitive;
  class Skeleton;
  class Joint;

  class BodyNode {
  public:
    BodyNode(char *_name = NULL);
    ~BodyNode();

    // functions from Node
    void evalR(VectorXd&);
    void init();
    void update(VectorXd&);
    void evalSecondOrder(VectorXd&);
    MatrixXd evalM(VectorXd&);
    VectorXd evalC(VectorXd&);
    VectorXd evalWorldPos(VectorXd h);	// recursive function
    MatrixXd evalDpDq(VectorXd lp);	
    void evalSecDpDq(VectorXd, vector<MatrixXd>&);
    VectorXd evalMomenta(VectorXd,VectorXd);
    MatrixXd evalP(VectorXd);
    MatrixXd evaldLdq(VectorXd, VectorXd);
    double evalG();
    VectorXd evaldGdq();

    inline VectorXd evalCOM();
    void draw();
    inline double getMass();

    // Jie's helper functions
    void evalJC();
    void evalJW();
    void evalJCq(int dofIndex);
    void evalJWq(int dofIndex);

    // ori functions
    // Transformation
    MatrixXd getWorldTransform();
  
    // Inverse Transformation
    MatrixXd getWorldInvTransform();
    MatrixXd getLocalInvTransform();
	
    void draw(Vector4d _color, bool _default, int depth = 0);	// render the entire bodylink subtree rooted here
    void drawHandles(Vector4d _color, bool _default);	// render the handles

    inline char* getName();
    inline Vector3d getOffset();
    inline void setOffset(Vector3d _off);
    inline int getModelIndex();
    inline void setModelIndex(int _idx);
    inline BodyNode* getNodeIn();
    inline void setSkel(Skeleton* skel);

    inline void addHandle(Marker *h);
    inline vector<Marker*> clearHandles();
    inline void removeHandle(Marker *h);
    inline int getNumHandles();
    inline Marker* getHandle(int i);
	
    inline Primitive* getPrimitive();
    inline void setPrimitive(Primitive *_p);

    inline void addJointOut(Joint *_c);
    inline int getNumJoints();
    inline Joint* getJointOut(int i);
    inline Joint* getJointIn();
    inline void setJointIn(Joint *_p);

    // wrapper functions for joints
    inline BodyNode* getNodeOut(int i);
    inline int getNumDofs();
    inline Dof* getDof(int i);
    inline bool isPresent(Dof* q);
    inline Matrix4d getLocalTrans();
    inline Matrix4d getLocalDeriv(Dof* q);
    inline Matrix4d getLocalDeriv2(Dof* q1, Dof* q2);

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

  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_BODYNODE_H

