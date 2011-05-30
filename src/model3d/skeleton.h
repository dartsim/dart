#ifndef SRC_MODEL3D_SKELETON_H
#define SRC_MODEL3D_SKELETON_H

#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;
#include "renderer/OpenGLRenderInterface.h"

namespace model3d {

  class Transformation;
  class Marker;
  class Joint;
  class BodyNode;
  class Dof;

  class Skeleton {
  public:
    VectorXd mCurrState; 
    BodyNode* mRoot;
    int nDofs;
    int nNodes;
    int nHandles;

    Skeleton();
    ~Skeleton();
	
    void addHandle(Marker *h);
    void addNode(BodyNode *b);
    void addJoint(Joint *_j);
    void addDof(Dof *d);
    void addTransform(Transformation *t);
	
    // init the model after parsing
    void initSkel();
    void setDependDofMap(BodyNode *b);
	
    // inline access functions
    int getNumDofs() { return mDofs.size(); }
    int getNumNodes() { return mNodes.size(); }
    int getNumHandles() { return mHandles.size(); }
    Dof* getDof(int i) { return mDofs[i]; }
    BodyNode* getNode(int i) { return mNodes[i]; }
    BodyNode* getNode(const char* const name);
    int getNodeIndex(const char* const name);
    Marker* getHandle(int i) { return mHandles[i]; }
    double getMass() { return mMass; }
    VectorXd EvalCOM();

    void setState(VectorXd&);
    void setState(vector<double>&);

    void draw(Renderer::OpenGLRenderInterface* RI, const Vector4d& _color, bool _default = true) ;
    inline void drawHandles(Renderer::OpenGLRenderInterface* RI, const Vector4d& _color, bool _default = true );
	
    inline void setPose(VectorXd& _pose);
    inline void setPose(vector<double>& _pose);
    inline void getPose(VectorXd& _pose);
    inline void getPose(vector<double>& _pose);

  protected:
    vector<Marker*> mHandles;
    vector<Dof*> mDofs;
    vector<Transformation*> mTransforms;
    vector<BodyNode*> mNodes;
    vector<Joint*> mJoints;

    double mMass;
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_SKELETON_H

