#ifndef SRC_MODEL3D_SKELETON_H
#define SRC_MODEL3D_SKELETON_H

#include <vector>
using namespace std;

namespace model3d {
  class Transformation;
  class Marker;
  class Joint;
  class BodyNode;
  class Dof;

  class Skeleton {
  public:
    Vecd mCurrState; 
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
    inline int getNumDofs();
    inline int getNumNodes();
    inline int getNumHandles();
    inline Dof* getDof(int i);
    inline BodyNode* getNode(int i);
    BodyNode* getNode(const char* const name);
    int getNodeIndex(const char* const name);
    inline Marker* getHandle(int i);
    inline double getMass();
    Vecd EvalCOM();

    void setState(Vecd&);
    void setState(vector<double>&);

    inline void draw(Vector4d _color, bool _default = true) ;
    inline void drawHandles(Vector4d _color, bool _default = true );
	
    inline void setPose(Vecd& _pose);
    inline void setPose(vector<double>& _pose);
    inline void getPose(Vecd& _pose);
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

