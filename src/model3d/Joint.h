/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#ifndef MODEL3D_JOINT_H
#define MODEL3D_JOINT_H

#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

namespace model3d {

  class Dof;
  class BodyNode;
  class Transformation;

  class Joint {
  public:
    enum Type {
      FREE,
      BALLEULER,
      BALLQUAT,
      BALLEXPMAP,
      HINGE,
      UNIVERSAL
    };
	
    Joint(BodyNode* _bIn, BodyNode* _bOut);
    ~Joint();

    bool isPresent(Dof *d);	// true if d is present in the dof list
    int getIndex(int dofIndex); // get local index of the dof at this joint

    Matrix4d getTransform();
    void applyTransform(Vector3d& v);
    void applyTransform(Matrix4d& m);
	
    Matrix4d getDeriv(Dof* q);
    void applyDeriv(Dof* q, Vector3d& v);
    void applyDeriv(Dof* q, Matrix4d& m);
	
    Matrix4d getDeriv2(Dof* q1, Dof* q2);
    void applyDeriv2(Dof*, Dof*, Vector3d&);
    void applyDeriv2(Dof*, Dof*, Matrix4d&);

    void addTransform(Transformation *t, bool _isVariable = true);
    int getNumTransforms(){return mTransforms.size();}
    Transformation* getTransform(int i){return mTransforms[i];}

    int getNumDofs(){return mDofs.size();}
    Dof* getDof(int i){return mDofs[i];}
	
    BodyNode* getNodeIn(){return mNodeIn;}
    BodyNode* getNodeOut(){return mNodeOut;}
	
    void setModelIndex(int _idx){mModelIndex= _idx;}
    int getModelIndex(){return mModelIndex;}

  protected:
    BodyNode *mNodeIn;
    BodyNode *mNodeOut;
    int mModelIndex;	// unique to dof in model

    vector<Transformation*> mTransforms;	//transformations for mNodeOut
    vector<Dof*> mDofs;	// associated dofs
    void addDof(Dof *d);
  };

} // namespace model3d

#endif // #ifndef MODEL3D_JOINT_H

