/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Joint.h"

#include "Dof.h"
#include "Transformation.h"
#include "BodyNode.h"


namespace model3d {
  Joint::Joint(BodyNode *_bIn, BodyNode *_bOut){
    mModelIndex=-1;
    mNodeIn=_bIn;
    mNodeOut=_bOut;
    if(mNodeIn != NULL)
      mNodeIn->addJointOut(this);
    if(mNodeOut != NULL)
      mNodeOut->setJointIn(this);
  }

  Joint::~Joint(){
    for(int i=0; i<mTransforms.size(); i++){
      delete mTransforms[i];
    }
    mTransforms.clear();
    mDofs.clear();
  }

  bool Joint::isPresent(Dof* q){
    for(int i=0; i<mDofs.size(); i++)
      if(q==mDofs[i]) return true;
    return false;
  }

  int Joint::getIndex(int dofIndex)
  {
    for(int i=0; i<mDofs.size(); i++)
      if(mDofs[i]->getModelIndex()==dofIndex) return i;
    return -1;
  }

  Matrix4d Joint::getTransform(){
    Matrix4d m = Matrix4d::Identity();
    for(int i= mTransforms.size()-1; i>=0; i--)
      mTransforms[i]->applyTransform(m);
    return m;
  }

  void Joint::applyTransform(Vector3d& v){
    for(int i=mTransforms.size()-1; i>=0; i--){
      mTransforms[i]->applyTransform(v);
    }
  }

  void Joint::applyTransform(Matrix4d& m){
    for(int i=mTransforms.size()-1; i>=0; i--){
      mTransforms[i]->applyTransform(m);
    }
  }

  Matrix4d Joint::getDeriv(Dof* q){
    Matrix4d m = Matrix4d::Identity();
    for(int i=mTransforms.size()-1; i>=0; i--){
      if(mTransforms[i]->isPresent(q))
        mTransforms[i]->applyDeriv(q, m);
      else	
        mTransforms[i]->applyTransform(m);
    }
    return m;
  }

  void Joint::applyDeriv(Dof* q, Vector3d& v){
    for(int i=mTransforms.size()-1; i>=0; i--){
      if(mTransforms[i]->isPresent(q))
        mTransforms[i]->applyDeriv(q, v);
      else	
        mTransforms[i]->applyTransform(v);
    }
  }

  void Joint::applyDeriv(Dof* q, Matrix4d& m){
    for(int i=mTransforms.size()-1; i>=0; i--){
      if(mTransforms[i]->isPresent(q))
        mTransforms[i]->applyDeriv(q, m);
      else	
        mTransforms[i]->applyTransform(m);
    }
  }

  Matrix4d Joint::getDeriv2(Dof* q1, Dof* q2){
    Matrix4d m = Matrix4d::Identity();
    for(int i=mTransforms.size()-1; i>=0; i--){
      Transformation* transf = mTransforms[i];
      if(transf->isPresent(q1) && transf->isPresent(q2)){
        transf->applyDeriv2(q1, q2, m);
      }else if(transf->isPresent(q1)){
        transf->applyDeriv(q1, m);
      }else if(transf->isPresent(q2)){
        transf->applyDeriv(q2, m);
      }else{
        transf->applyTransform(m);
      }
    }
    return m;
  }

  void Joint::applyDeriv2(Dof* q1, Dof* q2, Vector3d& v){
    for(int i=mTransforms.size()-1; i>=0; i--){
      Transformation* transf = mTransforms[i];
      if(transf->isPresent(q1) && transf->isPresent(q2)){
        transf->applyDeriv2(q1, q2, v);
      }else if(transf->isPresent(q1)){
        transf->applyDeriv(q1, v);
      }else if(transf->isPresent(q2)){
        transf->applyDeriv(q2, v);
      }else{
        transf->applyTransform(v);
      }
    }
  }

  void Joint::applyDeriv2(Dof* q1, Dof* q2, Matrix4d& m){
    for(int i=mTransforms.size()-1; i>=0; i--){
      Transformation* transf = mTransforms[i];
      if(transf->isPresent(q1) && transf->isPresent(q2)){
        transf->applyDeriv2(q1, q2, m);
      }else if(transf->isPresent(q1)){
        transf->applyDeriv(q1, m);
      }else if(transf->isPresent(q2)){
        transf->applyDeriv(q2, m);
      }else{
        transf->applyTransform(m);
      }
    }
  }

  void Joint::addTransform(Transformation *t, bool _isVariable){
    t->setJoint(this);
    t->setVariable(_isVariable);
    mTransforms.push_back(t);
    if(_isVariable) {	// add dofs of variable transform only
      for(int i=0; i<t->getNumDofs(); i++){
        addDof(t->getDof(i));
      }
    }
  }

  void Joint::addDof(Dof *d) {
    mDofs.push_back(d);
    d->setJoint(this);
  }
  

} // namespace model3d

