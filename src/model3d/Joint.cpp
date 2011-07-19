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
using namespace Eigen;


namespace model3d {
    Joint::Joint(BodyNode *_bIn, BodyNode *_bOut){
		mType = UNKNOWN;
        mSkelIndex=-1;
        mNodeIn=_bIn;
        mNodeOut=_bOut;
        if(mNodeIn != NULL)
            mNodeIn->addJointOut(this);
        if(mNodeOut != NULL)
            mNodeOut->setJointIn(this);
    }

    Joint::~Joint(){
        for(unsigned int i=0; i<mTransforms.size(); i++){
            delete mTransforms[i];
        }
        mTransforms.clear();
        mDofs.clear();
    }

    bool Joint::isPresent (const Dof* _q) const{
        for(unsigned int i=0; i<mDofs.size(); i++)
            if(_q==mDofs[i]) return true;
        return false;
    }

    int Joint::getLocalIndex (int _dofSkelIndex) const{
        for(unsigned int i=0; i<mDofs.size(); i++)
            if(mDofs[i]->getSkelIndex()==_dofSkelIndex) return i;
        return -1;
    }

    Matrix4d Joint::getLocalTransform(){
        Matrix4d m = Matrix4d::Identity();
        for(int i= mTransforms.size()-1; i>=0; i--) {
            mTransforms[i]->applyTransform(m);
        }
        return m;
    }

    void Joint::applyTransform(Vector3d& _v){
        for(int i=mTransforms.size()-1; i>=0; i--){
            mTransforms[i]->applyTransform(_v);
        }
    }

    void Joint::applyTransform(Matrix4d& _m){
        for(int i=mTransforms.size()-1; i>=0; i--){
            mTransforms[i]->applyTransform(_m);
        }
    }

    Matrix4d Joint::getDeriv(const Dof* _q){
        Matrix4d m = Matrix4d::Identity();
        for(int i=mTransforms.size()-1; i>=0; i--){
            if(mTransforms[i]->isPresent(_q))
                mTransforms[i]->applyDeriv(_q, m);
            else	
                mTransforms[i]->applyTransform(m);
        }
        return m;
    }

    void Joint::applyDeriv(const Dof* _q, Vector3d& _v){
        for(int i=mTransforms.size()-1; i>=0; i--){
            if(mTransforms[i]->isPresent(_q))
                mTransforms[i]->applyDeriv(_q, _v);
            else	
                mTransforms[i]->applyTransform(_v);
        }
    }

    void Joint::applyDeriv(const Dof* _q, Matrix4d& _m){
        for(int i=mTransforms.size()-1; i>=0; i--){
            if(mTransforms[i]->isPresent(_q))
                mTransforms[i]->applyDeriv(_q, _m);
            else	
                mTransforms[i]->applyTransform(_m);
        }
    }

    Matrix4d Joint::getSecondDeriv(const Dof* _q1, const Dof* _q2){
        Matrix4d m = Matrix4d::Identity();
        for(int i=mTransforms.size()-1; i>=0; i--){
            Transformation* transf = mTransforms[i];
            if(transf->isPresent(_q1) && transf->isPresent(_q2)){
                transf->applySecondDeriv(_q1, _q2, m);
            }else if(transf->isPresent(_q1)){
                transf->applyDeriv(_q1, m);
            }else if(transf->isPresent(_q2)){
                transf->applyDeriv(_q2, m);
            }else{
                transf->applyTransform(m);
            }
        }
        return m;
    }

    void Joint::applySecondDeriv(const Dof* _q1, const Dof* _q2, Vector3d& _v){
        for(int i=mTransforms.size()-1; i>=0; i--){
            Transformation* transf = mTransforms[i];
            if(transf->isPresent(_q1) && transf->isPresent(_q2)){
                transf->applySecondDeriv(_q1, _q2, _v);
            }else if(transf->isPresent(_q1)){
                transf->applyDeriv(_q1, _v);
            }else if(transf->isPresent(_q2)){
                transf->applyDeriv(_q2, _v);
            }else{
                transf->applyTransform(_v);
            }
        }
    }

    void Joint::applySecondDeriv(const Dof* _q1, const Dof* _q2, Matrix4d& _m){
        for(int i=mTransforms.size()-1; i>=0; i--){
            Transformation* transf = mTransforms[i];
            if(transf->isPresent(_q1) && transf->isPresent(_q2)){
                transf->applySecondDeriv(_q1, _q2, _m);
            }else if(transf->isPresent(_q1)){
                transf->applyDeriv(_q1, _m);
            }else if(transf->isPresent(_q2)){
                transf->applyDeriv(_q2, _m);
            }else{
                transf->applyTransform(_m);
            }
        }
    }

    void Joint::addTransform(Transformation *_t, bool _isVariable){
        _t->setJoint(this);
        _t->setVariable(_isVariable);
        mTransforms.push_back(_t);
        if(_isVariable) {	// add dofs of variable transform only
            for(int i=0; i<_t->getNumDofs(); i++){
                addDof(_t->getDof(i));
            }
        }
    }

    void Joint::addDof(Dof *_d) {
        mDofs.push_back(_d);
        _d->setJoint(this);
    }
  

} // namespace model3d

