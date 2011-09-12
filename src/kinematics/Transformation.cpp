/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Transformation.h"

#include "utils/Misc.h" // For M_PI
#include "utils/UtilsMath.h"
#include "renderer/RenderInterface.h"

#include "Dof.h"
#include "Joint.h"

using namespace std;
using namespace Eigen;

namespace kinematics {

    Transformation::Transformation()
        :mTransform(Matrix4d::Zero()) {
            strcpy(mName,"");
            mJoint=NULL;
            mVariable=true;
            mSkelIndex=-1;
            mDirty = true;
    }

    Transformation::~Transformation(){
        for(unsigned int i=0; i<mDofs.size(); i++) {
            delete mDofs[i];
        }
        mDofs.clear();
    }

    Matrix4d Transformation::getTransform() {
        if ( mDirty ) {
            computeTransform();
            mDirty = false;
        }
        return mTransform;
    }

    Matrix4d Transformation::getInvTransform() {
        if ( mDirty ) {
            computeTransform();
            mDirty = false;
        }
        return mTransform.inverse();
    }

    bool Transformation::isPresent(const Dof *q) const {
        for(unsigned int i=0; i<mDofs.size(); i++){
            if(q==mDofs[i]) return true;
        }
        return false;
    }

    void Transformation::applyTransform(Vector3d& _v) {
        _v= utils::xformHom(getTransform(), _v);
        Vector4d v4d( _v[0], _v[1], _v[2], 1 );
        v4d = getTransform() *v4d;
        _v = Vector3d( v4d[0], v4d[1], v4d[2] );
    }

    void Transformation::applyTransform(Matrix4d& _m) {
        _m = getTransform() *_m;
    }

    void Transformation::applyInvTransform(Vector3d& _v) {
        _v = utils::xformHom(getInvTransform(), _v);
    }

    void Transformation::applyInvTransform(Matrix4d& _m) {
        _m = getInvTransform() *_m;
    }

    void Transformation::applyDeriv(const Dof*_q, Vector3d& _v) {
        _v = utils::xformHom(getDeriv(_q), _v);
    }

    void Transformation::applyDeriv(const Dof*_q, Matrix4d& _m)  {
        _m = getDeriv(_q) *_m;
    }

    void Transformation::applySecondDeriv(const Dof*_q1, const Dof*_q2, Vector3d& _v) {
        _v = utils::xformHom(getSecondDeriv(_q1, _q2), _v);
    }

    void Transformation::applySecondDeriv(const Dof*_q1, const Dof*_q2, Matrix4d& _m) {
        _m = getSecondDeriv(_q1, _q2)*_m;
    }


} // namespace kinematics
