/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "Transformation.h"

#include "utils/Misc.h" // For M_PI
#include "utils/EigenHelper.h"
#include "renderer/RenderInterface.h"

#include "Dof.h"
#include "Joint.h"

using namespace std;
using namespace Eigen;

namespace model3d {

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

    void Transformation::applyTransform(Vector3d& v) {
        using eigenhelper::xform;
        v= xform(getTransform(), v);
        Vector4d v4d( v[0], v[1], v[2], 1 );
        v4d = getTransform() * v4d;
        v = Vector3d( v4d[0], v4d[1], v4d[2] );
    }

    void Transformation::applyTransform(Matrix4d& m) {
        m = getTransform() * m;
    }

    void Transformation::applyInvTransform(Vector3d& v) {
        using eigenhelper::xform;
        v = xform(getInvTransform(), v);
    }

    void Transformation::applyInvTransform(Matrix4d& m) {
        m = getInvTransform() * m;
    }

    void Transformation::applyDeriv(const Dof* q, Vector3d& v) {
        using eigenhelper::xform;
        v = xform(getDeriv(q), v);
    }

    void Transformation::applyDeriv(const Dof* q, Matrix4d& m)  {
        m = getDeriv(q) * m;
    }

    void Transformation::applySecondDeriv(const Dof* q1, const Dof* q2, Vector3d& v) {
        using eigenhelper::xform;
        v = xform(getSecondDeriv(q1, q2), v);
    }

    void Transformation::applySecondDeriv(const Dof* q1, const Dof* q2, Matrix4d& m) {
        m = getSecondDeriv(q1, q2)*m;
    }


} // namespace model3d
