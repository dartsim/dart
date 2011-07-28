/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "TrfmRotateExpmap.h"
#include "Dof.h"
#include "utils/UtilsRotation.h"
#include "utils/UtilsMath.h"


using namespace std;
using namespace Eigen;

namespace model3d {

    TrfmRotateExpMap::TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, const char* _name){
        mDofs.resize(3);
        mDofs[0]=x;
        mDofs[1]=y;
        mDofs[2]=z;
        x->setTrans(this);
        y->setTrans(this);
        z->setTrans(this);
        mType = Transformation::T_ROTATEEXPMAP;
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "EXPMAP");
    }

    void TrfmRotateExpMap::computeTransform(){
        Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

        Matrix3d rot = utils::rotation::expMapRot(q);
        mTransform.setZero();
        mTransform.topLeftCorner(3,3) = rot;
        mTransform(3, 3) = 1.0;
    }

    Matrix4d TrfmRotateExpMap::getInvTransform(){
        if(mDirty){
            computeTransform();
            mDirty=false;
        }
        return mTransform.transpose();
    }

 
    Matrix4d TrfmRotateExpMap::getDeriv(const Dof *d){
        Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

        // derivative wrt which dof 
        int j=-1;
        for(unsigned int i=0; i<mDofs.size(); i++) if(d==mDofs[i]) j=i;
        assert(j!=-1);
        assert(j>=0 && j<=2);

        Matrix3d R = utils::rotation::expMapRot(q);
        Matrix3d J = utils::rotation::expMapJac(q);
        Matrix3d dRdj = utils::makeSkewSymmetric(J.col(j))*R;

        Matrix4d dRdj4d = Matrix4d::Zero();
        dRdj4d.topLeftCorner(3,3) = dRdj;

        return dRdj4d;
    }

    Matrix4d TrfmRotateExpMap::getSecondDeriv(const Dof *q1, const Dof *q2){
        Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

        // derivative wrt which mDofs
        int j=-1, k=-1;
        for(unsigned int i=0; i<mDofs.size(); i++) {
            if(q1==mDofs[i]) j=i;
            if(q2==mDofs[i]) k=i;
        }
        assert(j!=-1);
        assert(k!=-1);
        assert(j>=0 && j<=2);
        assert(k>=0 && k<=2);

        Matrix3d R = utils::rotation::expMapRot(q);
        Matrix3d J = utils::rotation::expMapJac(q);
        Matrix3d Jjss = utils::makeSkewSymmetric(J.col(j));
        Matrix3d Jkss = utils::makeSkewSymmetric(J.col(k));
        Matrix3d dJjdkss = utils::makeSkewSymmetric(utils::rotation::expMapJacDeriv(q, k).col(j));

        Matrix3d d2Rdidj = (Jjss*Jkss + dJjdkss)*R;

        Matrix4d d2Rdidj4 = Matrix4d::Zero();
        d2Rdidj4.topLeftCorner(3,3) = d2Rdidj;

        return d2Rdidj4;
    }

    void TrfmRotateExpMap::applyGLTransform(renderer::RenderInterface* _ri) const{
        Vector3d v(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
        double theta = v.norm();
        Vector3d vhat = Vector3d::Zero();
        if(!utils::isZero(theta)) {
            vhat= v/theta;
            _ri->rotate(vhat, theta * 180 / M_PI);
        }

    }
} // namespace model3d
