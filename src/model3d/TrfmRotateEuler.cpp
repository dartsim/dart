/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "TrfmRotateEuler.h"
#include "Dof.h"

using namespace Eigen;

namespace model3d {
////////////////////////////////////////////////////////////////////////////////
// rotation about X axis
    TrfmRotateEulerX::TrfmRotateEulerX(Dof *q, char *_name){
        mDofs.push_back(q);
        q->setTrans(this);
        mType = Transformation::T_ROTATEX;
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "EulerX");
    }

	void TrfmRotateEulerX::applyGLTransform(renderer::RenderInterface* _ri) const{
		if (_ri)
			_ri->rotate(Vector3d(1.0, 0.0, 0.0), mDofs[0]->getValue()*180/M_PI);
    }

    void TrfmRotateEulerX::evalTransform(){
        mTransform.setZero();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        mTransform(_X, _X) = 1.0; 
        mTransform(_Y, _Y) = cosq; 
        mTransform(_Y, _Z) = -sinq; 
        mTransform(_Z, _Y) = sinq; 
        mTransform(_Z, _Z) = cosq; 
        mTransform(3, 3) = 1.0; 
    }

    Matrix4d TrfmRotateEulerX::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(q==mDofs[0]){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(_Y, _Y) = -sinq;
            ret(_Y, _Z) = -cosq;
            ret(_Z, _Y) = cosq;
            ret(_Z, _Z) = -sinq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerX::getDeriv2(const Dof *q1, const Dof *q2){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0]==q1 && mDofs[0]==q2){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(_Y, _Y) = -cosq;
            ret(_Y, _Z) = sinq;
            ret(_Z, _Y) = -sinq;
            ret(_Z, _Z) = -cosq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerX::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(_Y, _Y) = cosq;
        ret(_Y, _Z) = sinq;
        ret(_Z, _Y) = -sinq;
        ret(_Z, _Z) = cosq;
        return ret;
    }

////////////////////////////////////////////////////////////////////////////////
// rotation about Y axis
    TrfmRotateEulerY::TrfmRotateEulerY(Dof *q, char *_name){
        mDofs.push_back(q);
        q->setTrans(this);
        mType = Transformation::T_ROTATEY;
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "EulerY");
    }

	void TrfmRotateEulerY::applyGLTransform(renderer::RenderInterface* _ri) const{
		if (_ri)
			_ri->rotate(Vector3d(0.0, 1.0, 0.0), mDofs[0]->getValue()*180/M_PI);
      
    }

    void TrfmRotateEulerY::evalTransform(){
        mTransform.setZero();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        mTransform(_Y, _Y) = 1.0; 
        mTransform(_X, _X) = cosq; 
        mTransform(_X, _Z) = sinq; 
        mTransform(_Z, _X) = -sinq; 
        mTransform(_Z, _Z) = cosq; 
        mTransform(3, 3) = 1.0; 
    }

    Matrix4d TrfmRotateEulerY::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(q==mDofs[0]){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(_X, _X) = -sinq;
            ret(_X, _Z) = cosq;
            ret(_Z, _X) = -cosq;
            ret(_Z, _Z) = -sinq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerY::getDeriv2(const Dof *q1, const Dof *q2){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0]==q1 && mDofs[0]==q2){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(_X, _X) = -cosq;
            ret(_X, _Z) = -sinq;
            ret(_Z, _X) = sinq;
            ret(_Z, _Z) = -cosq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerY::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(_X, _X) = cosq;
        ret(_X, _Z) = -sinq;
        ret(_Z, _X) = sinq;
        ret(_Z, _Z) = cosq;
        return ret;
    }

////////////////////////////////////////////////////////////////////////////////
// rotation about Z axis
    TrfmRotateEulerZ::TrfmRotateEulerZ(Dof *q, char* _name){
        mDofs.push_back(q);
        q->setTrans(this);
        mType = Transformation::T_ROTATEZ;
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "EulerZ");
    }

	void TrfmRotateEulerZ::applyGLTransform(renderer::RenderInterface* _ri) const{
		if (_ri)
			_ri->rotate(Vector3d(0.0, 0.0, 1.0), mDofs[0]->getValue()*180/M_PI); 
	}

    void TrfmRotateEulerZ::evalTransform(){
        mTransform.setZero();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        mTransform(_Z, _Z) = 1.0; 
        mTransform(_X, _X) = cosq; 
        mTransform(_X, _Y) = -sinq; 
        mTransform(_Y, _X) = sinq; 
        mTransform(_Y, _Y) = cosq; 
        mTransform(3, 3) = 1.0; 
    }

    Matrix4d TrfmRotateEulerZ::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(q==mDofs[0]){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(_X, _X) = -sinq;
            ret(_X, _Y) = -cosq;
            ret(_Y, _X) = cosq;
            ret(_Y, _Y) = -sinq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerZ::getDeriv2(const Dof *q1, const Dof *q2){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0]==q1 && mDofs[0]==q2){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(_X, _X) = -cosq;
            ret(_X, _Y) = sinq;
            ret(_Y, _X) = -sinq;
            ret(_Y, _Y) = -cosq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerZ::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(_X, _X) = cosq;
        ret(_X, _Y) = sinq;
        ret(_Y, _X) = -sinq;
        ret(_Y, _Y) = cosq;
        return ret;
    }
//

} // namespace model3d
