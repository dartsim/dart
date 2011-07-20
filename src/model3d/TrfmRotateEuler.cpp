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
    TrfmRotateEulerX::TrfmRotateEulerX(Dof *q, const char *_name){
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

    void TrfmRotateEulerX::computeTransform(){
        mTransform.setZero();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        mTransform(A_X, A_X) = 1.0; 
        mTransform(A_Y, A_Y) = cosq; 
        mTransform(A_Y, A_Z) = -sinq; 
        mTransform(A_Z, A_Y) = sinq; 
        mTransform(A_Z, A_Z) = cosq; 
        mTransform(3, 3) = 1.0; 
    }

    Matrix4d TrfmRotateEulerX::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(q==mDofs[0]){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(A_Y, A_Y) = -sinq;
            ret(A_Y, A_Z) = -cosq;
            ret(A_Z, A_Y) = cosq;
            ret(A_Z, A_Z) = -sinq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerX::getSecondDeriv(const Dof *q1, const Dof *q2){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0]==q1 && mDofs[0]==q2){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(A_Y, A_Y) = -cosq;
            ret(A_Y, A_Z) = sinq;
            ret(A_Z, A_Y) = -sinq;
            ret(A_Z, A_Z) = -cosq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerX::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_Y, A_Y) = cosq;
        ret(A_Y, A_Z) = sinq;
        ret(A_Z, A_Y) = -sinq;
        ret(A_Z, A_Z) = cosq;
        return ret;
    }

////////////////////////////////////////////////////////////////////////////////
// rotation about Y axis
    TrfmRotateEulerY::TrfmRotateEulerY(Dof *q, const char *_name){
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

    void TrfmRotateEulerY::computeTransform(){
        mTransform.setZero();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        mTransform(A_Y, A_Y) = 1.0; 
        mTransform(A_X, A_X) = cosq; 
        mTransform(A_X, A_Z) = sinq; 
        mTransform(A_Z, A_X) = -sinq; 
        mTransform(A_Z, A_Z) = cosq; 
        mTransform(3, 3) = 1.0; 
    }

    Matrix4d TrfmRotateEulerY::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(q==mDofs[0]){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(A_X, A_X) = -sinq;
            ret(A_X, A_Z) = cosq;
            ret(A_Z, A_X) = -cosq;
            ret(A_Z, A_Z) = -sinq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerY::getSecondDeriv(const Dof *q1, const Dof *q2){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0]==q1 && mDofs[0]==q2){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(A_X, A_X) = -cosq;
            ret(A_X, A_Z) = -sinq;
            ret(A_Z, A_X) = sinq;
            ret(A_Z, A_Z) = -cosq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerY::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_X, A_X) = cosq;
        ret(A_X, A_Z) = -sinq;
        ret(A_Z, A_X) = sinq;
        ret(A_Z, A_Z) = cosq;
        return ret;
    }

////////////////////////////////////////////////////////////////////////////////
// rotation about Z axis
    TrfmRotateEulerZ::TrfmRotateEulerZ(Dof *q, const char* _name){
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

    void TrfmRotateEulerZ::computeTransform(){
        mTransform.setZero();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        mTransform(A_Z, A_Z) = 1.0; 
        mTransform(A_X, A_X) = cosq; 
        mTransform(A_X, A_Y) = -sinq; 
        mTransform(A_Y, A_X) = sinq; 
        mTransform(A_Y, A_Y) = cosq; 
        mTransform(3, 3) = 1.0; 
    }

    Matrix4d TrfmRotateEulerZ::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(q==mDofs[0]){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(A_X, A_X) = -sinq;
            ret(A_X, A_Y) = -cosq;
            ret(A_Y, A_X) = cosq;
            ret(A_Y, A_Y) = -sinq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerZ::getSecondDeriv(const Dof *q1, const Dof *q2){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0]==q1 && mDofs[0]==q2){
            double cosq = cos(mDofs[0]->getValue());
            double sinq = sin(mDofs[0]->getValue());
            ret(A_X, A_X) = -cosq;
            ret(A_X, A_Y) = sinq;
            ret(A_Y, A_X) = -sinq;
            ret(A_Y, A_Y) = -cosq;
        }
        return ret;
    }

    Matrix4d TrfmRotateEulerZ::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        double cosq = cos(mDofs[0]->getValue());
        double sinq = sin(mDofs[0]->getValue());
        ret(A_X, A_X) = cosq;
        ret(A_X, A_Y) = sinq;
        ret(A_Y, A_X) = -sinq;
        ret(A_Y, A_Y) = cosq;
        return ret;
    }
//

} // namespace model3d
