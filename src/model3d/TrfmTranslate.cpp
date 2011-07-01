/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "TrfmTranslate.h"
#include "Dof.h"


#include <iostream>
using namespace std;
using namespace Eigen;

namespace model3d {
    TrfmTranslate::TrfmTranslate(Dof *x, Dof *y, Dof *z, char* _name){
        mDofs.resize(3);
        mDofs[0]=x;
        mDofs[1]=y;
        mDofs[2]=z;
        mDofs[0]->setTrans(this);
        mDofs[1]->setTrans(this);
        mDofs[2]->setTrans(this);
        mType = Transformation::T_TRANSLATE;
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "Translate");
    }

    void TrfmTranslate::applyGLTransform(renderer::RenderInterface* RI) const{
#if 1
		if (RI)
			RI->Translate(Vector3d(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue()));
#else
        glTranslatef(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
#endif
    }

    void TrfmTranslate::evalTransform(){
        mTransform.setZero();
        mTransform(0, 0) = 1.0; 
        mTransform(1, 1) = 1.0; 
        mTransform(2, 2) = 1.0; 
        mTransform(3, 3) = 1.0; 
        for(unsigned int i=0; i<mDofs.size(); i++){
            mTransform(i, 3) = mDofs[i]->getValue();
        }
    }

    Matrix4d TrfmTranslate::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        for(unsigned int i=0; i<mDofs.size(); i++)
            if(mDofs[i] == q){
                ret(i, 3) = 1.0;
                break;
            }
        return ret;
    }

    void TrfmTranslate::applyDeriv(const Dof* q, Vector3d& v){
        for(unsigned int i=0; i<mDofs.size(); i++){
            if(mDofs[i] != q) v(i) = 0;
            else v(i) = 1;
        }
    }

    void TrfmTranslate::applyDeriv(const Dof* q, Matrix4d& m){
        for(unsigned int i=0; i<mDofs.size(); i++){
            if(mDofs[i] != q) m.row(i).setZero();
            else m.row(i) = m.row(3);
        }
        m.row(3).setZero();
    }

    Matrix4d TrfmTranslate::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        for(unsigned int i=0; i<mDofs.size(); i++)
            ret(i, 3) = -mDofs[i]->getValue();
        return ret;
    }

    void TrfmTranslate::applyTransform(Vector3d& v){
        for(unsigned int i=0; i<mDofs.size(); i++) 
            v(i) +=mDofs[i]->getValue();
    }

    void TrfmTranslate::applyInvTransform(Vector3d& v){
        for(unsigned int i=0; i<mDofs.size(); i++)
            v(i) -= mDofs[i]->getValue();
    }

    void TrfmTranslate::applyTransform(Matrix4d& m){
        for(unsigned int i=0; i<mDofs.size(); i++) {
            m.row(i) += mDofs[i]->getValue() * m.row(3);
        }
    }

    void TrfmTranslate::applyInvTransform(Matrix4d& m){
        for(unsigned int i=0; i<mDofs.size(); i++) {
            m.row(i) -= mDofs[i]->getValue() * m.row(3);
        }
    }

    TrfmTranslateX::TrfmTranslateX(Dof *x, char *_name){
        mDofs.resize(1);
        mDofs[0]=x;
        mDofs[0]->setTrans(this);
        mType = Transformation::T_TRANSLATEX;
        strcpy(mName, _name);
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "TranslateX");
    }

    void TrfmTranslateX::applyGLTransform(renderer::RenderInterface* RI) const{
#if 1
		if (RI)
			RI->Translate(Vector3d(mDofs[0]->getValue(), 0, 0));
#else
		glTranslatef(mDofs[0]->getValue(), 0, 0);
#endif    
	}

    void TrfmTranslateX::evalTransform(){
        mTransform.setZero();
        mTransform(0, 0) = 1.0; 
        mTransform(1, 1) = 1.0; 
        mTransform(2, 2) = 1.0; 
        mTransform(3, 3) = 1.0; 
        mTransform(_X, 3) = mDofs[0]->getValue(); 
    }

    Matrix4d TrfmTranslateX::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0] == q)
            ret(_X, 3) = 1.0;
        return ret;
    }

    void TrfmTranslateX::applyDeriv(const Dof* q, Vector3d& v){
        if(mDofs[0] != q) v=Vector3d::Zero();
        else { 
            v = Vector3d(1, 0, 0);
            // v = Vector3d::Zero();
            // v(0) = 1;
        }
    }

    void TrfmTranslateX::applyDeriv(const Dof* q, Matrix4d& m){
        if(mDofs[0] != q) m=Matrix4d::Zero();
        else for(int i=0; i<4; i++){
                if(i==_X) m.row(i) = m.row(3);
                else m.row(i).setZero();
            }
    }

    Matrix4d TrfmTranslateX::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        ret(_X, 3) = -mDofs[0]->getValue();
        return ret;
    }

    void TrfmTranslateX::applyTransform(Vector3d& v){
        v(_X) += mDofs[0]->getValue();
    }

    void TrfmTranslateX::applyInvTransform(Vector3d& v){
        v(_X) -= mDofs[0]->getValue();
    }

    void TrfmTranslateX::applyTransform(Matrix4d& m){
        m.row(_X) += mDofs[0]->getValue() * m.row(3);
    }

    void TrfmTranslateX::applyInvTransform(Matrix4d& m){
        m.row(_X) -= mDofs[0]->getValue() * m.row(3);
    }

    TrfmTranslateY::TrfmTranslateY(Dof *y, char *_name){
        mDofs.resize(1);
        mDofs[0]=y;
        mDofs[0]->setTrans(this);
        mType = Transformation::T_TRANSLATEY;
        strcpy(mName, _name);
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "TranslateY");
    }

    void TrfmTranslateY::applyGLTransform(renderer::RenderInterface* RI) const {
#if 1
		if (RI)
			RI->Translate(Vector3d(0, mDofs[0]->getValue(), 0));
#else
		glTranslatef(0, mDofs[0]->getValue(), 0);
#endif
    }

    void TrfmTranslateY::evalTransform(){
        mTransform.setZero();
        mTransform(0, 0) = 1.0; 
        mTransform(1, 1) = 1.0; 
        mTransform(2, 2) = 1.0; 
        mTransform(3, 3) = 1.0; 
        mTransform(_Y, 3) = mDofs[0]->getValue(); 
    }

    Matrix4d TrfmTranslateY::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0] == q)
            ret(_Y, 3) = 1.0;
        return ret;
    }

    void TrfmTranslateY::applyDeriv(const Dof* q, Vector3d& v){
        if(mDofs[0] != q) v=Vector3d::Zero();
        else {
            v = Vector3d(0, 1, 0);
            // equivalent to the following?
            // v = Vector3d::Zero();
            // v(1) = 1;
        }
    }

    void TrfmTranslateY::applyDeriv(const Dof* q, Matrix4d& m){
        if(mDofs[0] != q) m=Matrix4d::Zero();
        else for(int i=0; i<4; i++){
                if(i==_Y) m.row(i)=m.row(3);
                else m.row(i).setZero();
            }
    }

    Matrix4d TrfmTranslateY::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        ret(_Y, 3) = -mDofs[0]->getValue();
        return ret;
    }

    void TrfmTranslateY::applyTransform(Vector3d& v){
        v(_Y) += mDofs[0]->getValue();
    }

    void TrfmTranslateY::applyInvTransform(Vector3d& v){
        v(_Y) -= mDofs[0]->getValue();
    }

    void TrfmTranslateY::applyTransform(Matrix4d& m){
        m.row(_Y) += mDofs[0]->getValue() * m.row(3);
    }

    void TrfmTranslateY::applyInvTransform(Matrix4d& m){
        m.row(_Y) -= mDofs[0]->getValue() * m.row(3);
    }

    TrfmTranslateZ::TrfmTranslateZ(Dof *z, char *_name){
        mDofs.resize(1);
        mDofs[0]=z;
        mDofs[0]->setTrans(this);
        mType = Transformation::T_TRANSLATEZ;
        strcpy(mName, _name);
        if(_name!=NULL)
            strcpy(mName, _name);
        else
            strcpy(mName, "TranslateZ");
    }

    void TrfmTranslateZ::applyGLTransform(renderer::RenderInterface* RI) const {
#if 1
		if (RI)
			RI->Translate(Vector3d(0, 0, mDofs[0]->getValue()));
#else
		glTranslatef(0, 0, mDofs[0]->getValue());
#endif    
	}

    void TrfmTranslateZ::evalTransform(){
        mTransform.setZero();
        mTransform(0, 0) = 1.0; 
        mTransform(1, 1) = 1.0; 
        mTransform(2, 2) = 1.0; 
        mTransform(3, 3) = 1.0; 
        mTransform(_Z, 3) = mDofs[0]->getValue(); 
    }

    Matrix4d TrfmTranslateZ::getDeriv(const Dof *q){
        Matrix4d ret = Matrix4d::Zero();
        if(mDofs[0] == q)
            ret(_Z, 3) = 1.0;
        return ret;
    }

    void TrfmTranslateZ::applyDeriv(const Dof* q, Vector3d& v){
        if(mDofs[0] != q) v=Vector3d::Zero();
        else v=Vector3d(0,0,1);
    }

    void TrfmTranslateZ::applyDeriv(const Dof* q, Matrix4d& m){
        if(mDofs[0] != q) m = Matrix4d::Zero();
        else for(int i=0; i<4; i++){
                if(i==_Z) m.row(i) = m.row(3);
                else m.row(i).setZero();
            }
    }

    Matrix4d TrfmTranslateZ::getInvTransform(){
        Matrix4d ret = Matrix4d::Ones();
        ret(_Z, 3) = -mDofs[0]->getValue();
        return ret;
    }

    void TrfmTranslateZ::applyTransform(Vector3d& v){
        v(_Z) += mDofs[0]->getValue();
    }

    void TrfmTranslateZ::applyInvTransform(Vector3d& v){
        v(_Z) -= mDofs[0]->getValue();
    }

    void TrfmTranslateZ::applyTransform(Matrix4d& m){
        m.row(_Z) += mDofs[0]->getValue() * m.row(3);
    }

    void TrfmTranslateZ::applyInvTransform(Matrix4d& m){
        m.row(_Z) -= mDofs[0]->getValue() * m.row(3);
    }

} // namespace model3d
