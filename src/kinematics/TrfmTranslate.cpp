/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "TrfmTranslate.h"
#include "Dof.h"
#include "renderer/RenderInterface.h"


#include <iostream>
using namespace std;
using namespace Eigen;

namespace kinematics {
TrfmTranslate::TrfmTranslate(Dof *x, Dof *y, Dof *z, const char* _name){
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
        strcpy(mName, "translate");
}

void TrfmTranslate::applyGLTransform(renderer::RenderInterface* _ri) const{
	if (_ri)
		_ri->translate(Vector3d(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue()));
}

void TrfmTranslate::computeTransform(){
    mTransform.setZero();
    mTransform(0, 0) = 1.0;
    mTransform(1, 1) = 1.0;
    mTransform(2, 2) = 1.0;
    mTransform(3, 3) = 1.0;
    for(unsigned int i=0; i<mDofs.size(); i++){
        mTransform(i, 3) = mDofs[i]->getValue();
    }
}

Matrix4d TrfmTranslate::getDeriv(const Dof *q) const{
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

Matrix4d TrfmTranslate::getSecondDeriv( const Dof *q1, const Dof *q2 ) const {
    return Eigen::Matrix4d::Zero(); // always zero
}

void TrfmTranslate::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Vector3d& v ) {
    v.setZero();
}

void TrfmTranslate::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Matrix4d& m ) {
    m.setZero();
}

Eigen::MatrixXd TrfmTranslate::getJacobian() const {
    assert(getNumDofs() == 3);
    Eigen::MatrixXd J = Eigen::Matrix<double,6,3>::Zero();

    Eigen::Matrix<double,6,1> J1 = Eigen::Matrix<double,6,1>::Zero();
    J1(A_X+3) = 1.0;
    J.col(0) = J1;
    Eigen::Matrix<double,6,1> J2 = Eigen::Matrix<double,6,1>::Zero();
    J2(A_Y+3) = 1.0;
    J.col(1) = J2;
    Eigen::Matrix<double,6,1> J3 = Eigen::Matrix<double,6,1>::Zero();
    J3(A_Z+3) = 1.0;
    J.col(2) = J3;

    return J;
}


Matrix4d TrfmTranslate::getInvTransform(){
    Matrix4d ret = Matrix4d::Identity();
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

TrfmTranslateX::TrfmTranslateX(Dof *x, const char *_name){
    mDofs.resize(1);
    mDofs[0]=x;
    mDofs[0]->setTrans(this);
    mType = Transformation::T_TRANSLATEX;
    if(_name!=NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "TranslateX");
}

void TrfmTranslateX::applyGLTransform(renderer::RenderInterface* _ri) const{
	if (_ri)
		_ri->translate(Vector3d(mDofs[0]->getValue(), 0, 0));
}

void TrfmTranslateX::computeTransform(){
    mTransform.setZero();
    mTransform(0, 0) = 1.0;
    mTransform(1, 1) = 1.0;
    mTransform(2, 2) = 1.0;
    mTransform(3, 3) = 1.0;
    mTransform(A_X, 3) = mDofs[0]->getValue();
}

Matrix4d TrfmTranslateX::getDeriv(const Dof *q) const{
    Matrix4d ret = Matrix4d::Zero();
    if(mDofs[0] == q)
        ret(A_X, 3) = 1.0;
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
        if(i==A_X) m.row(i) = m.row(3);
        else m.row(i).setZero();
    }
}

Matrix4d TrfmTranslateX::getSecondDeriv( const Dof *q1, const Dof *q2 ) const {
    return Eigen::Matrix4d::Zero(); // always zero
}

void TrfmTranslateX::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Vector3d& v ) {
    v.setZero();
}

void TrfmTranslateX::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Matrix4d& m ) {
    m.setZero();
}

Eigen::MatrixXd TrfmTranslateX::getJacobian() const
{
    Eigen::MatrixXd J = Eigen::Matrix<double,6,1>::Zero();

    J(A_X+3) = 1.0;

    return J;
}

Matrix4d TrfmTranslateX::getInvTransform(){
    Matrix4d ret = Matrix4d::Ones();
    ret(A_X, 3) = -mDofs[0]->getValue();
    return ret;
}

void TrfmTranslateX::applyTransform(Vector3d& v){
    v(A_X) += mDofs[0]->getValue();
}

void TrfmTranslateX::applyInvTransform(Vector3d& v){
    v(A_X) -= mDofs[0]->getValue();
}

void TrfmTranslateX::applyTransform(Matrix4d& m){
    m.row(A_X) += mDofs[0]->getValue() * m.row(3);
}

void TrfmTranslateX::applyInvTransform(Matrix4d& m){
    m.row(A_X) -= mDofs[0]->getValue() * m.row(3);
}

TrfmTranslateY::TrfmTranslateY(Dof *y, const char *_name){
    mDofs.resize(1);
    mDofs[0]=y;
    mDofs[0]->setTrans(this);
    mType = Transformation::T_TRANSLATEY;
    if(_name!=NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "TranslateY");
}

void TrfmTranslateY::applyGLTransform(renderer::RenderInterface* _ri) const {
	if (_ri)
		_ri->translate(Vector3d(0, mDofs[0]->getValue(), 0));
}

void TrfmTranslateY::computeTransform(){
    mTransform.setZero();
    mTransform(0, 0) = 1.0;
    mTransform(1, 1) = 1.0;
    mTransform(2, 2) = 1.0;
    mTransform(3, 3) = 1.0;
    mTransform(A_Y, 3) = mDofs[0]->getValue();
}

Matrix4d TrfmTranslateY::getDeriv(const Dof *q) const{
    Matrix4d ret = Matrix4d::Zero();
    if(mDofs[0] == q)
        ret(A_Y, 3) = 1.0;
    return ret;
}

void TrfmTranslateY::applyDeriv(const Dof* q, Vector3d& v){
    if(mDofs[0] != q) v=Vector3d::Zero();
    else {
        v = Vector3d(0, 1, 0);
    }
}

void TrfmTranslateY::applyDeriv(const Dof* q, Matrix4d& m){
    if(mDofs[0] != q) m=Matrix4d::Zero();
    else for(int i=0; i<4; i++){
        if(i==A_Y) m.row(i)=m.row(3);
        else m.row(i).setZero();
    }
}

Matrix4d TrfmTranslateY::getSecondDeriv( const Dof *q1, const Dof *q2 ) const {
    return Eigen::Matrix4d::Zero(); // always zero
}

void TrfmTranslateY::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Vector3d& v ) {
    v.setZero();
}

void TrfmTranslateY::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Matrix4d& m ) {
    m.setZero();
}

Eigen::MatrixXd TrfmTranslateY::getJacobian() const {
    Eigen::MatrixXd J = Eigen::Matrix<double,6,1>::Zero();

    J(A_Y+3) = 1.0;

    return J;
}

Matrix4d TrfmTranslateY::getInvTransform(){
    Matrix4d ret = Matrix4d::Ones();
    ret(A_Y, 3) = -mDofs[0]->getValue();
    return ret;
}

void TrfmTranslateY::applyTransform(Vector3d& v){
    v(A_Y) += mDofs[0]->getValue();
}

void TrfmTranslateY::applyInvTransform(Vector3d& v){
    v(A_Y) -= mDofs[0]->getValue();
}

void TrfmTranslateY::applyTransform(Matrix4d& m){
    m.row(A_Y) += mDofs[0]->getValue() * m.row(3);
}

void TrfmTranslateY::applyInvTransform(Matrix4d& m){
    m.row(A_Y) -= mDofs[0]->getValue() * m.row(3);
}

TrfmTranslateZ::TrfmTranslateZ(Dof *z, const char *_name){
    mDofs.resize(1);
    mDofs[0]=z;
    mDofs[0]->setTrans(this);
    mType = Transformation::T_TRANSLATEZ;
    if(_name!=NULL)
        strcpy(mName, _name);
    else
        strcpy(mName, "TranslateZ");
}

void TrfmTranslateZ::applyGLTransform(renderer::RenderInterface* _ri) const {
	if (_ri)
		_ri->translate(Vector3d(0, 0, mDofs[0]->getValue()));
}

void TrfmTranslateZ::computeTransform(){
    mTransform.setZero();
    mTransform(0, 0) = 1.0;
    mTransform(1, 1) = 1.0;
    mTransform(2, 2) = 1.0;
    mTransform(3, 3) = 1.0;
    mTransform(A_Z, 3) = mDofs[0]->getValue();
}

Matrix4d TrfmTranslateZ::getDeriv(const Dof *q) const{
    Matrix4d ret = Matrix4d::Zero();
    if(mDofs[0] == q)
        ret(A_Z, 3) = 1.0;
    return ret;
}

void TrfmTranslateZ::applyDeriv(const Dof* q, Vector3d& v){
    if(mDofs[0] != q) v=Vector3d::Zero();
    else v=Vector3d(0,0,1);
}

void TrfmTranslateZ::applyDeriv(const Dof* q, Matrix4d& m){
    if(mDofs[0] != q) m = Matrix4d::Zero();
    else for(int i=0; i<4; i++){
        if(i==A_Z) m.row(i) = m.row(3);
        else m.row(i).setZero();
    }
}

Matrix4d TrfmTranslateZ::getSecondDeriv( const Dof *q1, const Dof *q2 ) const {
    return Eigen::Matrix4d::Zero(); // always zero
}

void TrfmTranslateZ::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Vector3d& v ) {
    v.setZero();
}

void TrfmTranslateZ::applySecondDeriv( const Dof* q1, const Dof* q2, Eigen::Matrix4d& m ) {
    m.setZero();
}

Eigen::MatrixXd TrfmTranslateZ::getJacobian() const {
    Eigen::MatrixXd J = Eigen::Matrix<double,6,1>::Zero();

    J(A_Z+3) = 1.0;

    return J;
}

Matrix4d TrfmTranslateZ::getInvTransform(){
    Matrix4d ret = Matrix4d::Ones();
    ret(A_Z, 3) = -mDofs[0]->getValue();
    return ret;
}

void TrfmTranslateZ::applyTransform(Vector3d& v){
    v(A_Z) += mDofs[0]->getValue();
}

void TrfmTranslateZ::applyInvTransform(Vector3d& v){
    v(A_Z) -= mDofs[0]->getValue();
}

void TrfmTranslateZ::applyTransform(Matrix4d& m){
    m.row(A_Z) += mDofs[0]->getValue() * m.row(3);
}

void TrfmTranslateZ::applyInvTransform(Matrix4d& m){
    m.row(A_Z) -= mDofs[0]->getValue() * m.row(3);
}

} // namespace kinematics
