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

#include "Skeleton.h"

#include <cassert>
using namespace std;
using namespace Eigen;

#include "Dof.h"
#include "Joint.h"
#include "BodyNode.h"
#include "Marker.h"
#include "Transformation.h"

#include "renderer/RenderInterface.h"


namespace kinematics {

    Skeleton::Skeleton() {
        mMass = 0;
    }

    Skeleton::~Skeleton(){
        for(unsigned int i = 0; i < mJoints.size(); i++) delete mJoints[i];
        mJoints.clear();
        mDofs.clear();
        mTransforms.clear();
        for(unsigned int i=0; i<mNodes.size(); i++) delete mNodes[i];
        mNodes.clear();
        mMarkers.clear();
    }

    BodyNode* Skeleton::createBodyNode(const char* const _name) {
        return new BodyNode(_name);
    }

    void Skeleton::addMarker(Marker *_h) {
        mMarkers.push_back(_h);
        _h->setSkelIndex(mMarkers.size()-1);
        BodyNode *body = _h->getNode();
        body->addMarker(_h);
    }

    void Skeleton::addNode(BodyNode *_b) {
        mNodes.push_back(_b);
        _b->setSkelIndex(mNodes.size()-1);
        addJoint(_b->getParentJoint());
    }

    void Skeleton::addJoint(Joint *_j) {
        mJoints.push_back(_j);
        _j->setSkelIndex(mJoints.size()-1);
    }

    void Skeleton::addDof(Dof *_q) {
        mDofs.push_back(_q);
        _q->setSkelIndex(mDofs.size()-1);
        _q->setVariable();
    }

    void Skeleton::addTransform(Transformation *_t) {
        mTransforms.push_back(_t);
        _t->setVariable(true);
        _t->setSkelIndex(mTransforms.size()-1);
        for(int i=0; i<_t->getNumDofs(); i++) {
            addDof(_t->getDof(i));
        }
    }
  
    void Skeleton::initSkel() {
        mRoot = mNodes[0];

        // calculate mass
        // init the dependsOnDof stucture for each bodylink
        for(int i=0; i<getNumNodes(); i++) {
            mNodes[i]->setSkel(this);
            // mNodes[i]->setDependDofMap(getNumDofs());
            mNodes[i]->setDependDofList();
            mNodes.at(i)->init();
            mMass += mNodes[i]->getMass();
        }

        mCurrPose = VectorXd::Zero(getNumDofs());

        for(int i=0; i<getNumDofs(); i++)
            mCurrPose[i] = mDofs.at(i)->getValue();
        for(int i=0; i<getNumNodes(); i++) {
            mNodes.at(i)->updateTransform();
        }
    }

    BodyNode* Skeleton::getNode(const char* const name) {
        const int nNodes = getNumNodes();
        for(int i = 0; i < nNodes; i++){
            BodyNode* node = getNode(i);
            if (strcmp(name, node->getName()) == 0) {
                return node;
            }
        }
        return NULL;
    }

    int Skeleton::getNodeIndex(const char* const name) {
        const int nNodes = getNumNodes();
        for(int i = 0; i < nNodes; i++){
            BodyNode* node = getNode(i);
            if (strcmp(name, node->getName()) == 0) {
                return i;
            }
        }
        return -1;
    }

    Vector3d Skeleton::getWorldCOM() {
        assert(mMass != 0);
        Vector3d com(0, 0, 0);
        const int nNodes = getNumNodes();
        for(int i = 0; i < nNodes; i++) {
            BodyNode* node = getNode(i);
            com += (node->getMass() * node->getWorldCOM());
        }
        return com / mMass;
    }
  
    void Skeleton::setPose(const VectorXd& state, bool bCalcTrans, bool bCalcDeriv) {
        assert(state.size() == getNumDofs());
        int k=0;
        for(k=0; k<getNumDofs(); k++)
            if(mCurrPose[k]!=state[k]) break;
        if(k==getNumDofs()) return;

        mCurrPose = state;
        for(int i=0; i<getNumDofs(); i++) {
            mDofs.at(i)->setValue(state[i]);
        }

        if (bCalcTrans) {
            for(int i = 0; i < getNumNodes(); i++) {
                mNodes.at(i)->updateTransform();
            }
        }

        if (bCalcDeriv) {
            for(int i = 0; i < getNumNodes(); i++) {
                mNodes.at(i)->updateFirstDerivatives();
            }
        }
    }

    void Skeleton::setPose(const vector<double>& state, bool bCalcTrans, bool bCalcDeriv) {
        VectorXd x(state.size());
        for (unsigned int i = 0; i < state.size(); i++) {
            x(i) = state[i];
        }
        setPose(x, bCalcTrans, bCalcDeriv);
    }

    //void Skeleton::setPose(const VectorXd& _pose){
    //    assert(_pose.size() == getNumDofs());
    //    for(int i=0; i<getNumDofs(); i++)
    //        mDofs[i]->setValue(_pose(i));
    //}

    //void Skeleton::setPose(const vector<double>& _pose){
    //    assert(_pose.size() == getNumDofs());
    //    for(int i=0; i<getNumDofs(); i++)
    //        mDofs[i]->setValue(_pose[i]);
    //}

    void Skeleton::getPose(Eigen::VectorXd& _pose) {
        _pose.resize(getNumDofs());
        for (int i = 0; i < getNumDofs(); i++) {
            _pose(i) = mDofs[i]->getValue();
        }
    }
    void Skeleton::getPose(std::vector<double>& _pose) {
        _pose.resize(getNumDofs());
        for (int i = 0; i < getNumDofs(); i++) {
            _pose[i] = mDofs[i]->getValue();
        }
    }

    void Skeleton::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        mRoot->draw(_ri, _color, _useDefaultColor);
    }
    void Skeleton::drawMarkers(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        mRoot->drawMarkers(_ri, _color, _useDefaultColor);
    }


} // namespace kinematics
