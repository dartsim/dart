/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
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

#include "math/Geometry.h"
#include "math/Helpers.h"
#include "dynamics/BodyNode.h"
#include "dynamics/GenCoord.h"
#include "dynamics/Joint.h"
#include "dynamics/Marker.h"
#include "dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

Skeleton::Skeleton(const std::string& _name)
    : GenCoordSystem(),
      mName(_name),
      mIsSelfCollidable(false),
      mTotalMass(0.0),
      mIsMobile(true)
{
    for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
        delete (*it);
}

Skeleton::~Skeleton()
{
}

void Skeleton::setName(const std::string& _name)
{
    mName = _name;
}

const std::string& Skeleton::getName() const
{
    return mName;
}

void Skeleton::setSelfCollidable(bool _isSelfCollidable)
{
    mIsSelfCollidable = _isSelfCollidable;
}

bool Skeleton::isSelfCollidable() const
{
    return mIsSelfCollidable;
}

void Skeleton::setMobile(bool _isMobile)
{
    mIsMobile = _isMobile;
}

bool Skeleton::isMobile() const
{
    return mIsMobile;
}

double Skeleton::getMass() const
{
    return mTotalMass;
}

void Skeleton::init()
{
    mGenCoords.clear();

    // Initialize body nodes
    for(int i = 0; i < getNumBodyNodes(); i++)
    {
        Joint* joint = mBodyNodes[i]->getParentJoint();
        for (int j = 0; j < joint->getNumGenCoords(); ++j)
        {
            joint->getGenCoord(j)->setSkeletonIndex(mGenCoords.size());
            mGenCoords.push_back(joint->getGenCoord(j));
        }
        mBodyNodes[i]->init(this, i);
        mBodyNodes[i]->updateTransform();
        mBodyNodes[i]->updateVelocity();
    }

    int DOF = getNumGenCoords();

    mM    = Eigen::MatrixXd::Zero(DOF, DOF);
    mMInv = Eigen::MatrixXd::Zero(DOF, DOF);
    mC    = Eigen::MatrixXd::Zero(DOF, DOF);
    mCvec = Eigen::VectorXd::Zero(DOF);
    mG    = Eigen::VectorXd::Zero(DOF);
    mCg   = Eigen::VectorXd::Zero(DOF);
    set_tau(Eigen::VectorXd::Zero(DOF));
    mFext = Eigen::VectorXd::Zero(DOF);
    mFc   = Eigen::VectorXd::Zero(DOF);
    mDampingForce = Eigen::VectorXd::Zero(DOF);

    // calculate mass
    // init the dependsOnDof stucture for each bodylink
    mTotalMass = 0.0;
    for(int i = 0; i < getNumBodyNodes(); i++)
        mTotalMass += getBodyNode(i)->getMass();
}

void Skeleton::addBodyNode(BodyNode* _body)
{
    assert(_body && _body->getParentJoint());

    mBodyNodes.push_back(_body);
}

int Skeleton::getNumBodyNodes() const
{
    return mBodyNodes.size();
}

BodyNode* Skeleton::getRootBodyNode() const
{
    // We assume that the first element of body nodes is root.
    return mBodyNodes[0];
}

BodyNode* Skeleton::getBodyNode(int _idx) const
{
    return mBodyNodes[_idx];
}

BodyNode* Skeleton::getBodyNode(const std::string& _name) const
{
    assert(!_name.empty());

    for (std::vector<BodyNode*>::const_iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody) {
        if ((*itrBody)->getName() == _name)
            return *itrBody;
    }

    return NULL;
}

Joint* Skeleton::getJoint(int _idx) const
{
    return mBodyNodes[_idx]->getParentJoint();
}

Joint* Skeleton::getJoint(const std::string& _name) const
{
    assert(!_name.empty());

    for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
         it != mBodyNodes.end();
         ++it)
    {
        if ((*it)->getParentJoint()->getName() == _name)
            return (*it)->getParentJoint();
    }

    return NULL;
}

Marker* Skeleton::getMarker(const std::string& _name) const
{
    assert(!_name.empty());

    for (std::vector<BodyNode*>::const_iterator it = mBodyNodes.begin();
         it != mBodyNodes.end(); ++it)
    {
        for (int i = 0; i < (*it)->getNumMarkers(); ++i)
        {
            if ((*it)->getMarker(i)->getName() == _name)
                return (*it)->getMarker(i);
        }
    }

    return NULL;
}

Eigen::VectorXd Skeleton::getConfig(const std::vector<int>& _id) const
{
    Eigen::VectorXd q(_id.size());

    for(unsigned int i = 0; i < _id.size(); i++)
        q[i] = mGenCoords[_id[i]]->get_q();

    return q;
}

Eigen::VectorXd Skeleton::getConfig() const
{
    return get_q();
}

void Skeleton::setConfig(const std::vector<int>& _genCoords, const Eigen::VectorXd& _config)
{
    for( unsigned int i = 0; i < _genCoords.size(); i++ )
        mGenCoords[_genCoords[i]]->set_q(_config(i));

    for (std::vector<BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end(); ++itrBody)
    {
        (*itrBody)->updateTransform();
    }
}

void Skeleton::setConfig(const Eigen::VectorXd& _config)
{
    set_q(_config);

    for (std::vector<BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end(); ++itrBody)
    {
        (*itrBody)->updateTransform();
    }
}

void Skeleton::setState(const Eigen::VectorXd& _state)
{
    set_q(_state.head(_state.size() / 2));
    set_dq(_state.tail(_state.size() / 2));
    
    for (std::vector<BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end(); ++itrBody)
    {
        (*itrBody)->updateTransform();
        (*itrBody)->updateVelocity();
    }
}

Eigen::VectorXd Skeleton::getState()
{
    Eigen::VectorXd state(2 * mGenCoords.size());
    state << get_q(), get_dq();
    return state;
}

Eigen::MatrixXd Skeleton::getMassMatrix() const
{
    return mM;
}

Eigen::MatrixXd Skeleton::getInvMassMatrix() const
{
    return mMInv;
}

Eigen::MatrixXd Skeleton::getCoriolisMatrix() const
{
    return mC;
}

Eigen::VectorXd Skeleton::getCoriolisVector() const
{
    return mCvec;
}

Eigen::VectorXd Skeleton::getGravityVector() const
{
    return mG;
}

Eigen::VectorXd Skeleton::getCombinedVector() const
{
    return mCg;
}

Eigen::VectorXd Skeleton::getExternalForces() const
{
    return mFext;
}

Eigen::VectorXd Skeleton::getInternalForces() const
{
    return get_tau();
}

void Skeleton::draw(renderer::RenderInterface* _ri,
                    const Eigen::Vector4d& _color,
                    bool _useDefaultColor) const
{
    getRootBodyNode()->draw(_ri, _color, _useDefaultColor);
}

void Skeleton::drawMarkers(renderer::RenderInterface* _ri,
                           const Eigen::Vector4d& _color,
                           bool _useDefaultColor) const
{
    getRootBodyNode()->drawMarkers(_ri, _color, _useDefaultColor);
}

void Skeleton::computeInverseDynamicsLinear(const Eigen::Vector3d& _gravity,
                                      bool _computeJacobian,
                                      bool _computeJacobianDeriv,
                                      bool _withExternalForces,
                                      bool _withDampingForces)
{
    // Skip immobile or 0-dof skeleton
    if (!isMobile() || getNumGenCoords() == 0)
        return;

    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody
         = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody)
    {
        (*itrBody)->updateEta();
        (*itrBody)->updateAcceleration();
    }

    // Backward recursion
    for (std::vector<dynamics::BodyNode*>::reverse_iterator ritrBody
         = mBodyNodes.rbegin();
         ritrBody != mBodyNodes.rend();
         ++ritrBody)
    {
        (*ritrBody)->updateBodyForce(_gravity,
                                     _withExternalForces);
        (*ritrBody)->updateGeneralizedForce(_withDampingForces);
    }
}

void Skeleton::updateExternalForces()
{
    // Clear external force.
    mFext.setZero();

    // Recursive
    for (std::vector<BodyNode*>::iterator itr = mBodyNodes.begin();
         itr != mBodyNodes.end(); ++itr)
        (*itr)->aggregateExternalForces(mFext);
}

void Skeleton::updateDampingForces()
{
    // Clear external force.
    mDampingForce.setZero();

    for (std::vector<BodyNode*>::iterator itr = mBodyNodes.begin();
         itr != mBodyNodes.end(); ++itr)
    {
        Eigen::VectorXd jointDampingForce = (*itr)->getParentJoint()->getDampingForces();
        for (int i = 0; i < jointDampingForce.size(); i++)
        {
            mDampingForce((*itr)->getParentJoint()->getGenCoord(i)->getSkeletonIndex()) =
                    jointDampingForce(i);
        }
    }
}

void Skeleton::clearExternalForces()
{
    int nNodes = getNumBodyNodes();

    for (int i = 0; i < nNodes; i++)
        mBodyNodes[i]->clearExternalForces();
}

void Skeleton::computeEquationsOfMotionID(
        const Eigen::Vector3d& _gravity)
{
    int n = getNumGenCoords();

    // Skip immobile or 0-dof skeleton
    if (!isMobile() == true || n == 0)
        return;

    // Save current tau
    Eigen::VectorXd tau_old = get_tau();

    // Set ddq as zero
    set_ddq(Eigen::VectorXd::Zero(n));

    // M(q) * ddq + b(q,dq) = tau
    computeInverseDynamicsLinear(_gravity, true);
    mCg = get_tau();

    // Calcualtion mass matrix, M
    mM = Eigen::MatrixXd::Zero(n,n);
    for (int i = 0; i < getNumBodyNodes(); i++)
    {
        BodyNode *nodei = getBodyNode(i);
        nodei->updateMassMatrix();
        nodei->aggregateMass(mM);
    }

    // Inverse of mass matrix
    mMInv = mM.ldlt().solve(Eigen::MatrixXd::Identity(n,n));

    // Restore the torque
    set_tau(tau_old);

    // Evaluate external forces in generalized coordinate.
    updateExternalForces();

    // Update damping forces
    updateDampingForces();
}

void Skeleton::computeForwardDynamicsID(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
    Eigen::VectorXd qddot = this->getInvMassMatrix()
                            * (-mCg
                               + mFext
                               + this->getInternalForces()
                               + mDampingForce
                               + mFc );

    this->set_ddq(qddot);
}

void Skeleton::computeForwardDynamicsFS(
        const Eigen::Vector3d& _gravity, bool _equationsOfMotion)
{
    // Skip immobile or 0-dof skeleton
    if (!isMobile() == true || getNumGenCoords() == 0)
        return;

    // Backward recursion
    for (std::vector<dynamics::BodyNode*>::reverse_iterator ritrBody
         = mBodyNodes.rbegin();
         ritrBody != mBodyNodes.rend();
         ++ritrBody)
    {
        (*ritrBody)->updateArticulatedInertia();
        (*ritrBody)->updateBiasForce(_gravity);
        (*ritrBody)->updatePsi();
        (*ritrBody)->updatePi();
        (*ritrBody)->updateEta();
        (*ritrBody)->updateBeta();
    }

    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody)
    {
        (*itrBody)->update_ddq();
        (*itrBody)->updateAcceleration();
        (*itrBody)->update_F_fs();
    }
}

Eigen::VectorXd Skeleton::getDampingForces() const
{
    return mDampingForce;
}

Eigen::VectorXd Skeleton::getConstraintForces() const
{
    return mFc;
}

void Skeleton::setInternalForces(const Eigen::VectorXd& _forces)
{
    set_tau(_forces);
}

void Skeleton::setMinInternalForces(Eigen::VectorXd _minForces)
{
    set_tauMin(_minForces);
}

Eigen::VectorXd Skeleton::getMinInternalForces() const
{
    return get_tauMin();
}

void Skeleton::setMaxInternalForces(Eigen::VectorXd _maxForces)
{
    set_tauMax(_maxForces);
}

Eigen::VectorXd Skeleton::getMaxInternalForces() const
{
    return get_tauMax();
}

void Skeleton::clearInternalForces()
{
    set_tau(Eigen::VectorXd::Zero(getNumGenCoords()));
}

void Skeleton::setConstraintForces(const Eigen::VectorXd& _Fc)
{
    mFc = _Fc;
}

Eigen::Vector3d Skeleton::getWorldCOM()
{
    Eigen::Vector3d com(0, 0, 0);

    assert(mTotalMass != 0);
    const int nNodes = getNumBodyNodes();

    for(int i = 0; i < nNodes; i++)
    {
        BodyNode* bodyNode = getBodyNode(i);
        com += bodyNode->getMass() *
               (bodyNode->getWorldTransform() * bodyNode->getLocalCOM());
    }

    return com / mTotalMass;
}

} // namespace dynamics
} // namespace dart
