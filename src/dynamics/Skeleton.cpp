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
      mSelfCollidable(false),
      mTotalMass(0.0),
      mImmobile(false),
      mJointLimit(true)
{
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

void Skeleton::setSelfCollidable(bool _selfCollidable)
{
    mSelfCollidable = _selfCollidable;
}

bool Skeleton::getSelfCollidable() const
{
    return mSelfCollidable;
}

void Skeleton::setImmobileState(bool _immobile)
{
    mImmobile = _immobile;
}

bool Skeleton::getImmobileState() const
{
    return mImmobile;
}

bool Skeleton::getJointLimitState() const
{
    return mJointLimit;
}

void Skeleton::setJointLimitState(bool _s)
{
    mJointLimit = _s;
}

double Skeleton::getMass() const
{
    return mTotalMass;
}

void Skeleton::initDynamics()
{
    initKinematics();

    int DOF = getDOF();

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

void Skeleton::addBodyNode(BodyNode* _body, bool _addParentJoint)
{
    assert(_body != NULL);

    mBodyNodes.push_back(_body);
    _body->setSkeletonIndex(mBodyNodes.size() - 1);

    // The parent joint possibly be null
    if (_addParentJoint)
        addJoint(_body->getParentJoint());
}

void Skeleton::addJoint(Joint* _joint)
{
    assert(_joint);

    mJoints.push_back(_joint);
    _joint->setSkeletonIndex(mJoints.size() - 1);

    const std::vector<GenCoord*>& dofs = _joint->getGenCoords();
    for (std::vector<GenCoord*>::const_iterator itrDof = dofs.begin();
         itrDof != dofs.end();
         ++itrDof)
    {
        mGenCoords.push_back((*itrDof));
        (*itrDof)->setSkeletonIndex(mGenCoords.size() - 1);
    }
}

int Skeleton::getNumBodyNodes() const
{
    return mBodyNodes.size();
}

int Skeleton::getNumJoints() const
{
    return mJoints.size();
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

int Skeleton::getBodyNodeIndex(const std::string& _name) const
{
    const int nNodes = getNumBodyNodes();

    for(int i = 0; i < nNodes; i++)
    {
        BodyNode* node = getBodyNode(i);

        if (_name == node->getName())
            return i;
    }

    return -1;
}

Joint* Skeleton::getJoint(int _idx) const
{
    return mJoints[_idx];
}

Joint* Skeleton::getJoint(const std::string& _name) const
{
    assert(!_name.empty());

    for (std::vector<Joint*>::const_iterator itrJoint = mJoints.begin();
         itrJoint != mJoints.end();
         ++itrJoint)
    {
        if ((*itrJoint)->getName() == _name)
            return *itrJoint;
    }

    return NULL;
}

int Skeleton::getJointIndex(const std::string& _name) const
{
    const int nJoints = getNumJoints();

    for(int i = 0; i < nJoints; i++)
    {
        Joint* node = getJoint(i);

        if (_name == node->getName())
            return i;
    }

    return -1;
}

void Skeleton::addMarker(Marker* _h)
{
    mMarkers.push_back(_h);
    _h->setSkeletonIndex(mMarkers.size()-1);
    BodyNode *body = _h->getNode();
    body->addMarker(_h);
}

int Skeleton::getNumMarkers() const
{
    return mMarkers.size();
}

Marker* Skeleton::getMarker(int _i)
{
    return mMarkers[_i];
}

Marker*Skeleton::getMarker(const std::string& _name) const
{
    assert(!_name.empty());

    for (std::vector<Marker*>::const_iterator itrMarker = mMarkers.begin();
         itrMarker != mMarkers.end();
         ++itrMarker)
    {
        if ((*itrMarker)->getName() == _name)
            return *itrMarker;
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

void Skeleton::setConfig(const std::vector<int>& _id, Eigen::VectorXd _vals,
                         bool _calcTrans, bool _calcDeriv)
{
    for( unsigned int i = 0; i < _id.size(); i++ )
        mGenCoords[_id[i]]->set_q(_vals(i));

    if (_calcTrans)
    {
        if (_calcDeriv)
            updateForwardKinematics(true, false);
        else
            updateForwardKinematics(false, false);
    }
}

void Skeleton::setConfig(const Eigen::VectorXd& _pose,
                       bool bCalcTrans,
                       bool bCalcDeriv)
{
    for (int i = 0; i < getDOF(); i++)
        mGenCoords.at(i)->set_q(_pose[i]);

    if (bCalcTrans)
    {
        if (bCalcDeriv)
            updateForwardKinematics(true, false);
        else
            updateForwardKinematics(false, false);
    }
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

void Skeleton::initKinematics()
{
    // init the dependsOnDof stucture for each bodylink
    for(int i = 0; i < getNumBodyNodes(); i++)
    {
        mBodyNodes.at(i)->setSkeleton(this);
        mBodyNodes.at(i)->setDependDofList();
        mBodyNodes.at(i)->init();
    }

    updateForwardKinematics();
}

void Skeleton::updateForwardKinematics(bool _firstDerivative,
                                       bool _secondDerivative)
{
    _updateJointKinematics(_firstDerivative, _secondDerivative);
    _updateBodyForwardKinematics(_firstDerivative, _secondDerivative);
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

void Skeleton::_updateJointKinematics(bool _firstDerivative,
                                      bool _secondDerivative)
{
    for (std::vector<Joint*>::iterator itrJoint = mJoints.begin();
         itrJoint != mJoints.end(); ++itrJoint)
    {
        (*itrJoint)->updateKinematics(_firstDerivative,
                                      _secondDerivative);
    }
}

void Skeleton::_updateBodyForwardKinematics(bool _firstDerivative,
                                            bool _secondDerivative)
{
    for (std::vector<BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end(); ++itrBody)
    {
        (*itrBody)->updateTransform();

        if (_firstDerivative)
            (*itrBody)->updateVelocity();

        if (_secondDerivative)
        {
            (*itrBody)->updateEta();
            (*itrBody)->updateAcceleration();
        }
    }
}

void Skeleton::computeInverseDynamicsLinear(const Eigen::Vector3d& _gravity,
                                      bool _computeJacobian,
                                      bool _computeJacobianDeriv,
                                      bool _withExternalForces,
                                      bool _withDampingForces)
{
    _updateJointKinematics();

    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody) {
        (*itrBody)->updateTransform();
        (*itrBody)->updateVelocity(_computeJacobian);
        (*itrBody)->updateEta();
        (*itrBody)->updateAcceleration(_computeJacobianDeriv);
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

Eigen::VectorXd Skeleton::computeInverseDynamicsLinear(
        const Eigen::Vector3d& _gravity,
        const Eigen::VectorXd* _qdot,
        const Eigen::VectorXd* _qdotdot,
        bool _computeJacobians,
        bool _withExternalForces,
        bool _withDampingForces)
{
    int n = getDOF();

    if (_qdot == NULL)
        set_dq(Eigen::VectorXd::Zero(n));

    if (_qdotdot == NULL)
        set_ddq(Eigen::VectorXd::Zero(n));

    _updateJointKinematics();

    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody) {
        (*itrBody)->updateTransform();
        (*itrBody)->updateVelocity(_computeJacobians);
        (*itrBody)->updateEta();
        (*itrBody)->updateAcceleration(_computeJacobians);
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

    return get_tau();
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

    for (std::vector<Joint*>::iterator itr = mJoints.begin();
         itr != mJoints.end(); ++itr)
    {
        Eigen::VectorXd jointDampingForce = (*itr)->getDampingForces();
        for (int i = 0; i < jointDampingForce.size(); i++)
        {
            mDampingForce((*itr)->getGenCoord(i)->getSkeletonIndex()) =
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
    int n = getDOF();

    // skip immobile objects in forward simulation
    if (getImmobileState() == true || n == 0)
    {
        return;
    }

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
    int n = getDOF();

    // skip immobile objects in forward simulation
    if (getImmobileState() == true || n == 0)
    {
        return;
    }

    // Forward recursion
    for (std::vector<dynamics::BodyNode*>::iterator itrBody = mBodyNodes.begin();
         itrBody != mBodyNodes.end();
         ++itrBody)
    {
        (*itrBody)->updateTransform();
        (*itrBody)->updateVelocity();
        (*itrBody)->updateEta();
    }

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
        (*ritrBody)->updateBeta();
    }

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
    set_tau(Eigen::VectorXd::Zero(getDOF()));
}

void Skeleton::setConstraintForces(const Eigen::VectorXd& _Fc)
{
    mFc = _Fc;
}

double Skeleton::getKineticEnergy() const
{
    double KineticEnergy = 0.0;

    for (int i = 0; i < mBodyNodes.size(); i++)
        KineticEnergy += mBodyNodes[i]->getKineticEnergy();

    return 0.5 * KineticEnergy;
}

double Skeleton::getPotentialEnergy() const
{
    double potentialEnergy = 0.0;

    //// Gravity and Springs on bodies
    //for (int i = 0; i < mBodies.size(); i++)
    //    potentialEnergy += mBodies[i]->getPotentialEnergy();

    // Springs on joints
    for (int i = 0; i < mJoints.size(); i++)
        potentialEnergy += mJoints[i]->getPotentialEnergy();

    return potentialEnergy;
}

Eigen::Vector3d Skeleton::getWorldCOM()
{
    Eigen::Vector3d com(0, 0, 0);

    assert(mTotalMass != 0);
    const int nNodes = getNumBodyNodes();

    for(int i = 0; i < nNodes; i++)
    {
        BodyNode* bodyNode = getBodyNode(i);
        com += (bodyNode->getMass() * bodyNode->getWorldCOM());
    }

    return com / mTotalMass;
}

} // namespace dynamics
} // namespace dart
