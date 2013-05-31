/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
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

#ifndef DART_KINEMATICS_SKELETON_H
#define DART_KINEMATICS_SKELETON_H

#include <vector>
#include <Eigen/Dense>

#include "kinematics/System.h"

namespace renderer { class RenderInterface; }

namespace kinematics
{

class Transformation;
class Marker;
class Joint;
class BodyNode;
class Dof;

class Skeleton : public System
{
public:


public:
    /// @brief
    Skeleton();

    /// @brief
    virtual ~Skeleton();

    /// @brief
    virtual BodyNode* createBodyNode(const char* const name = NULL);

    /// @brief
    void addMarker(Marker *_h);

    /// @brief
    void addNode(BodyNode *_b, bool _addParentJoint = true);

    /// @brief
    void addJoint(Joint *_j);

    /// @brief
    void addDof(Dof *_d);

    /// @brief
    void addTransform(Transformation *_t);

    /// @brief Init the model after parsing.
    void initSkel();

    /// @brief
    int getNumNodes() const { return mNodes.size(); }

    /// @brief
    int getNumMarkers() const { return mMarkers.size(); }

    /// @brief
    int getNumJoints() const { return mJoints.size();}

    /// @brief
    BodyNode* getNode(int _i) const { return mNodes[_i]; }

    /// @brief
    BodyNode* getRoot() { return mRoot; }

    /// @brief
    BodyNode* getNode(const char* const _name) const;

    /// @brief
    int getNodeIndex(const char* const _name) const;

    /// @brief
    Joint* getJoint(int _i) const { return mJoints[_i]; }

    /// @brief
    Joint* getJoint(const char* const _name) const;

    /// @brief
    int getJointIndex(const char* const _name) const;

    /// @brief
    Marker* getMarker(int _i) { return mMarkers[_i]; }

    /// @brief
    double getMass() { return mMass; }

    /// @brief
    Eigen::Vector3d getWorldCOM();

    /// @brief
    std::string getName() { return mName; }

    /// @brief
    void setName( std::string _name ) { mName = _name; }

    /// @brief
    virtual void setPose(const Eigen::VectorXd&,
                         bool bCalcTrans = true,
                         bool bCalcDeriv = true);

    /// @brief
    Eigen::VectorXd getConfig(std::vector<int> _id);

    /// @brief
    void setConfig(std::vector<int> _id,
                   Eigen::VectorXd _vals,
                   bool _calcTrans = true,
                   bool _calcDeriv = true);

    /// @brief
    Eigen::MatrixXd getJacobian(BodyNode* _bd, Eigen::Vector3d& _localOffset);

    /// @brief
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true) const;

    /// @brief
    void drawMarkers(renderer::RenderInterface* _ri = NULL,
                     const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                     bool _useDefaultColor = true ) const;

    /// @brief
    void setSelfCollidable(bool _selfCollidable)
    { mSelfCollidable = _selfCollidable; }

    /// @brief
    bool getSelfCollidable() const { return mSelfCollidable; }

    /// @brief
    Eigen::VectorXd getPose();

    /// @brief
    Eigen::VectorXd getPoseVelocity();

protected:
    /// @brief
    std::string mName;

    /// @brief
    std::vector<Marker*> mMarkers;

    /// @brief
    std::vector<Transformation*> mTransforms;

    /// @brief
    std::vector<BodyNode*> mNodes;

    /// @brief
    std::vector<Joint*> mJoints;

    /// @brief
    double mMass;

    /// @brief
    bool mSelfCollidable;

    /// @brief
    BodyNode* mRoot;
};

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_SKELETON_H

