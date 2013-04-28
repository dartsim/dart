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

#ifndef KINEMATICS_SKELETON_H
#define KINEMATICS_SKELETON_H

#include <vector>
#include <Eigen/Dense>

namespace renderer { class RenderInterface; };

namespace kinematics {

    class Transformation;
    class Marker;
    class Joint;
    class BodyNode;
    class Dof;

    class Skeleton {
    public:
        Eigen::VectorXd mCurrPose; 
        BodyNode* mRoot;


        Skeleton();
        virtual ~Skeleton();

        virtual BodyNode* createBodyNode(const char* const name = NULL);
        void addMarker(Marker *_h);
        void addNode(BodyNode *_b, bool _addParentJoint = true);
        void addJoint(Joint *_j);
        void addDof(Dof *_d);
        void addTransform(Transformation *_t);
	
        // init the model after parsing
        void initSkel();
	
        // inline access functions
        inline int getNumDofs() { return mDofs.size(); }
        inline int getNumNodes() { return mNodes.size(); }
        inline int getNumMarkers() { return mMarkers.size(); }
        inline int getNumJoints(){return mJoints.size();}
        inline Dof* getDof(int _i) { return mDofs[_i]; }
        inline BodyNode* getNode(int _i) { return mNodes[_i]; }
        inline BodyNode* getRoot() { return mRoot; }
        BodyNode* getNode(const char* const name);
        int getNodeIndex(const char* const name);
        inline Marker* getMarker(int _i) { return mMarkers[_i]; }
        inline double getMass() { return mMass; }
        Eigen::Vector3d getWorldCOM();
        inline std::string getName() { return mName; }
        inline void setName( std::string _name ) { mName = _name; }
        Eigen::VectorXd getPose();
        virtual void setPose(const Eigen::VectorXd&, bool bCalcTrans = true, bool bCalcDeriv = true);
        Eigen::VectorXd getConfig(std::vector<int> _id);
        void setConfig(std::vector<int> _id, Eigen::VectorXd _vals, bool _calcTrans = true, bool _calcDeriv = true);
        Eigen::MatrixXd getJacobian(BodyNode* _bd, Eigen::Vector3d& _localOffset);


        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const;
        void drawMarkers(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true ) const;

        /// @brief Find body node by name.
        /// @param[in] _name The name of body node looking for.
        /// @return Searched body node. If the skeleton does not have a body
        /// node with _name, then return NULL.
        BodyNode* getBodyNode(const char* const _name) const;

        /// @brief Find joint by name.
        /// @param[in] _name The name of joint looking for.
        /// @return Searched joint. If the skeleton does not have a joint with
        /// _name, then return NULL.
        Joint* getJoint(const char* const _name) const;

    protected:
        std::string mName;
        std::vector<Marker*> mMarkers;
        std::vector<Dof*> mDofs;
        std::vector<Transformation*> mTransforms;
        std::vector<BodyNode*> mNodes;
        std::vector<Joint*> mJoints;
        double mMass;
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_SKELETON_H

