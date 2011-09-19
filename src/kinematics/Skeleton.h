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
#include "renderer/RenderInterface.h"

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
        void addMarker(Marker *h);
        void addNode(BodyNode *b);
        void addJoint(Joint *_j);
        void addDof(Dof *d);
        void addTransform(Transformation *t);
	
        // init the model after parsing
        void initSkel();
	
        // inline access functions
        inline int getNumDofs() { return mDofs.size(); }
        inline int getNumNodes() { return mNodes.size(); }
        inline int getNumMarkers() { return mMarkers.size(); }
        inline int getNumJoints(){return mJoints.size();}
        inline Dof* getDof(int i) { return mDofs[i]; }
        inline BodyNode* getNode(int i) { return mNodes[i]; }
        inline BodyNode* getRoot() { return mRoot; }
        BodyNode* getNode(const char* const name);
        int getNodeIndex(const char* const name);
        inline Marker* getMarker(int i) { return mMarkers[i]; }
        inline double getMass() { return mMass; }
        Eigen::Vector3d getWorldCOM();

        virtual void setPose(const Eigen::VectorXd&, bool bCalcTrans = true, bool bCalcDeriv = true);
        virtual void setPose(const std::vector<double>&, bool bCalcTrans = true, bool bCalcDeriv = true);

        void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const;
        void drawMarkers(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true ) const;
	
        //void setPose(const Eigen::VectorXd& _pose);
        //void setPose(const std::vector<double>& _pose);
        void getPose(Eigen::VectorXd& _pose);
        void getPose(std::vector<double>& _pose);

    protected:
        std::vector<Marker*> mMarkers;
        std::vector<Dof*> mDofs;
        std::vector<Transformation*> mTransforms;
        std::vector<BodyNode*> mNodes;
        std::vector<Joint*> mJoints;

        double mMass;
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_SKELETON_H

