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

#ifndef DART_KINEMATICS_MARKER_H
#define DART_KINEMATICS_MARKER_H

#include <Eigen/Dense>

namespace renderer { class RenderInterface; };

namespace kinematics {
#define MAX_MARKER_NAME 256

    class Dof;
    class BodyNode;
    class ShapeEllipsoid;

    class Marker {
    public:
        enum ConstraintType {
            NO,
            HARD,
            SOFT
        };
    
    public:
        Marker(const char* _name, Eigen::Vector3d& , BodyNode*, ConstraintType _type = NO);
        virtual ~Marker(){}

		void draw(renderer::RenderInterface* _ri = NULL, bool _offset = true, const Eigen::Vector4d& _color = Eigen::Vector4d::Identity(), bool _useDefaultColor = true) const;

        Eigen::Vector3d getWorldCoords(); ///< get the world coordinates of mOffset
	
        inline Eigen::Vector3d getLocalCoords() const {return mOffset;}
        inline void setLocalCoords(Eigen::Vector3d& _offset){mOffset = _offset;}

        inline int getSkelIndex() const{return mSkelIndex;}
        inline void setSkelIndex(int _idx){mSkelIndex=_idx;}
        
        inline int getID() const {return mID;}
        inline BodyNode* getNode() const {return mNode;}
        inline const char* getName() const {return mName;}
        
        // useful for IK
        inline ConstraintType getConstraintType() const {return mType;}
        inline void setConstraintType(ConstraintType _type){mType = _type;}
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    protected:
        BodyNode* mNode;	///< body link associated with
        Eigen::Vector3d mOffset;	///< local coordinates in the links
        char mName[MAX_MARKER_NAME]; ///< name of this marker, max length 256 characters
        int mSkelIndex;	///< position in the model class marker vector
        ConstraintType mType; ///< type of constraint
    
    private:
        int mID; ///< a unique ID of this marker globally
        static int msMarkerCount; ///< counts the number of markers globally
    };

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_MARKER_H

