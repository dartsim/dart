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

#ifndef KINEMATICS_SHAPE_H
#define KINEMATICS_SHAPE_H

#include <Eigen/Dense>

namespace renderer {
    class RenderInterface;
}

extern "C" { struct aiScene; }

namespace kinematics {
    class Transformation;

    class Shape {
    public:
        enum ShapeType {
            P_UNDEFINED,
            P_BOX,
            P_ELLIPSOID,
            P_SPHERE,
            P_CYLINDER,
            P_MESH
        };

        Shape(ShapeType _type = P_UNDEFINED, double _mass = 0.0);
        virtual ~Shape() {};
        
        void setInertia(const Eigen::Matrix3d& _inertia);
        inline Eigen::Matrix3d getInertia() const { return mInertia; }
        inline Eigen::Matrix4d getMassTensor() const { return mMassTensor; }

        inline void setColor(const Eigen::Vector3d& _color) { mColor = _color; }
        inline Eigen::Vector3d getColor() const { return mColor; }

        void setDim(const Eigen::Vector3d& _dim);
        inline Eigen::Vector3d getDim() const { return mDim; }

				inline void setOffset(Eigen::Vector3d _offset) { mOffset = _offset; }
        inline Eigen::Vector3d getOffset() const { return mOffset; }

        void setMass(const double _m);
        inline double getMass() { return mMass; }

        inline void setVolume(double _v) { mVolume = _v; }
        inline double getVolume() const { return mVolume; }

        inline int getID() const { return mID; }
        inline ShapeType getShapeType() const { return mType; }

        inline const aiScene* getVizMesh() const { return mVizMesh; }
        inline void setVizMesh(const aiScene *_mesh) { mVizMesh = _mesh; } 

        inline const aiScene* getCollisionMesh() const { return mCollisionMesh; }
        inline void setCollisionMesh(const aiScene *_mesh) { mCollisionMesh = _mesh; } 

        virtual void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const {}

    protected:
        void setMassTensorFromInertia();    ///< sets the "mass tensor" in lagrangian dynamics from the inertia matrix
        void computeInertiaFromMassTensor();    ///< computes the inertia matrix from the "mass tensor" in lagrangian dynamics
        virtual void computeMassTensor() {}
        inline virtual void computeInertia() {
            computeMassTensor();
            computeInertiaFromMassTensor();
        }
        virtual void computeVolume() {}

        virtual void initMeshes() {}

        ShapeType mType;    ///< Type of primitive; unknown in the general case
        Eigen::Vector3d mDim; ///< dimensions for bounding box
        double mMass;	///< mass of the body
        double mVolume; ///< volume enclosed by the geometry

        Eigen::Matrix3d mInertia;	///< inertia matrix
        Eigen::Matrix4d mMassTensor; ///< homogenous mass tensor for lagrangian dynamics

        int mID; // unique id
        Eigen::Vector3d mColor;		///< color for the primitive
        Eigen::Vector3d mOffset; ///< the origin of this primitive in the bodynode frame>

        const aiScene *mVizMesh; ///< mesh for visualization>
        const aiScene *mCollisionMesh; ///< mesh for collision detection>

        static int mCounter;
    public:

        unsigned int listIndex;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_PRIMITIVE_H

