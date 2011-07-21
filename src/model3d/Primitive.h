/*
RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
All rights reserved.

Author	Sehoon Ha
Date		06/12/2011
*/

#ifndef MODEL3D_PRIMITIVE_H
#define MODEL3D_PRIMITIVE_H

#include <Eigen/Dense>

namespace renderer{
    class RenderInterface;
}

namespace model3d {
    class Transformation;

    class Primitive {
    public:
        Primitive();

        virtual bool isInside(Eigen::Vector3d& _pt) { return false; }
        virtual Eigen::Vector3d getNormal(Eigen::Vector3d& _pt); 

        void setInertia(const Eigen::Matrix3d& _inertia);
        Eigen::Matrix3d getInertia() const { return mInertia; }
        Eigen::Matrix4d getMassTensor() const { return mMassTensor; }

        void setColor(const Eigen::Vector3d& _color) { mColor = _color; }
        Eigen::Vector3d getColor() const { return mColor; }

        void setWorldPos(const Eigen::Vector3d& _pos) { mPosWorld = _pos; }
        Eigen::Vector3d getCOM() { return mCOMLocal; } //in local coordinates

        void setDim(const Eigen::Vector3d& _dim);
        Eigen::Vector3d getDim() { return mDim; }

        void setMass(double _m);
        double getMass() { return mMass; }

        void setVolume(double _v) { mVolume = _v; }
        double getVolume() { return mVolume; }

        int getID() { return mID; }

        virtual void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const {}

    protected:
        void setMassTensorFromInertia();    ///< sets the "mass tensor" in lagrangian dynamics from the inertia matrix
        void computeInertiaFromMassTensor();    ///< computes the inertia matrix from the "mass tensor" in lagrangian dynamics
        virtual void computeInertia() {}
        virtual void computeMassTensor() {}
        virtual void computeVolume() {}
        virtual void computeCOM() {}

        Eigen::Vector3d mDim; ///< dimensions for bounding box
        double mMass;	///< mass of the body
        double mVolume; ///< volume enclosed by the geometry

        Eigen::Matrix3d mInertia;	///< inertia matrix
        Eigen::Matrix4d mMassTensor; ///< homogenous mass tensor for lagrangian dynamics

        Eigen::Vector3d mCOMLocal;	///< COM in local coordinate; default (0,0,0)
        Eigen::Vector3d mPosWorld;  ///< local origin in world coordinate
        Eigen::Vector3d mLinearVel; ///< Linear velocity of the COM in the world coodinates
        Eigen::Vector3d mAngVel;    ///< Angular velocity of the body in the world coordinates

        int mID; // unique id
        Eigen::Vector3d mColor;		///< color for the primitive

        static int mCounter;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace model3d

#endif // #ifndef MODEL3D_PRIMITIVE_H

