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
        enum PrimitiveType {
            P_UNDEFINED,
            P_CUBE,
            P_ELLIPSOID
        };

        Primitive();

        inline virtual bool isInside(Eigen::Vector3d& _pt) { return false; }
        inline virtual Eigen::Vector3d getNormal(Eigen::Vector3d& _pt){ return Eigen::Vector3d::Zero(); }

        void setInertia(const Eigen::Matrix3d& _inertia);
        inline Eigen::Matrix3d getInertia() const { return mInertia; }
        inline Eigen::Matrix4d getMassTensor() const { return mMassTensor; }

        inline void setColor(const Eigen::Vector3d& _color) { mColor = _color; }
        inline Eigen::Vector3d getColor() const { return mColor; }

        //Eigen::Vector3d getOffset() { return mOffset; } //in local coordinates

        void setDim(const Eigen::Vector3d& _dim);
        inline Eigen::Vector3d getDim() { return mDim; }

        void setMass(double _m);
        inline double getMass() { return mMass; }

        inline void setVolume(double _v) { mVolume = _v; }
        inline double getVolume() { return mVolume; }

        inline int getID() { return mID; }
        inline PrimitiveType getPrimitiveType(){return mType;}

        virtual void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const {}

    protected:
        void setMassTensorFromInertia();    ///< sets the "mass tensor" in lagrangian dynamics from the inertia matrix
        void computeInertiaFromMassTensor();    ///< computes the inertia matrix from the "mass tensor" in lagrangian dynamics
        virtual void computeMassTensor()=0;
        inline virtual void computeInertia() {
            computeMassTensor();
            computeInertiaFromMassTensor();
        }
        virtual void computeVolume() {}

        PrimitiveType mType;    ///< Type of primitive; unknown in the general case
        Eigen::Vector3d mDim; ///< dimensions for bounding box
        double mMass;	///< mass of the body
        double mVolume; ///< volume enclosed by the geometry

        Eigen::Matrix3d mInertia;	///< inertia matrix
        Eigen::Matrix4d mMassTensor; ///< homogenous mass tensor for lagrangian dynamics
        //Eigen::Vector3d mOffset;	///< Offset to draw if needed; default (0,0,0)

        int mID; // unique id
        Eigen::Vector3d mColor;		///< color for the primitive

        static int mCounter;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace model3d

#endif // #ifndef MODEL3D_PRIMITIVE_H

