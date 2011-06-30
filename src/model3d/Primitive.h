/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_PRIMITIVE_H
#define MODEL3D_PRIMITIVE_H

#include <Eigen/Dense>

#include "renderer/RenderInterface.h"


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

        void setWorldPos(const Eigen::Vector3d& _pos) { mWorldPos = _pos; }
        Eigen::Vector3d getCOM() { return mCOM; } //in local coordinates
        Eigen::Vector3d getWorldCOM() { return (mWorldPos + mCOM); } //in world coordinates;
        // assume the world position is up to date
	
        void setDim(const Eigen::Vector3d& _dim);
        Eigen::Vector3d getDim() { return mDim; }
	
        void setMass(double _m);
        double getMass() { return mMass; }

        void setVolume(double _v) { mVolume = _v; }
        double getVolume() { return mVolume; }
	
        int getID() { return mID; }

		virtual void draw(Renderer::RenderInterface* RI = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const {}

    protected:
        void setMassTensorFromInertia();
        void calInertiaFromMassTensor();
        virtual void calInertia() {}
        virtual void calMassTensor() {}
        virtual void calVolume() {}
        virtual void calCOM() {}

        Eigen::Vector3d mDim; // dimensions for bounding box
        double mMass;	// mass
        double mVolume; // volume

        Eigen::Matrix3d mInertia;	// inertia
        Eigen::Matrix4d mMassTensor; // mass tensor for lagrangian dynamics
    
        Eigen::Vector3d mCOM;	// COM in local coordinate; default (0,0,0)
        Eigen::Vector3d mWorldPos; // local origin in world coordinate
        Eigen::Vector3d mLinearVel;
        Eigen::Vector3d mAngVel;
	
        int mID; // unique id
        Eigen::Vector3d mColor;		// color for the primitive

        static int mCounter;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

} // namespace model3d

#endif // #ifndef MODEL3D_PRIMITIVE_H

