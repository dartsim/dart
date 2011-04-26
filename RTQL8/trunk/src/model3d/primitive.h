#ifndef SRC_MODEL3D_PRIMITIVE_H
#define SRC_MODEL3D_PRIMITIVE_H

#include <Eigen/Dense>
using namespace Eigen;

namespace model3d {
  class Transformation;

  class Primitive {
  public:
    Primitive();
    /* virtual void draw(Vector4d& _color, bool _default); */
    virtual bool isInside(Vector3d _pt) { return false;	}
    virtual Vector3d getNormal(Vector3d& _pt); 
	
    void setInertia(Matrix3d& _inertia) {
        mInertia = _inertia;
        setMassTensorFromInertia();
    }
    Matrix3d getInertia() { return mInertia; }
    Matrix4d getMassTensor() { return mMassTensor; }

    void setColor(Vector3d _color) { mColor = _color; }
    Vector3d getColor() { return mColor; }

    void setWorldPos(Vector3d _pos) { mWorldPos = _pos; }
    Vector3d getCOM() { return mCOM; } //in local coordinates
    Vector3d getWorldCOM() { return (mWorldPos + mCOM); } //in world coordinates;
                                // assume the world position is up to date
	
    void setDim(Vector3d _dim) {
        mDim = _dim;
        calVolume();
        calMassTensor();
        calInertiaFromMassTensor();
    }
    Vector3d getDim() { return mDim; }
	
    void setMass(double _m) {
        mMass = _m;
        calMassTensor();
    }
    double getMass() { return mMass; }

    void setVolume(double _v) { mVolume = _v; }
    double getVolume() { return mVolume; }
	
    int getID() { return mID; }

  protected:
    void setMassTensorFromInertia();
    void calInertiaFromMassTensor();
    virtual void calInertia() {}
    virtual void calMassTensor() {}
    virtual void calVolume() {}
    virtual void calCOM() {}

    Vector3d mDim; // dimensions for bounding box
    Matrix3d mInertia;	// inertia
    Matrix4d mMassTensor; // mass tensor for lagrangian dynamics
    double mMass;	// mass
    double mVolume; // volume

    Vector3d mCOM;	// COM in local coordinate; default (0,0,0)
    Vector3d mWorldPos; // local origin in world coordinate
    Vector3d mLinearVel;
    Vector3d mAngVel;
	
    Vector3d mColor;		// color for the primitive
    int mID; // unique id

    static int mCounter;
  };

} // namespace model3d

#endif
