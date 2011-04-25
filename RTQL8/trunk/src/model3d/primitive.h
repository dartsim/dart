#ifndef SRC_MODEL3D_PRIMITIVE_H
#define SRC_MODEL3D_PRIMITIVE_H

#include <Eigen/Dense>
using namespace Eigen;

namespace model3d {
  class Transformation;

  class Primitive{
  public:
    Primitive();
    virtual void draw(Vector4d& _color, bool _default);
    virtual bool isInside(Vector3d _pt);	// decides if the point is inside
    virtual Vector3d getNormal(Vector3d& _pt); //in world coordinate
	
    inline void setInertia(Matrix3d& _inertia) {
        mInertia = _inertia;
        setMassTensorFromInertia();
    }
    Matrix3d getInertia() { return mInertia; }
    inline Matrix4d getMassTensor() { return mMassTensor; }

    inline void setColor(Vector3d _color) { mColor = _color; }
    inline Vector3d getColor() { return mColor; }

    inline void setWorldPos(Vector3d _pos) { mWorldPos = _pos; }
    inline Vector3d getCOM() { return mCOM; } //in local coordinates
    inline Vector3d getWorldCOM() { return (mWorldPos + mCOM); } //in world coordinates;
                                // assume the world position is up to date
	
    inline void setDim(Vector3d _dim) {
        mDim = _dim;
        calVolume();
        calMassTensor();
        calInertiaFromMassTensor();
    }
    inline Vector3d getDim() { return mDim; }
	
    inline void setMass(double _m) {
        mMass = _m;
        calMassTensor();
    }
    inline double getMass() { return mMass; }

    inline void setVolume(double _v) { mVolume = _v; }
    inline double getVolume() { return mVolume; }
	
    inline int getID() { return mID; }

  protected:
    void setMassTensorFromInertia();
    void calInertiaFromMassTensor();
    virtual void calInertia();
    virtual void calMassTensor();
    virtual void calVolume();
    virtual void calCOM();

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
