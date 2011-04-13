#ifndef SRC_MODEL3D_PRIMITIVE_H
#define SRC_MODEL3D_PRIMITIVE_H

#include <Eigen/Dense>
using namespace Eigen;

namespace model3d {
  class Transformation;

  class Primitive{
  public:
    Primitive();
    virtual void draw(Vec4d _color, bool _default);
    virtual bool isInside(Vec3d _pt);	// decides if the point is inside
    virtual Vec3d getNormal(Vec3d& _pt); //in world coordinate
	
    inline void setInertia(Mat3d& _inertia);
    Mat3d getInertia();
    inline Mat4d getMassTensor();

    inline void setColor(Vec3d _color);
    inline Vec3d getColor();

    inline void setWorldPos(Vec3d _pos);
    inline Vec3d getCOM(); //in local coordinate
    inline Vec3d getWorldCOM(); //in world coordinatei;
                                // assume the world position is up to date
	
    inline void setDim(Vec3d _dim);
    inline Vec3d getDim();
	
    inline void setMass(double _m);
    inline double getMass();

    inline void setVolume(double _v);
    inline double getVolume();
	
    inline int getID();
  protected:
    void setMassTensorFromInertia();
    void calInertiaFromMassTensor();
    virtual void calInertia();
    virtual void calMassTensor();
    virtual void calVolume();
    virtual void calCOM();

    Vec3d mDim; // dimensions for bounding box
    Mat3d mInertia;	// inertia
    Mat4d mMassTensor; // mass tensor for lagrangian dynamics
    double mMass;	// mass
    double mVolume; // volume

    Vec3d mCOM;	// COM in local coordinate; default (0,0,0)
    Vec3d mWorldPos; // local origin in world coordinate
    Vec3d mLinearVel;
    Vec3d mAngVel;
	
    Vec3d mColor;		// color for the primitive
    int mID; // unique id

    static int mCounter;
  };

} // namespace model3d

#endif
