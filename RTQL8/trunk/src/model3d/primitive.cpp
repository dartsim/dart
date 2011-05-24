#include "primitive.h"

#define PRIMITIVE_MAGIC_NUMBER 1000

namespace model3d {

  // initialize in the same order as declaration
  Primitive::Primitive()
    : mDim(0,0,0), mMass(0), mVolume(0),
      mInertia(Matrix3d::Zero()),
      mMassTensor(Matrix4d::Zero()), mCOM(0,0,0),
      mID(mCounter++), mColor(0.5,0.5,1.0) {
  }


  Vector3d Primitive::getNormal(Vector3d& _pt) {
    return Vector3d::Zero();
  }

  int Primitive::mCounter = PRIMITIVE_MAGIC_NUMBER;

  void Primitive::setMassTensorFromInertia()
  {
    // compute the half trace of mat = row*integral((x*x+y*y+z*z)dxdydz) = sum of moment of inertia along all 3 axes
	
    double halftrace = 0.5* mInertia.trace();
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        mMassTensor(i, j) = -mInertia(i, j);
      }
    }
    mMassTensor(0, 0) += halftrace;
    mMassTensor(1, 1) += halftrace;
    mMassTensor(2, 2) += halftrace;
    mMassTensor(3, 3) = mMass;
  }

  void Primitive::calInertiaFromMassTensor(){
    double trace = mMassTensor(0, 0) + mMassTensor(1, 1) + mMassTensor(2, 2);
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        mInertia(i, j) = -mMassTensor(i, j);
    mInertia(0, 0) += trace;
    mInertia(1, 1) += trace;
    mInertia(2, 2) += trace;
  }

} // namespace model3d
