#include "transformation.h"
#include "dof.h"
#include "joint.h"

namespace model3d {

Transformation::Transformation():mTransform(Matrix4d::Zero()){
  mJoint=NULL;
  mVariable=true;
  mModelIndex=-1;
  isDirty = true;
}

Transformation::~Transformation(){
  for(int i=0; i<mDofs.size(); i++) {
    delete mDofs[i];
  }
  mDofs.clear();
}

bool Transformation::isPresent(Dof *q){
  for(int i=0; i<mDofs.size(); i++){
    if(q==mDofs[i]) return true;
  }
  return false;
}

void Transformation::applyTransform(Vector3d& v){
  // TODO: fix
  // v= xform(getTransform(), v);
  // Vector4d v4d( v.x, v.y, v.z, 1 );
  // v4d = getTransform() * v4d;
  // v = Vector3d( v4d.x, v4d.y, v4d.z );
}

void Transformation::applyInvTransform(Vector3d& v){
  // TODO: fix
  // v = xform(getInvTransform(), v);
}

void Transformation::applyTransform(Matrix4d& m){
  m = getTransform() * m;
}

void Transformation::applyInvTransform(Matrix4d& m){
  m = getInvTransform() * m;
}
} // namespace model3d