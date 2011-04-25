#include "primitive_cube.h"
// TODO: do we need to include an equivalent config file?
// #include "RenderConfig.h"

using namespace std;

namespace model3d {

PrimitiveCube::PrimitiveCube(Vector3d _dim, double _mass)
{
	mDim = _dim;
	mMass = _mass;
	if(_dim!=Vector3d::Zero())
		calVolume();
	if( mMass != 0){
		calMassTensor();
		calInertiaFromMassTensor();
		calVolume();
	}
}

void PrimitiveCube::draw(Vector4d& _color, bool _default){
// TODO: enable opengl renderer flag may not exist yet
#ifdef ENABLE_OPENGL_RENDERER
  Vector4d col = _default? Vector4d(mColor, 1.0): _color;
  glColor4dv( &col ); // TODO: check if this is valid
  glPushMatrix();
  glScalef(mDim(0), mDim(1), mDim(2));
  glutSolidCube(1.0);
  glPopMatrix();
#endif
}

void PrimitiveCube::calMassTensor(){
	mMassTensor(0, 0) = (mDim(0)*mDim(0))/12;
	mMassTensor(1, 1) = (mDim(1)*mDim(1))/12;
	mMassTensor(2, 2) = (mDim(2)*mDim(2))/12;
	mMassTensor(3, 3) = 1;
	mMassTensor *= mMass;
}

void PrimitiveCube::calVolume(){
    mVolume = mDim(0) * mDim(1) * mDim(2); // a * b * c
}

} // namespace model3d
