#include "PrimitiveCube.h"
#ifndef _RENDERER_TEST
#include "utils/LoadOpengl.h"
#endif

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

  void PrimitiveCube::draw(Renderer::OpenGLRenderInterface* RI, const Vector4d& _color, bool _default){
#ifdef _RENDERER_TEST
	if (!RI) return;
	if (_default)
		RI->SetPenColor( _color );
	else
		RI->SetPenColor( mColor );
	RI->PushMatrix();
	RI->DrawCube(mDim);
	RI->PopMatrix();
#else
	if (_default)
		glColor4d( _color[0], _color[1], _color[2], _color[3] );
	else
		glColor4d( mColor[0], mColor[1], mColor[2], 1.0 );
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
