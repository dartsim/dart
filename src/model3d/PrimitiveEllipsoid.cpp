/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "PrimitiveEllipsoid.h"

using namespace Eigen;

// TODO: do we need to include an equivalent config file?
// #include "RenderConfig.h"

namespace model3d {

    PrimitiveEllipsoid::PrimitiveEllipsoid(Vector3d _dim, double _mass){
        mDim = _dim;
        mMass = _mass;
        if(mDim != Vector3d::Zero())
            calVolume();
        if(mMass != 0){
            calMassTensor();
            calInertiaFromMassTensor();
            calCOM();
        }
    }

	void PrimitiveEllipsoid::draw(renderer::RenderInterface* RI, const Vector4d& _color, bool _useDefaultColor) const {
#if 1
		if (!RI)
			return;
		if (!_useDefaultColor)
			RI->SetPenColor( _color );
		else
			RI->SetPenColor( mColor );
		RI->PushMatrix();
		RI->DrawEllipsoid(mDim);
		RI->PopMatrix();
#else
        if (_useDefaultColor)
            glColor4d( mColor[0], mColor[1], mColor[2], 1.0 );
        else
            glColor4d( _color[0], _color[1], _color[2], _color[3] );
        glPushMatrix();
        glScalef(mDim(0), mDim(1), mDim(2));
        glutSolidSphere(0.5, 16, 16);
        glPopMatrix();
#endif
    }

    void PrimitiveEllipsoid::calMassTensor(){
        // if cuboid scaling factor is 1/12
        mMassTensor(0, 0) = (mDim(0)*mDim(0))/5;
        mMassTensor(1, 1) = (mDim(1)*mDim(1))/5;
        mMassTensor(2, 2) = (mDim(2)*mDim(2))/5;
        mMassTensor(3, 3) = 1;
        mMassTensor *= mMass;
    }

    void PrimitiveEllipsoid::calVolume(){
        mVolume = M_PI * mDim(0) * mDim(1) *mDim(2) /6;	//	4/3* Pi* a/2* b/2* c/2
    }

} // namespace model3d
