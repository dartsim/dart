/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "PrimitiveEllipsoid.h"
#include "renderer/RenderInterface.h"

using namespace Eigen;

// TODO: do we need to include an equivalent config file?
// #include "RenderConfig.h"

namespace model3d {

    PrimitiveEllipsoid::PrimitiveEllipsoid(Vector3d _dim, double _mass){
        mDim = _dim;
        mMass = _mass;
        if(mDim != Vector3d::Zero())
            computeVolume();
        if(mMass != 0){
            computeMassTensor();
            computeInertiaFromMassTensor();
            computeCOM();
        }
    }

	void PrimitiveEllipsoid::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
		if (!_ri)
			return;
		if (!_useDefaultColor)
			_ri->setPenColor( _color );
		else
			_ri->setPenColor( mColor );
		_ri->pushMatrix();
		_ri->drawEllipsoid(mDim);
		_ri->popMatrix();
    }

    void PrimitiveEllipsoid::computeMassTensor(){
        // if cuboid scaling factor is 1/12
        mMassTensor(0, 0) = (mDim(0)*mDim(0))/5;
        mMassTensor(1, 1) = (mDim(1)*mDim(1))/5;
        mMassTensor(2, 2) = (mDim(2)*mDim(2))/5;
        mMassTensor(3, 3) = 1;
        mMassTensor *= mMass;
    }

    void PrimitiveEllipsoid::computeVolume(){
        mVolume = M_PI * mDim(0) * mDim(1) *mDim(2) /6;	//	4/3* Pi* a/2* b/2* c/2
    }

} // namespace model3d
