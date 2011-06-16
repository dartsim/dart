/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#include "PrimitiveCube.h"

#include "utils/LoadOpengl.h"
using namespace std;
using namespace Eigen;


// TODO: do we need to include an equivalent config file?
// #include "RenderConfig.h"


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

    void PrimitiveCube::draw(const Vector4d& _color, bool _useDefaultColor) const{
        if (_useDefaultColor)
            glColor4d( mColor[0], mColor[1], mColor[2], 1.0 );
        else
            glColor4d( _color[0], _color[1], _color[2], _color[3] );
        glPushMatrix();
        glScalef(mDim(0), mDim(1), mDim(2));
        glutSolidCube(1.0);
        glPopMatrix();
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
