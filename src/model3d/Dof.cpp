/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/


#include "Dof.h"
#include "Transformation.h"

double inf = 1e9;

namespace model3d {
    Dof::Dof(){
        init(0, "dof", -inf , inf );
    }

    Dof::Dof(double _v){
        init(_v, "dof", -inf , inf );
    }

    Dof::Dof(double _v, const char * _name){
        init(_v, _name, -inf , inf );
    }

    Dof::Dof(double _v, double _min, double _max){
        init(_v, "dof", _min, _max);
    }

    Dof::Dof(double _v, const char * _name, double _min, double _max){
        init(_v, _name, _min, _max);
    }

    void Dof::setValue(double _v){
        mVal = _v; 
        if (mTrans != NULL) mTrans->setDirty();
    }

    void Dof::init(double _v, const char * _name, double _min, double _max){
        strcpy(mName, _name);
        mVal = _v;
        mMinVal = _min;
        mMaxVal = _max;
        mSkelIndex = -1;
        mVariable = false;
        mTrans = NULL;
        mJoint = NULL;	// remains null if const dof
    }

} // namespace model3d

