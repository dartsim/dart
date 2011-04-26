#include "dof.h"

double inf = 1e9;

namespace model3d {
	Dof::Dof(){
		init(0, "dof", -inf , inf );
	}

	Dof::Dof(double _v){
		init(_v, "dof", -inf , inf );
	}

	Dof::Dof(double _v, char * _name){
		init(_v, _name, -inf , inf );
	}

	Dof::Dof(double _v, double _min, double Max){
		init(_v, "dof", _min, Max);
	}

	Dof::Dof(double _v, char * _name, double _min, double Max){
		init(_v, _name, _min, Max);
	}

	void Dof::setValue(double v){
		mVal=v; 
		if(mTrans!=NULL) mTrans->setDirty();
	}

	void Dof::init(double _v, const char * _name, double _min, double Max){
		strcpy(mName, _name);
		mVal=_v;
		mMinVal=_min;
		mMaxVal=Max;
		mModelIndex=-1;
		mVariable=false;
		mTrans=NULL;
		mJoint=NULL;	// remains null if const dof
	}

} // namespace model3d

