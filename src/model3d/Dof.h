/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_DOF_H
#define MODEL3D_DOF_H

#include <cstring>
#include <Eigen/Dense>

namespace model3d {
#define MAX_DOF_NAME 128
    class Joint;
    class Transformation;

    class Dof{
    public:
        Dof();
        Dof(double _val);
        Dof(double _val, double _min, double _max);
        Dof(double _val, const char *_name);
        Dof(double _val, const char *_name, double _min, double _max);
	
        virtual ~Dof(){}

        // some helper functions
        void setName(char* _n) { strcpy(mName, _n); }
        char* getName() { return mName; }
	
        void setValue(double _v);
        double getValue() const { return mVal; }
	
        double getMin() const { return mMinVal; }
        double getMax() const { return mMaxVal; }
        void setMin(double _min) { mMinVal = _min; }
        void setMax(double _max) { mMaxVal = _max; }
	
        int getSkelIndex() const { return mSkelIndex; }
        void setSkelIndex(int _idx) { mSkelIndex = _idx; }

        bool isVariable() const { return mVariable; }
        void setVariable() { mVariable = true; }
	
        void setTrans(Transformation *_t){ mTrans = _t; }
        Transformation* getTrans() const{ return mTrans; }
	
        void setJoint(Joint *_j) { mJoint = _j; }
        Joint *getJoint() const { return mJoint; }

    protected:
        void init(double _v, const char * _name, double _min, double _max);
        char mName[MAX_DOF_NAME];
        int mSkelIndex; // Unique to dof in model

        double mVal;	// Value of the joint angle
        double mMinVal;	// Min value allowed
        double mMaxVal;	// Max value allowed

        double mTorque;
        double mMinTorque;
        double mMaxTorque;

        Transformation *mTrans;	// Transformation associated with
        Joint *mJoint;	// Joint to which it belongs

        bool mVariable;	// True when it is a variable and included int he model
    };
} // namespace model3d

#endif // #ifndef MODEL3D_DOF_H

