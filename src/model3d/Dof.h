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
        Dof(double _val, char *_name);
        Dof(double _val, char *_name, double _min, double _max);
	
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
	
        int getModelIndex() const { return mModelIndex; }
        void setModelIndex(int _idx) { mModelIndex = _idx; }

        bool isVariable() const { return mVariable; }
        void setVariable() { mVariable = true; }
	
        void setTrans(Transformation *_t){ mTrans = _t; }
        Transformation* getTrans() const{ return mTrans; }
	
        void setJoint(Joint *_j) { mJoint = _j; }
        Joint *getJoint() const { return mJoint; }

    protected:
        void init(double _v, const char * _name, double _min, double _max);
        char mName[MAX_DOF_NAME];
        int mModelIndex; // unique to dof in model

        double mVal;	// value of the joint angle
        double mMinVal;	// min value allowed
        double mMaxVal;	// max value allowed

        double mTorque;
        double mMinTorque;
        double mMaxTorque;

        Transformation *mTrans;	// transformation associated with
        Joint *mJoint;	// joint to which it belongs

        bool mVariable;	// true when it is a variable and included int he model
    };
} // namespace model3d

#endif // #ifndef MODEL3D_DOF_H

