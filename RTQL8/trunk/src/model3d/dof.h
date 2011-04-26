#ifndef SRC_MODEL3D_DOF_H
#define SRC_MODEL3D_DOF_H

#include <cstring>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;

#include "transformation.h"

namespace model3d {
#define MAX_DOF_NAME 128
  class Joint;
  class Transformation;

  class Dof{
  public:
    Dof();
    Dof(double val);
    Dof(double val, double min, double max);
    Dof(double val, char *name);
    Dof(double val, char *name, double min, double max);
	
    ~Dof(){}

    // some helper functions
    void setName(char* _n){ strcpy(mName,_n); }
    char* getName(){return mName;}
	
    void setValue(double v);
    double getValue(){return mVal;}
	
    double getMin(){return mMinVal;}
    double getMax(){return mMaxVal;}
    void setMin(double _min){mMinVal=_min;}
    void setMax(double _max){mMaxVal=_max;}
	
    int getModelIndex(){return mModelIndex;}
    void setModelIndex(int _idx){mModelIndex=_idx;}

    bool isVariable(){return mVariable;}
    void setVariable(){mVariable=true;}
	
    void setTrans(Transformation *_t){mTrans=_t;}
    Transformation* getTrans(){return mTrans;}
	
    void setJoint(Joint *_j){mJoint=_j;}
    Joint *getJoint(){return mJoint;}

  protected:
    void init(double _v, const char * _name, double _min, double Max);
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

#endif // #ifndef SRC_MODEL3D_DOF_H

