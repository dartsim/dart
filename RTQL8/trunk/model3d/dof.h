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

  class Dof{
  public:
    Dof();
    Dof(double val);
    Dof(double val, double min, double max);
    Dof(double val, char *name);
    Dof(double val, char *name, double min, double max);
	
    ~Dof(){}

    // some inline helper functions
    inline void setName(char* _n){ strcpy(mName,_n); }
    inline char* getName(){return mName;}
	
    inline void setValue(double v);
    inline double getValue(){return mVal;}
	
    inline double getMin(){return mMinVal;}
    inline double getMax(){return mMaxVal;}
    inline void setMin(double _min){mMinVal=_min;}
    inline void setMax(double _max){mMaxVal=_max;}
	
    inline int getModelIndex(){return mModelIndex;}
    inline void setModelIndex(int _idx){mModelIndex=_idx;}

    inline bool isVariable(){return mVariable;}
    inline void setVariable(){mVariable=true;}
	
    inline void setTrans(Transformation *_t){mTrans=_t;}
    inline Transformation* getTrans(){return mTrans;}
	
    inline void setJoint(Joint *_j){mJoint=_j;}
    inline Joint *getJoint(){return mJoint;}

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

