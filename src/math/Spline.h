#ifndef _SPLINE_
#define _SPLINE_

#include <vector>
#include "vl/vld.h"
#include <algorithm>

#define SPLINE_EPSILON 1.0e-3

using namespace std;

class lessThan{
	public:
		bool operator ()(const Vec2d p1, const Vec2d p2);
};

class Spline{
protected:
	vector<Vec2d> mCtrlPts;	// [0] value is the time stamp

public:
	Spline(){mCtrlPts.clear();}
	Spline(vector<Vec2d> ctrlpts);
	~Spline(){mCtrlPts.clear();}

	void addCtrlPt(Vec2d p);

	virtual double getValue(double p)=0;
	virtual double getPartialDerivative(double p, int npt)=0;
	virtual double getSlope(double p)=0;
	virtual double getSlopePartialDerivative(double p, int npt)=0;
	virtual double getSecondSlope(double p)=0;

	virtual void superSample(vector<double> newctrlpts)=0;
	virtual void subSample(vector<double> newctrlpts)=0;
	virtual void scaleTime(double _scale){
		for(int i=0; i<mCtrlPts.size(); i++){
			mCtrlPts[i][0]*=_scale;
		}
	}
	virtual void scaleIntervalTime(double _fs, double _fe, double _scale){
		for(int i=0; i<mCtrlPts.size(); i++){
			double ctrlShift = (_fe-_fs)*(_scale-1);
			if(mCtrlPts[i][0]>=_fs && mCtrlPts[i][0]<=_fe){
				mCtrlPts[i][0] = _fs + (mCtrlPts[i][0] - _fs)*_scale;
			}
			else if(mCtrlPts[i][0]>_fe){
				mCtrlPts[i][0]+=ctrlShift;
			}
		}
	}
	virtual void setOffsetTime(double _o){
		for(int i=0; i<mCtrlPts.size(); i++){
			mCtrlPts[i][0] += _o;
		}
	}

	inline int getNumCtrlPts(){return mCtrlPts.size();}
	inline Vec2d& getCtrlPt(int i){return mCtrlPts[i];}
};

#endif

