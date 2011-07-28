#ifndef _SPLINEB_
#define _SPLINEB_

#include "Spline.h"
#include <cassert>

class SplineB: public Spline{
protected:
//public:
	vector<double> mSplineCtrlPts;
	vector<double> mKnots;
	int mNumCtrlPts;
	int mNumKnots;
	int mDegree;
	bool mNonUniform;

	Mat4d mBasisMat;

	// blending function N_{i,j}(t)
	inline double blendFunc(int _i, int _j, double _t);
	inline double blendFuncDeriv(int _i, int _j, double _t);
	inline double blendFuncSecondDeriv(int _i, int _j, double _t);
	double blendPartialKnot(int _i, int _j, int _k, double _t);
	double blendDerivPartialKnot(int _i, int _j, int _k, double _t);

	void construct(vector<double> _ctrlPts, vector<double> _knots, bool _nonUnif, int _degree);

public:
	SplineB(){}
	SplineB(int _numCtrlPts, bool _nonUnif, int _degree=3){
		vector<double> _knots(_numCtrlPts - _degree + 1);
		for(int i=0; i<_knots.size(); i++) {
			_knots[i] = (double)rand()/(RAND_MAX+1);
			if(i>0) _knots[i]+=_knots[i-1];
		}
		vector<double> _ctrl(_numCtrlPts);
		for(int i=0; i<_ctrl.size(); i++) {
			_ctrl[i] = (double)rand()/(RAND_MAX+1);
		}
		construct(_ctrl, _knots, _nonUnif, _degree);
	}
	SplineB(int _numCtrlPts, double _k0, double _kn, bool _nonUnif, int _degree=3){
		vector<double> knots(_numCtrlPts - _degree + 1);
		double diff = (double)(_kn+0.001-_k0)/(_numCtrlPts -3);
		for(int i=0; i<knots.size(); i++) {
			knots[i]=_k0+i*diff;
		}
		vector<double> _ctrl(_numCtrlPts);
		for(int i=0; i<_ctrl.size(); i++) {
			//_ctrl[i] = 1.0;
			_ctrl[i] = (double)rand()/(RAND_MAX+1);
		}
		construct(_ctrl, knots, _nonUnif, _degree);
	}
	SplineB(int _numCtrlPts, vector<double> _knots, bool _nonUnif, int _degree=3){
		construct(vector<double>(_numCtrlPts, 0), _knots, _nonUnif, _degree);
	}
	SplineB(vector<double> _ctrlPts, vector<double> _knots, bool _nonUnif, int _degree=3){
		construct(_ctrlPts, _knots, _nonUnif, _degree);
	}

	int getLowerBound(double p);
	void getCtrlRange(int _i, double *_ts, double *_te);

	inline int getNumCtrlPts(){return (int)mSplineCtrlPts.size();}
	inline int getNumKnots(){return (int)mKnots.size();}
	inline double getCtrlPt(int _i){
		//if(_i<0) return mSplineCtrlPts[0];
		//if(_i>=mNumCtrlPts) return mSplineCtrlPts[mNumCtrlPts-1];
		return mSplineCtrlPts[_i];
	}
	inline void setCtrlPt(int _i, double _v){
		//if(_i<0) return;
		//if(_i>=mNumCtrlPts) return;
		mSplineCtrlPts[_i] = _v;
	}
	inline double getKnot(int _i){return mKnots[_i];}
	inline void setKnot(int _i, double _v){
		if(_i<=mDegree){
			//printf("Error: SplineB::setKnot - setting first set of duplicate knots!\n");
			for(int i=0; i<=mDegree; i++) mKnots[i] = _v;
		}
		else if(_i>=mNumKnots-mDegree-1){
			//printf("Error: SplineB::setKnot - setting last set of duplicate knots!\n");
			for(int i=0; i<=mDegree; i++) mKnots[mNumKnots-mDegree-1 + i] = _v;
		}
		else mKnots[_i]=_v;
	}

	double getValue(double p);
	double getSlope(double p);
	double getSecondSlope(double p);
	// since 4 pts affect the value/slope - get partial wrt 0,1,2 or 3rd ctrl pt (Local)
	double getPartialDerivative(double p, int npt);
	double getSlopePartialDerivative(double p, int npt);
	double getSecondSlopePartialDerivative(double _t, int _npt);
	// get partial wrt to nth ctrl pt (not local as before)
	double getValuePartialCtrlGlobal(double p, int _ctrlIndex);
	double getSlopePartialCtrlGlobal(double p, int _ctrlIndex);
	double getValuePartialKnot(double _t, int _ki);
	double getSlopePartialKnot(double _t, int _ki);

	// not implemented yet
	virtual void superSample(vector<double> newctrlpts){}
	virtual void subSample(vector<double> newctrlpts){}

	inline vector<double> getSplineCtrlPts(){
		return mSplineCtrlPts;
	}
	inline vector<double> getKnots(){
		vector<double> knots;
		// remove padding on both sides
		int numKnots = mNumKnots - 2*mDegree;
		knots.resize(numKnots);
		for(int i=0; i<mNumKnots; i++){
			if(i-mDegree<0) knots[0] = mKnots[i];
			else if(i-mDegree > knots.size()-1) knots[knots.size()-1] = mKnots[i];
			else knots[i-mDegree] = mKnots[i];
		}
		return knots;
	}

};

#endif
