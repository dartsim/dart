#include "SplineB.h"
#include "moremath.h"
#include <cassert>
#include <functional>
#include <algorithm>

void SplineB::construct(vector<double> _ctrlPts, vector<double> _knots, bool _nonUniform, int _degree){
	// assume full multiplicity at the end points to ensure interpolation
	assert(_ctrlPts.size() - _degree + 1 == _knots.size());	
	mDegree = _degree;
	mNonUniform = _nonUniform;
	mNumCtrlPts = (int)_ctrlPts.size();
	mNumKnots = (int)_knots.size() + 2*mDegree;

	mSplineCtrlPts=_ctrlPts;
	// add padding on both sides
	//mKnots=_knots;
	mKnots.resize(mNumKnots);
	for(int i=0; i<mKnots.size(); i++){
		if(i-mDegree<0) mKnots[i] = _knots[0];
		else if(i-mDegree > _knots.size()-1) mKnots[i] = _knots[_knots.size()-1];
		else mKnots[i] = _knots[i-mDegree];

	}

	//for(int i=0; i<mSplineCtrlPts.size(); i++) printf("i=%d, ctrlpt=%lf\n", i, mSplineCtrlPts[i]);
	//cout<<endl;
	//for(int i=0; i<mKnots.size(); i++) printf("i=%d, knot=%lf\n", i, mKnots[i]);
}

// blending function N_{i,j}(t)
inline double SplineB::blendFunc(int _i, int _j, double _t){
	//if(_i<0) return 0;
	//if(_i+_j >= mNumCtrlPts + mDegree) return 0;
	if(_j==0) {
		if(_t>=mKnots[_i] && _t<mKnots[_i+1]) return 1.0;
		else return 0;
	}
	double val = 0;
	if((mKnots[_i+_j] - mKnots[_i]) != 0) val += blendFunc(_i, _j-1, _t)*(_t - mKnots[_i])/(mKnots[_i+_j] - mKnots[_i]);
	if((mKnots[_i+_j+1] - mKnots[_i+1]) != 0) val += blendFunc(_i+1, _j-1, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1] - mKnots[_i+1]);
	return val;
}

// blending function N_{i,j}(t) derivative
inline double SplineB::blendFuncDeriv(int _i, int _j, double _t){
	//if(_i<0) return 0;
	//if(_i+_j >= mNumCtrlPts + mDegree) return 0;
	if(_j==0) return 0;
	double deriv = 0;
	if((mKnots[_i+_j] - mKnots[_i]) != 0) {
		deriv += blendFuncDeriv(_i, _j-1, _t)*(_t - mKnots[_i])/(mKnots[_i+_j] - mKnots[_i]);
		deriv += blendFunc(_i, _j-1, _t)/(mKnots[_i+_j] - mKnots[_i]);
	}
	if((mKnots[_i+_j+1] - mKnots[_i+1]) != 0) {
		deriv += blendFuncDeriv(_i+1, _j-1, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1] - mKnots[_i+1]);
		deriv += -blendFunc(_i+1, _j-1, _t)/(mKnots[_i+_j+1] - mKnots[_i+1]);
	}
	return deriv;
}

// blending function N_{i,j}(t) second derivative
inline double SplineB::blendFuncSecondDeriv(int _i, int _j, double _t){
	//if(_i<0) return 0;
	//if(_i+_j >= mNumCtrlPts + mDegree) return 0;
	if(_j==0) return 0;
	double deriv = 0;
	if((mKnots[_i+_j] - mKnots[_i]) != 0) {
		deriv += blendFuncSecondDeriv(_i, _j-1, _t)*(_t - mKnots[_i])/(mKnots[_i+_j] - mKnots[_i]);
		deriv += 2*blendFuncDeriv(_i, _j-1, _t)/(mKnots[_i+_j] - mKnots[_i]);
	}
	if((mKnots[_i+_j+1] - mKnots[_i+1]) != 0) {
		deriv += blendFuncSecondDeriv(_i+1, _j-1, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1] - mKnots[_i+1]);
		deriv += -2*blendFuncDeriv(_i+1, _j-1, _t)/(mKnots[_i+_j+1] - mKnots[_i+1]);
	}
	return deriv;
}

// return the "i" index such that the interval is [ti, ti+1)
int SplineB::getLowerBound(double _t){
	vector<double>::iterator uindex = upper_bound(mKnots.begin(), mKnots.end(), _t, less<double>());
	// if uindex points to the last or after that
	if(uindex>=mKnots.end()){
		uindex = lower_bound(mKnots.begin(), mKnots.end(), _t);
	}

	// convert the iterator into int value
	int uindex_int=0;
	for(;uindex!=mKnots.end(); uindex_int++, uindex++);
	int ub = (int)(mKnots.size()-1 - uindex_int);
	return ub;
}

void SplineB::getCtrlRange(int _i, double *_ts, double *_te){
	*_ts = mKnots[_i];
	*_te = mKnots[_i+mDegree+1];
}

double SplineB::getValue(double _t){
	double val=0;
	int index = getLowerBound(_t);
	// compute the value using the blending functions
	for(int i=index - mDegree; i<=index; i++){
		val += blendFunc(i, mDegree, _t)*mSplineCtrlPts[i];
	}
	return val;
}

// computes slope of the curve at time _t
double SplineB::getSlope(double _t){
	double val=0;
	int index = getLowerBound(_t);
	// compute the value using the blending functions
	for(int i=index - mDegree; i<=index; i++){
		val += blendFuncDeriv(i, mDegree, _t)*mSplineCtrlPts[i];
	}
	return val;
}

double SplineB::getSecondSlope(double _t){
	double val=0;
	int index = getLowerBound(_t);
	// compute the value using the blending functions
	for(int i=index - mDegree; i<=index; i++){
		val += blendFuncSecondDeriv(i, mDegree, _t)*mSplineCtrlPts[i];
	}
	return val;
}

double SplineB::getPartialDerivative(double _t, int _npt){
	int index = getLowerBound(_t);
	return blendFunc(index - mDegree + _npt, mDegree, _t);
}

double SplineB::getSlopePartialDerivative(double _t, int _npt){
	int index = getLowerBound(_t);
	return blendFuncDeriv(index - mDegree + _npt, mDegree, _t);
}

double SplineB::getSecondSlopePartialDerivative(double _t, int _npt){
	int index = getLowerBound(_t);
	return blendFuncSecondDeriv(index - mDegree + _npt, mDegree, _t);
}

double SplineB::getValuePartialCtrlGlobal(double _t, int _ctrlIndex){
	int index = getLowerBound(_t);
	if(_ctrlIndex >= index - mDegree && _ctrlIndex < index - mDegree + 4) return blendFunc(_ctrlIndex, mDegree, _t);
	else return 0;
}

double SplineB::getSlopePartialCtrlGlobal(double _t, int _ctrlIndex){
	int index = getLowerBound(_t);
	if(_ctrlIndex >= index - mDegree && _ctrlIndex < index - mDegree + 4) return blendFuncDeriv(_ctrlIndex, mDegree, _t);
	else return 0;
}


double SplineB::getValuePartialKnot(double _t, int _ki){
	if(_ki<=mDegree || _ki>=mNumKnots-mDegree-1) cout<<"SplineB::getValuePartialKnot - Not correctly computed !\n";
	double val=0;
	int index = getLowerBound(_t);
	// compute the value using the blending functions
	for(int i=index - mDegree; i<=index; i++){
		val += blendPartialKnot(i, mDegree, _ki, _t)*mSplineCtrlPts[i];
	}
	return val;
}

double SplineB::getSlopePartialKnot(double _t, int _ki){
	double val=0;
	int index = getLowerBound(_t);
	// compute the value using the blending functions
	for(int i=index - mDegree; i<=index; i++){
		val += blendDerivPartialKnot(i, mDegree, _ki, _t)*mSplineCtrlPts[i];
	}
	return val;
}

// evals dN_{i,j}(t)/dt_k
double SplineB::blendPartialKnot(int _i, int _j, int _k, double _t){
	//if(_i < 0) return 0;
	//if(_i + _j >= mNumCtrlPts + mDegree) return 0;
	if(_j == 0) return 0;
	if(_k < _i) return 0;
	if(_k > _i+_j+1) return 0;

	double val=0;
	if(_k==_i){
		if(mKnots[_i+_j]-mKnots[_i] !=0) {
			val += blendFunc(_i, _j-1, _t)*(_t-mKnots[_i+_j])/(moremath::sqr(mKnots[_i+_j]-mKnots[_i]));
			val += blendPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
		}
		return val;
	}
	if(_k==_i+_j){
		if(mKnots[_i+_j]-mKnots[_i] !=0) {
			val += -blendFunc(_i, _j-1, _t)*(_t-mKnots[_i])/moremath::sqr(mKnots[_i+_j]-mKnots[_i]);
			val += blendPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
		}
		if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
			if(_j==1) val += blendFunc(_i+1, _j-1, _t)*(mKnots[_i+_j+1] - _t)/moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += blendPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
		}
		return val;
	}
	if(_k==_i+1){
		if(mKnots[_i+_j]-mKnots[_i] !=0) {
			if(_j==1) val += -blendFunc(_i, _j-1, _t)*(_t-mKnots[_i])/moremath::sqr(mKnots[_i+_j]-mKnots[_i]);
			val += blendPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
		}
		if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
			val += blendFunc(_i+1, _j-1, _t)*(mKnots[_i+_j+1] - _t)/moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += blendPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
		}
		return val;
	}
	if(_k==_i+_j+1){
		if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
			val += blendFunc(_i+1, _j-1, _t)*(_t-mKnots[_i+1])/(moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]));
			val += blendPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
		}
		return val;
	}
	// default case
	if(mKnots[_i+_j]-mKnots[_i] !=0) {
		val += blendPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
	}
	if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
		val += blendPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
	}
	return val;
}

// evals d[dN_{i,j}(t)/dt]/dt_k
double SplineB::blendDerivPartialKnot(int _i, int _j, int _k, double _t){
	//if(_i < 0) return 0;
	//if(_i + _j >= mNumCtrlPts + mDegree) return 0;
	if(_j == 0) return 0;
	if(_k < _i) return 0;
	if(_k > _i+_j+1) return 0;

	double val=0;
	if(_k==_i){
		if(mKnots[_i+_j]-mKnots[_i] !=0) {
			val += blendFuncDeriv(_i, _j-1, _t)*(_t-mKnots[_i+_j])/(moremath::sqr(mKnots[_i+_j]-mKnots[_i]));
			val += blendDerivPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
			val += blendFunc(_i, _j-1, _t)/(moremath::sqr(mKnots[_i+_j]-mKnots[_i]));
			val += blendPartialKnot(_i, _j-1, _k, _t)/(mKnots[_i+_j]-mKnots[_i]);
		}
		return val;
	}
	if(_k==_i+_j){
		if(mKnots[_i+_j]-mKnots[_i] !=0) {
			val += -blendFuncDeriv(_i, _j-1, _t)*(_t-mKnots[_i])/moremath::sqr(mKnots[_i+_j]-mKnots[_i]);
			val += blendDerivPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
			val += -blendFunc(_i, _j-1, _t)/moremath::sqr(mKnots[_i+_j]-mKnots[_i]);
			val += blendPartialKnot(_i, _j-1, _k, _t)/(mKnots[_i+_j]-mKnots[_i]);
		}
		if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
			if(_j==1) val += blendFuncDeriv(_i+1, _j-1, _t)*(mKnots[_i+_j+1] - _t)/moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += blendDerivPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
			if(_j==1) val += -blendFunc(_i+1, _j-1, _t)/moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += -blendPartialKnot(_i+1, _j-1, _k, _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
		}
		return val;
	}
	if(_k==_i+1){
		if(mKnots[_i+_j]-mKnots[_i] !=0) {
			if(_j==1) val += -blendFuncDeriv(_i, _j-1, _t)*(_t-mKnots[_i])/moremath::sqr(mKnots[_i+_j]-mKnots[_i]);
			val += blendDerivPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
			if(_j==1) val += -blendFunc(_i, _j-1, _t)/moremath::sqr(mKnots[_i+_j]-mKnots[_i]);
			val += blendPartialKnot(_i, _j-1, _k, _t)/(mKnots[_i+_j]-mKnots[_i]);
		}
		if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
			val += blendFuncDeriv(_i+1, _j-1, _t)*(mKnots[_i+_j+1] - _t)/moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += blendDerivPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += -blendFunc(_i+1, _j-1, _t)/moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += -blendPartialKnot(_i+1, _j-1, _k, _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
		}		
		return val;
	}
	if(_k==_i+_j+1){
		if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
			val += blendFuncDeriv(_i+1, _j-1, _t)*(_t-mKnots[_i+1])/(moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]));
			val += blendDerivPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
			val += blendFunc(_i+1, _j-1, _t)/(moremath::sqr(mKnots[_i+_j+1]-mKnots[_i+1]));
			val += -blendPartialKnot(_i+1, _j-1, _k, _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
		}
		return val;
	}
	// default case
	if(mKnots[_i+_j]-mKnots[_i] !=0) {
		val += blendDerivPartialKnot(_i, _j-1, _k, _t)*(_t-mKnots[_i])/(mKnots[_i+_j]-mKnots[_i]);
		val += blendPartialKnot(_i, _j-1, _k, _t)/(mKnots[_i+_j]-mKnots[_i]);
	}
	if(mKnots[_i+_j+1]-mKnots[_i+1] !=0) {
		val += blendDerivPartialKnot(_i+1, _j-1, _k, _t)*(mKnots[_i+_j+1] - _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
		val += -blendPartialKnot(_i+1, _j-1, _k, _t)/(mKnots[_i+_j+1]-mKnots[_i+1]);
	}
	return val;
}
