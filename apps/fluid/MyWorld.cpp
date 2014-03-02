#include "MyWorld.h"
#include "math/Helpers.h"

using namespace Eigen;

MyWorld::MyWorld(int _numCells, double _timeStep, double _diffCoef, double _viscCoef) {
    mNumCells = _numCells;
    mTimeStep = _timeStep;
    mDiffusionCoef = _diffCoef;
    mViscosityCoef = _viscCoef;

    int size = (mNumCells + 2) * (mNumCells + 2);
    
    // Allocate memory for velocity and density fields
    mU = new double[size];
    mV = new double[size];
    mPreU = new double[size];
    mPreV = new double[size];
    mDensity = new double[size];
    mPreDensity = new double[size];
    
    for (int i = 0; i < size; i++) {
        mU[i] = mPreU[i] = 0.0;
        mV[i] = mPreV[i] = 0.0;
        mDensity[i] = mPreDensity[i] = 0.0;
    }            
}

MyWorld::~MyWorld() {
}

void MyWorld::simulate() {
    velocityStep(mU, mV, mPreU, mPreV);
    densityStep(mDensity, mPreDensity);
    externalForces();
}

void MyWorld::densityStep(double *_x, double *_x0) {
    addDensity(_x, _x0); // Add density to _x which points at mDensity
    SWAP(_x, _x0); // _x now points at mPresendity
    diffuseDensity(_x, _x0); // Diffusion on _x which pointst at mPreDensity
    SWAP(_x, _x0); // _x now points at mDensity
    advectDensity(_x, _x0, mU, mV); // Advection on _x which points atmDensity
}


void MyWorld::velocityStep(double *_u, double *_v, double *_u0, double *_v0) {
    addVelocity(_u, _v, _u0, _v0);
    SWAP(_u, _u0); // _u now points at mPreU
    SWAP(_v, _v0); // _v now points at mPreV
    diffuseVelocity(_u, _v, _u0, _v0);
    project (_u, _v, _u0, _v0);
    SWAP(_u, _u0); // _u now points at mU
    SWAP(_v, _v0); // _u now points at mV
    advectVelocity(_u, _v, _u0, _v0);
    project(_u, _v, _u0, _v0);
}

void MyWorld::addDensity(double *_x, double *_x0) {
    int size = (mNumCells + 2) * (mNumCells + 2);
    for (int i = 0; i < size; i++)
        _x[i] += mTimeStep * _x0[i];
}

void MyWorld::addVelocity(double *_u, double *_v, double *_u0, double *_v0) {
    int size = (mNumCells + 2) * (mNumCells + 2);
    for (int i = 0; i < size; i++) {
        _u[i] += mTimeStep * _u0[i];
        _v[i] += mTimeStep * _v0[i];
    }
}

void MyWorld::diffuseDensity(double *_x, double *_x0) {
    double a = mTimeStep * mDiffusionCoef * mNumCells * mNumCells;
    linearSolve(_x, _x0, a, 1 + 4 * a);
    setBoundary(_x);
}

void MyWorld::diffuseVelocity(double *_u, double *_v, double *_u0, double *_v0) {
    double a = mTimeStep * mViscosityCoef * mNumCells * mNumCells;
    linearSolve(_u, _u0, a, 1 + 4 * a);
    linearSolve(_v, _v0, a, 1 + 4 * a);
    setVelocityBoundary(_u, _v);
}

void MyWorld::advectDensity(double *_d, double *_d0, double *_u, double *_v) {
    double dt0 = mTimeStep * mNumCells;
    for (int i = 1; i <= mNumCells; i++) {
        for (int j = 1; j <= mNumCells; j++) {
            double x = i- dt0 * _u[IX(i,j)]; 
            double y = j - dt0 * _v[IX(i,j)];
            if (x < 0.5) 
                x = 0.5f; 
            if (x > mNumCells + 0.5) 
                x = mNumCells + 0.5;
            int i0 = (int)x;
            int i1 = i0 + 1;
            if (y < 0.5) 
                y = 0.5;
            if (y > mNumCells + 0.5)
                y = mNumCells + 0.5;
            int j0 = (int)y;
            int j1 = j0 + 1;
            double s1 = x - i0;
            double s0 = 1 - s1;
            double t1 = y - j0;
            double t0 = 1 - t1;
            _d[IX(i,j)] = s0 * (t0 * _d0[IX(i0, j0)] + t1 * _d0[IX(i0,j1)])+ s1 * (t0 * _d0[IX(i1, j0)] + t1 * _d0[IX(i1,j1)]);
	}
    }
    setBoundary(_d);
}

void MyWorld::advectVelocity(double *_u, double *_v, double *_u0, double *_v0) {
    // TODO: Add velocity advection code here
}

void MyWorld::project(double *_u, double *_v, double *_p, double *_div) {
    // TODO: Add projection code here
}

void MyWorld::externalForces() {
    int size = (mNumCells + 2) * (mNumCells + 2);
    for (int i = 0; i< size; i++) {
        mPreDensity[i] = 0;
        mPreU[i] = 0;
        mPreV[i] = 0;
    }
}

void MyWorld::linearSolve(double *_x, double *_x0, double _a, double _c) {
    for (int k = 0; k < 20; k++) {
        for (int i = 1; i <= mNumCells; i++) {
            for (int j = 1; j <= mNumCells; j++) {
                _x[IX(i, j)] = (_x0[IX(i, j)] + _a * (_x[IX(i-1, j)] + _x[IX(i+1, j)] + _x[IX(i, j-1)] + _x[IX(i, j+1)])) / _c;
            }
        }
    }
}

void MyWorld::setBoundary(double *_x) {
    for (int i = 1; i <= mNumCells; i++) {
        _x[IX(0 ,i)] = _x[IX(1,i)];
        _x[IX(mNumCells+1, i)] = _x[IX(mNumCells, i)];
        _x[IX(i, 0)] = _x[IX(i, 1)];
        _x[IX(i, mNumCells+1)] = _x[IX(i, mNumCells)];
 
    }
    _x[IX(0, 0)] = 0.5 * (_x[IX(1, 0)] + _x[IX(0, 1)]);
    _x[IX(0, mNumCells+1)] = 0.5 * (_x[IX(1, mNumCells+1)] + _x[IX(0, mNumCells)]);
    _x[IX(mNumCells+1, 0)] = 0.5 * (_x[IX(mNumCells, 0)] + _x[IX(mNumCells+1, 1)]);
    _x[IX(mNumCells+1, mNumCells+1)] = 0.5 * (_x[IX(mNumCells, mNumCells+1)] + _x[IX(mNumCells+1, mNumCells)]);
}

void MyWorld::setVelocityBoundary(double *_u, double *_v) {
    for (int i = 1; i <= mNumCells; i++) {
        _u[IX(0 ,i)] = -_u[IX(1,i)];
        _u[IX(mNumCells+1, i)] = -_u[IX(mNumCells, i)];
        _u[IX(i, 0)] = _u[IX(i, 1)];
        _u[IX(i, mNumCells+1)] = _u[IX(i, mNumCells)];

        _v[IX(0 ,i)] = _v[IX(1,i)];
        _v[IX(mNumCells+1, i)] = _v[IX(mNumCells, i)];
        _v[IX(i, 0)] = -_v[IX(i, 1)];
        _v[IX(i, mNumCells+1)] = -_v[IX(i, mNumCells)];
    }
    _u[IX(0, 0)] = 0.5 * (_u[IX(1, 0)] + _u[IX(0, 1)]);
    _u[IX(0, mNumCells+1)] = 0.5 * (_u[IX(1, mNumCells+1)] + _u[IX(0, mNumCells)]);
    _u[IX(mNumCells+1, 0)] = 0.5 * (_u[IX(mNumCells, 0)] + _u[IX(mNumCells+1, 1)]);
    _u[IX(mNumCells+1, mNumCells+1)] = 0.5 * (_u[IX(mNumCells, mNumCells+1)] + _u[IX(mNumCells+1, mNumCells)]);
    _v[IX(0, 0)] = 0.5 * (_v[IX(1, 0)] + _v[IX(0, 1)]);
    _v[IX(0, mNumCells+1)] = 0.5 * (_v[IX(1, mNumCells+1)] + _v[IX(0, mNumCells)]);
    _v[IX(mNumCells+1, 0)] = 0.5 * (_v[IX(mNumCells, 0)] + _v[IX(mNumCells+1, 1)]);
    _v[IX(mNumCells+1, mNumCells+1)] = 0.5 * (_v[IX(mNumCells, mNumCells+1)] + _v[IX(mNumCells+1, mNumCells)]);

}
