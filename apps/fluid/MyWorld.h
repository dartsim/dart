#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <Eigen/Dense>

#define IX(i, j) ((i)+(mNumCells+2)*(j))
#define SWAP(x0,x) {double *tmp=x0;x0=x;x=tmp;}

class MyWorld {
 public:
    MyWorld(int _numCells, double _timeStep, double _diffCoef, double _viscCoef);

    virtual ~MyWorld();

    int getNumCells() { return mNumCells;}
    double getDensity(int _index) { return mDensity[_index]; }
    double getVelocityU(int _index) { return mU[_index]; }
    double getVelocityV(int _index) { return mV[_index]; }
    void setPreDensity(int _i, int _j, double _source) { mPreDensity[IX(_i, _j)] = _source; }
    void setPreU(int _i, int _j, double _force) { mPreU[IX(_i, _j)] = _force; }
    void setPreV(int _i, int _j, double _force) { mPreV[IX(_i, _j)] = _force; }
    
    void simulate();
    
 protected:
    void densityStep(double *_x, double *_x0);
    void velocityStep(double *_u, double *_v, double *_u0, double *_v0);
    void addDensity(double *_x, double *_x0);
    void addVelocity(double *_u, double *_v, double *_u0, double *_v0);
    void diffuseDensity(double *_x, double *_x0);
    void diffuseVelocity(double *_u, double *_v, double *_u0, double *_v0);
    void advectDensity(double *_d, double *_d0, double *_u, double *_v);
    void advectVelocity(double *_u, double *_v, double *_u0, double *_v0);
    void project(double *_u, double *_v, double *_p, double *_div);
    void externalForces();
    void linearSolve(double *_x, double *_x0, double _a, double _c);
    void setBoundary(double *_x);
    void setVelocityBoundary(double *_u, double *_v);

    int mNumCells;
    double mTimeStep;
    double mDiffusionCoef;
    double mViscosityCoef;
    double *mU;
    double *mV;
    double *mPreU;
    double *mPreV;
    double *mDensity;
    double *mPreDensity;
};

#endif
