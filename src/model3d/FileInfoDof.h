/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_FILEINFO_DOF_H
#define MODEL3D_FILEINFO_DOF_H

#include <vector>
#include <cassert>
#include <climits>
#include <Eigen/Dense>

//#include "fileinfo_base.h"

namespace model3d {
    class Skeleton;
 
    //Yuting: let's have the most basic info for now
    /* 
       struct OtherContact{
       int frameNum;
       Vector3d pos;
       Vector3d force;
       int node;
       OtherContact(int _fn, int _n, Vector3d _p, Vector3d _f);
       };
    */
    class FileInfoDof /*: public FileInfoBase*/ {
    public:
        FileInfoDof(Skeleton* _skel, double _fps = 120.0);
        ~FileInfoDof();
    
        bool loadFile(const char*);
        bool saveFile(const char*, int, int, double sampleRate = 1.0);

//   inline int getNumContacts();
//   inline void setNumContacts(int _n);
        inline void addDof(std::vector<double>& _dofs){ mDofs.push_back(_dofs); mNumFrames++; }
//    inline void addDof(VectorXd _dofs );
//    inline void addMuscles(const vector<double>& _muscles );
//    inline void addContactForces(vector<Vector3d>& _contacts );
//    inline void addContactPos(vector<Vector3d>& _pos );
        inline double getDofAt(int _frame, int _id) const { assert(_frame>=0 && _frame<mNumFrames); return mDofs.at(_frame).at(_id); }
//   inline double getMuscleAt( int _frame, int _id);
//   inline void getContactAt(int _frame, vector<Vector3d>& _force, vector<Vector3d>& _pos);
//    inline void addOtherContact(int _frame, int _node, Vector3d _pos, Vector3d _force );
//  protected:
//    static Vector4d mColorPool[2];
  	inline void setFPS(double _fps){ mFPS = _fps; }
	inline double getFPS() const { return mFPS; }
  	inline double getNumFrames() const { return mNumFrames; }
 	inline Skeleton* getModel() const { return mSkel; }

    public:
        std::vector< std::vector<double> > mDofs;
//    vector< vector<double> > mMuscles;
//    vector< vector<Vector3d> > mContactForces;
//    vector< vector<Vector3d> > mContactPos;
//    vector< OtherContact* > mOther;
//    int mNumContacts;
  
    protected:
  	Skeleton* mSkel;
  	double mFPS;
	int mNumFrames;
  	char mFileName[256];
    };

} // namespace model3d

#endif // #ifndef MODEL3D_FILEINFO_DOF_H

