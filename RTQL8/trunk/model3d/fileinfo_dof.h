#ifndef SRC_MODEL3D_FILEINFO_DOF_H
#define SRC_MODEL3D_FILEINFO_DOF_H

#include <vector>
#include <cassert>
#include <climits>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;
#include "fileinfo_base.h"

namespace model3d {
  class Skeleton;
  
  struct OtherContact{
    int frameNum;
    Vector3d pos;
    Vector3d force;
    int node;
    OtherContact(int _fn, int _n, Vector3d _p, Vector3d _f);
  };

  class FileInfoDof : public FileInfoBase {
  public:
    FileInfoDof(Skeleton* _skel, double _fps = 120.0);
    virtual ~FileInfoDof();
    virtual bool loadFile(const char*);
    virtual bool saveFile(const char*, int, int, double sampleRate = 1.0);

    inline int getNumContacts();
    inline void setNumContacts(int _n);
    inline void addDof(vector<double>& _dofs);
    inline void addDof(VectorXd _dofs );
    inline void addMuscles(const vector<double>& _muscles );
    inline void addContactForces(vector<Vector3d>& _contacts );
    inline void addContactPos(vector<Vector3d>& _pos );
    inline double getDofAt(int _frame, int _id);
    inline double getMuscleAt( int _frame, int _id);
    inline void getContactAt(int _frame, vector<Vector3d>& _force,
                             vector<Vector3d>& _pos);
    inline void addOtherContact(int _frame, int _node, Vector3d _pos,
                                Vector3d _force );
  protected:
    static Vector4d mColorPool[2];
  public:
    vector< vector<double> > mDofs;
    vector< vector<double> > mMuscles;
    vector< vector<Vector3d> > mContactForces;
    vector< vector<Vector3d> > mContactPos;
    vector< OtherContact* > mOther;
    int mNumContacts;
  };

} // namespace model3d

#endif // #ifndef SRC_MODEL3D_FILEINFO_DOF_H

