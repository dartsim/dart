#ifndef SRC_MODEL3D_FILEINFO_C3D_H
#define SRC_MODEL3D_FILEINFO_C3D_H

#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;
//Yuting: does fileinfo_base still exist?
//#include "fileinfo_base.h"

namespace model3d {
  class FileInfoC3D /*: public FileInfoBase*/ {
  public:
    FileInfoC3D();
    virtual ~FileInfoC3D();

    //Yuting: do we want to make it public?
    vector< vector<Vector3d> > mData;

    int getNumMarkers() const { return mNumMarkers; } 
    int getNumFrames() const { return mNumFrames; }
    double getFPS() const { return mFPS; }

    virtual bool loadFile(const char*);
    virtual bool saveFile(const char*, int _start, int _end,
                          double sampleRate = 1);
  protected:
    int mNumMarkers;
    int mNumFrames;
    double mFPS;
    char mFileName[256]; // change to string?
  };
} // namespace model3d

#endif // #ifndef SRC_MODEL3D_FILEINFO_C3D_H

