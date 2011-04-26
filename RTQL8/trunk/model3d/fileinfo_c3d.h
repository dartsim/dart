#ifndef SRC_MODEL3D_FILEINFO_C3D_H
#define SRC_MODEL3D_FILEINFO_C3D_H

#include <vector>
using namespace std;
#include <Eigen/Dense>
using namespace Eigen;
#include "fileinfo_base.h"

namespace model3d {
  class FileInfoC3D : public FileInfoBase {
  public:
    FileInfoC3D();
    virtual ~FileInfoC3D();
	
    vector< vector<Vector3d> > mData;

    int getNumMarkers();
    virtual bool loadFile(const char*);
    virtual bool saveFile(const char*, int _start, int _end,
                          double sampleRate = 1);
  protected:
    int mNumMarkers;
  };
} // namespace model3d

#endif // #ifndef SRC_MODEL3D_FILEINFO_C3D_H

