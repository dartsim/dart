/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_FILEINFO_C3D_H
#define MODEL3D_FILEINFO_C3D_H

#include <vector>
#include <Eigen/Dense>

//Yuting: does fileinfo_base still exist?
//#include "fileinfo_base.h"
#include "utils/EigenArrayHelper.h"

namespace model3d {
    class FileInfoC3D /*: public FileInfoBase*/ {
    public:
        FileInfoC3D();
        virtual ~FileInfoC3D();

        //Yuting: do we want to make it public?
        EIGEN_VV_VEC3D mData;

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

#endif // #ifndef MODEL3D_FILEINFO_C3D_H

