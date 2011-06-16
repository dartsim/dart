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

#include "utils/EigenArrayHelper.h"

namespace model3d {
    class FileInfoC3D {
    
    public:
        FileInfoC3D();
        virtual ~FileInfoC3D(){}

        int getNumMarkers() const { return mNumMarkers; } 
        int getNumFrames() const { return mNumFrames; }
        double getFPS() const { return mFPS; }

        Eigen::Vector3d getDataAt(int _frame, int _idx) const { return mData.at(_frame).at(_idx); } ///< Note: not checking index range
        void addData(const EIGEN_V_VEC3D& _data) { mData.push_back(_data); }

        virtual bool loadFile(const char*);
        virtual bool saveFile(const char*, int _start, int _end, double _sampleRate = 1); ///< Note: down sampling not implemented yet
    
    protected:
        int mNumMarkers;
        int mNumFrames;
        EIGEN_VV_VEC3D mData;
        double mFPS;
        char mFileName[256]; // change to string?
    };
} // namespace model3d

#endif // #ifndef MODEL3D_FILEINFO_C3D_H

