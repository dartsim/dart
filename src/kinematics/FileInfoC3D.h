/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef KINEMATICS_FILEINFO_C3D_H
#define KINEMATICS_FILEINFO_C3D_H

#include <vector>
#include <Eigen/Dense>

#include "utils/EigenHelper.h"

namespace kinematics {
    class FileInfoC3D {
    
    public:
        FileInfoC3D();
        virtual ~FileInfoC3D(){}

        inline int getNumMarkers() const { return mNumMarkers; } 
        inline int getNumFrames() const { return mNumFrames; }
        inline double getFPS() const { return mFPS; }

        inline Eigen::Vector3d getDataAt(int _frame, int _idx) const { return mData.at(_frame).at(_idx); } ///< Note: not checking index range
        inline void addData(const EIGEN_V_VEC3D& _data) { mData.push_back(_data); }

        virtual bool loadFile(const char*);
        virtual bool saveFile(const char*, int _start, int _end, double _sampleRate = 1); ///< Note: down sampling not implemented yet
    
    protected:
        int mNumMarkers;
        int mNumFrames;
        EIGEN_VV_VEC3D mData;
        double mFPS;
        char mFileName[256]; // change to string?
    };
} // namespace kinematics

#endif // #ifndef KINEMATICS_FILEINFO_C3D_H

