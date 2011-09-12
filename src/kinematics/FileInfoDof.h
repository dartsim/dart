/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef KINEMATICS_FILEINFO_DOF_H
#define KINEMATICS_FILEINFO_DOF_H

#include <vector>
#include <cassert>
#include <climits>
#include <Eigen/Dense>

namespace kinematics {
    class Skeleton;
 
    class FileInfoDof {
    public:
        FileInfoDof(Skeleton* _skel, double _fps = 120.0);
        virtual ~FileInfoDof();

        bool loadFile(const char* _fileName);
        bool saveFile(const char* _fileName, int _start, int _end, double _sampleRate = 1.0); ///< Note: down sampling not implemented yet

        inline void addDof(std::vector<double>& _dofs){ mDofs.push_back(_dofs); mNumFrames++; }
        inline double getDofAt(int _frame, int _id) const { assert(_frame>=0 && _frame<mNumFrames); return mDofs.at(_frame).at(_id); }
        inline std::vector<double>& getPoseAtFrame(int _frame) { return mDofs.at(_frame); }

        inline void setFPS(double _fps){ mFPS = _fps; }
        inline double getFPS() const { return mFPS; }
        
        inline int getNumFrames() const { return mNumFrames; }
        inline Skeleton* getSkel() const { return mSkel; }

    protected:
        Skeleton* mSkel; ///< model associated with
        double mFPS; ///< frame rate
        int mNumFrames; ///< number of frames
        char mFileName[256]; ///< file name
        std::vector< std::vector<double> > mDofs; ///< dof data [frame][dofIndex]
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_FILEINFO_DOF_H

