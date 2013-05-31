/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_KINEMATICS_FILEINFO_DOF_H
#define DART_KINEMATICS_FILEINFO_DOF_H

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

        inline void addDof(Eigen::VectorXd& _dofs){ mDofs.push_back(_dofs); mNumFrames++; }
        inline double getDofAt(int _frame, int _id) const { assert(_frame>=0 && _frame<mNumFrames); return mDofs.at(_frame)[_id]; }
        inline Eigen::VectorXd getPoseAtFrame(int _frame) { return mDofs.at(_frame); }

        inline void setFPS(double _fps){ mFPS = _fps; }
        inline double getFPS() const { return mFPS; }
        
        inline int getNumFrames() const { return mNumFrames; }
        inline Skeleton* getSkel() const { return mSkel; }

    protected:
        Skeleton* mSkel; ///< model associated with
        double mFPS; ///< frame rate
        int mNumFrames; ///< number of frames
        char mFileName[256]; ///< file name
        std::vector< Eigen::VectorXd > mDofs; ///< dof data [frame][dofIndex]
    };

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_FILEINFO_DOF_H

