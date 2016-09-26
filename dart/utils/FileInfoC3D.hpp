/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_UTILS_FILEINFOC3D_HPP_
#define DART_UTILS_FILEINFOC3D_HPP_

#include <vector>
#include <Eigen/Dense>

#include "dart/math/MathTypes.hpp"

namespace dart {
namespace utils {

class FileInfoC3D {
    
public:
    FileInfoC3D();
    virtual ~FileInfoC3D(){}

    inline int getNumMarkers() const { return mNumMarkers; }
    inline int getNumFrames() const { return mNumFrames; }
    inline double getFPS() const { return mFPS; }

    inline Eigen::Vector3d getDataAt(int _frame, int _idx) const { return mData.at(_frame).at(_idx); } ///< Note: not checking index range
    inline void addData(const Eigen::EIGEN_V_VEC3D& _data) { mData.push_back(_data); }

    virtual bool loadFile(const char*);
    virtual bool saveFile(const char*, int _start, int _end, double _sampleRate = 1); ///< Note: down sampling not implemented yet
    
protected:
    int mNumMarkers;
    int mNumFrames;
    Eigen::EIGEN_VV_VEC3D mData;
    double mFPS;
    char mFileName[256]; // change to string?
};

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_FILEINFOC3D_HPP_

