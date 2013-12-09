/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/24/2011
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

#ifndef MAYAEXPORTS_MAYAEXPORTSKELETON_H
#define MAYAEXPORTS_MAYAEXPORTSKELETON_H

#include <fstream>
#include <string>
#include "dart/math/UtilsRotation.h"

namespace kinematics{
    class Skeleton;
    class BodyNode;
}   // namespace kinematics

namespace utils {
    namespace mayaexports {

        class MayaExportSkeleton{
        public:    
            static bool exportMayaAscii( kinematics::Skeleton *_model, ofstream &_outFile );
    
        private:
            friend class MayaExportMotion;
        
            static const string mSuffixPrim;
            static const string mSuffixShape;
            static const string mSuffixGeom;
            static const dart_math::RotationOrder mRotOrder;

            static bool exportMayaAsciiHeader(ofstream &_outFile );
            static bool exportMayaAsciiShape(kinematics::BodyNode* b, ofstream &_outFile, const string &_prefix, const string &_suffix);
            static bool exportMayaAsciiSegmentSubtree(kinematics::BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix );
            // do not write the const stuff - all keyed now
            static bool exportMayaAsciiSegmentSubtree2(kinematics::BodyNode *_b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix );
            static bool exportMayaAsciiSegmentFooter(kinematics::BodyNode* _b, const int _writeNumNodes, ofstream &_outFile, const string &_prefix, const string &_suffix  );

        };
    } // namespace mayaexports
} // namespace utils

#endif  // MAYAEXPORTS_MAYAEXPORTSKELETON_H
