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

#ifndef DART_KINEMATICS_C3D_H
#define DART_KINEMATICS_C3D_H

#include <vector>
#include <ctime>
#include <Eigen/Dense>
#include "math/EigenHelper.h"

////////////////////////////////////////////////////////////////////////////////
//  C3D file reader and writer
namespace kinematics {

#define C3D_REC_SIZE   512

    typedef struct c3d_head_t {
        unsigned char	prec_start;
        unsigned char	key;
        short	pnt_cnt;
        short	a_channels;
        short	start_frame;
        short	end_frame;
        short	int_gap;
        float	scale;
        short	rec_start;
        short	a_frames;
        float	freq;
        short	stuff[244];	
    } c3d_head;

    typedef struct c3d_param_t {
        unsigned char	reserved[2];
        unsigned char	pblocks;
        unsigned char	ftype;
        char stuff[C3D_REC_SIZE-4];	
    } c3d_param;

    typedef struct c3d_frameSI_t {
        short	x, y, z;
        unsigned char	cam_byte;
        unsigned char	residual;
    } c3d_frameSI;

    typedef struct c3d_frame_t {
        float	x, y, z;
        float	residual;
    } c3d_frame;

    float convertDecToFloat(char _bytes[4]);
    void convertFloatToDec(float _f, char* _bytes);

    bool loadC3DFile( const char *_fileName, EIGEN_VV_VEC3D& _pointData,
                      int *_nFrame, int *_nMarker, double *_freq );
    bool saveC3DFile( const char* _fileName, EIGEN_VV_VEC3D& _pointData,
                      int _nFrame, int _nMarker, double _freq ); 

    double maxElem(std::vector<double>& _arr, int& _index);
    double minElem(std::vector<double>& _arr, int& _index);

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_C3D_H


