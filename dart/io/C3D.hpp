/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_IO_C3D_HPP_
#define DART_IO_C3D_HPP_

#include <vector>
#include <ctime>
#include <Eigen/Dense>
#include "dart/math/MathTypes.hpp"

////////////////////////////////////////////////////////////////////////////////
//  C3D file reader and writer
namespace dart {
namespace io {

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

bool loadC3DFile( const char* _fileName, std::vector<std::vector<Eigen::Vector3d>>& _pointData,
                  int* _nFrame, int* _nMarker, double* _freq );
bool saveC3DFile( const char* _fileName, std::vector<std::vector<Eigen::Vector3d>>& _pointData,
                  int _nFrame, int _nMarker, double _freq );

} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_C3D_HPP_


