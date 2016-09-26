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

#include "dart/utils/C3D.hpp"

#include <cstring>
#include <cstdio>

///////////////////////////////////////////////////////////////////////
//  C3D file reader and writer
///////////////////////////////////////////////////////////////////////

namespace dart {
namespace utils {

float convertDecToFloat(char _bytes[4]) {
    union {
        char theChars[4];
        float theFloat;
    } p;
    p.theChars[0] = _bytes[2];
    p.theChars[1] = _bytes[3];
    p.theChars[2] = _bytes[0];
    p.theChars[3] = _bytes[1];
    if (p.theChars[0] || p.theChars[1] || p.theChars[2] || p.theChars[3])
        --p.theChars[3];          // adjust exponent
    return p.theFloat;
}


void convertFloatToDec(float _f, char* _bytes) {
    char* p = (char*)&_f;
    _bytes[0] = p[2];
    _bytes[1] = p[3];
    _bytes[2] = p[0];
    _bytes[3] = p[1];
    if (_bytes[0] || _bytes[1] || _bytes[2] || _bytes[3])
        ++_bytes[1];      // adjust exponent
}


bool loadC3DFile(const char* _fileName, Eigen::EIGEN_VV_VEC3D& _pointData, int* _nFrame, int* _nMarker, double* _freq) {
    char buf[C3D_REC_SIZE];
    FILE *file;
    Eigen::Vector3d v;
    c3d_head hdr;
    c3d_param param;
    c3d_frameSI frameSI;
    c3d_frame frame;
    bool bDecFmt = true;

    if ((file = fopen( _fileName, "rb" )) == nullptr)
        return false;

    //get the header
    if (!fread(buf, C3D_REC_SIZE, 1, file))
    {
        fclose(file);
        return false;
    }
    memcpy(&hdr, buf, sizeof(hdr));

    //get number format
    if (hdr.rec_start > 2) {
        if (!fread(buf, C3D_REC_SIZE, 1, file))
        {
            fclose(file);
            return false;
        }
        memcpy(&param, buf, sizeof(param));
        if (param.ftype == 84)
            bDecFmt = false;
    }

    //convert if in dec format
    if (bDecFmt) {
        hdr.freq = convertDecToFloat((char*)&hdr.freq);
        hdr.scale = convertDecToFloat((char*)&hdr.scale);
    }

    int numFrames = hdr.end_frame - hdr.start_frame + 1;
    int numMarkers = hdr.pnt_cnt;
    double c3dScale = hdr.scale;

    *_freq = hdr.freq;
    *_nMarker = numMarkers;
    *_nFrame = numFrames;

    float pntScale = (hdr.scale < 0) ? 1 : hdr.scale;

    //eat parameter records
    for (int i = 3; i < hdr.rec_start; i++) {
        if (!fread(buf, C3D_REC_SIZE, 1, file)) {
            fclose(file);
            return false;
        }
    }

    // start retrieving data
    _pointData.resize(numFrames);

    int iRecSize;
    if (c3dScale < 0)
        iRecSize = sizeof(c3d_frame) + ( hdr.a_channels * hdr.a_frames * sizeof(float));
    else
        iRecSize = sizeof(c3d_frameSI) + ( hdr.a_channels * hdr.a_frames * sizeof(short));

    for (int i = 0; i < numFrames; i++) {
        _pointData[i].resize(numMarkers);
        for (int j = 0; j < numMarkers; j++) {
            if (!fread(buf, iRecSize, 1, file))
            {
                fclose(file);
                return false;
            }
            if (c3dScale < 0) {
                memcpy(&frame, buf, sizeof(frame));
                if(bDecFmt){
                    frame.y = convertDecToFloat((char*)&frame.y);
                    frame.z = convertDecToFloat((char*)&frame.z);
                    frame.x = convertDecToFloat((char*)&frame.x);
                }
                v[0] = frame.y / 1000.0;
                v[1] = frame.z / 1000.0;
                v[2] = frame.x / 1000.0;
            } else {
                memcpy(&frameSI, buf, sizeof(frameSI));
                if (bDecFmt) {
                    frameSI.y = (short)convertDecToFloat((char*)&frameSI.y);
                    frameSI.z = (short)convertDecToFloat((char*)&frameSI.z);
                    frameSI.x = (short)convertDecToFloat((char*)&frameSI.x);
                }
                v[0] = (float)frameSI.y * pntScale / 1000.0;
                v[1] = (float)frameSI.z * pntScale / 1000.0;
                v[2] = (float)frameSI.x * pntScale / 1000.0;
            }
            _pointData[i][j] = v;
        }
    }
    fclose(file);

    //const char *pch = strrchr(_fileName, '\\');  //clip leading path
    //if (pch)
    //    _fileName = pch + 1;

    return true;
}

bool saveC3DFile(const char* _fileName, Eigen::EIGEN_VV_VEC3D& _pointData, int _nFrame, int _nMarker, double _freq) {
    FILE *file;
    Eigen::Vector3d v;
    c3d_head hdr;
    c3d_param parm;
    c3d_frame frame;

    if ((file = fopen(_fileName, "wb")) == nullptr)
        return false;

    int mrkrCnt = _nMarker;

    //write header
    memset(&hdr, 0, sizeof(hdr));
    hdr.prec_start = 1;
    hdr.key = 80;
    hdr.pnt_cnt = mrkrCnt;
    hdr.a_channels = 0;
    hdr.start_frame = 1;
    hdr.end_frame = _nFrame;

    //make scale neg to show we are saving floats rather than scaled ints
    hdr.scale = (float)-0.1;
    hdr.rec_start = 3;
    hdr.a_frames = 1;
    hdr.freq = (float)_freq;
    fwrite(&hdr, C3D_REC_SIZE, 1, file);

    memset(&parm, 0, sizeof(parm));
    parm.ftype = 84;
    fwrite(&parm, C3D_REC_SIZE, 1, file);

    //write the data
    frame.residual = 0;
    for (int i = 0; i < _nFrame; i++) {
        for (int j = 0; j < _nMarker; j++) {
            // data is in meters, so put it into millimeters
            v = _pointData[i][j] * 1000.0;
            frame.x = (float)v[2];
            frame.y = (float)v[0];
            frame.z = (float)v[1];
            fwrite(&frame, sizeof(frame), 1, file);
        }
    }

    fclose(file);

    return true;
}

} // namespace utils
} // namespace dart

