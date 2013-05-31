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

#include <string>
#include <fstream>
#include <iostream>

#include "kinematics/Skeleton.h"
#include "kinematics/Dof.h"
#include "kinematics/FileInfoDof.h"

using namespace std;

namespace kinematics{

    FileInfoDof::FileInfoDof(Skeleton* _skel, double _fps)
        : mSkel(_skel), mFPS(_fps), mNumFrames(0){
    }

    FileInfoDof::~FileInfoDof(){
        mDofs.clear();
        mNumFrames = 0;
    }

    bool FileInfoDof::loadFile(const char* _fName)
    {
        ifstream inFile(_fName);
        if (inFile.fail() == 1) return false;

        inFile.precision(20);
        char buffer[256];
        int nDof;

        //nFrames =
        inFile >> buffer;
        inFile >> buffer;
        inFile >> mNumFrames;

        //nDof =
        inFile >> buffer;
        inFile >> buffer;
        inFile >> nDof;

        if (mSkel == NULL || mSkel->getNumDofs()!=nDof)
            return false;

        mDofs.resize(mNumFrames);

        //dof names
        for (int i = 0; i < nDof; i++)
            inFile >> buffer;
        for (int j = 0; j < mNumFrames; j++) {
            mDofs[j].resize(nDof);
            for (int i = 0; i < nDof; i++) {
                double val;
                inFile >> val;
                mDofs[j][i] = val;
            }
        }

        // fps
        inFile >> buffer;
        if (!inFile.eof())
            inFile>>mFPS;
        
        inFile.close();

        string text = _fName;
        int lastSlash = text.find_last_of("/");
        text = text.substr(lastSlash+1);
        strcpy(mFileName, text.c_str());
        return true;
    }

    bool FileInfoDof::saveFile( const char* _fName, int _start, int _end, double _sampleRate ){
        if (_end < _start) return false;

        ofstream outFile(_fName, ios::out);
        if (outFile.fail()) return false;

        int first = _start<mNumFrames?_start:mNumFrames-1;
        int last = _end<mNumFrames?_end:mNumFrames-1;

        outFile.precision(20);
        outFile << "frames = " << last-first+1 << " dofs = " << mSkel->getNumDofs() << endl;

        for (int i = 0; i < mSkel->getNumDofs(); i++)
            outFile << mSkel->getDof(i)->getName() << ' ';
        outFile << endl;

        for (int i = first; i <= last; i++){
            for (int j = 0; j < mSkel->getNumDofs(); j++){
                outFile << mDofs[i][j] << ' ';
            }
            outFile << endl;
        }

        outFile << "FPS " << mFPS << endl;

        outFile.close();

        string text = _fName;
        int lastSlash = text.find_last_of("/");
        text = text.substr(lastSlash+1);
        strcpy(mFileName, text.c_str());
        return true;
    }
}
