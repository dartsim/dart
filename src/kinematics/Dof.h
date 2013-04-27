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

#ifndef KINEMATICS_DOF_H
#define KINEMATICS_DOF_H

#include <cstring>
#include <Eigen/Dense>

namespace kinematics {
#define MAX_DOF_NAME 128
    class Joint;
    class Transformation;

    class Dof{
    public:
        Dof();
        Dof(double _val);
        Dof(double _val, double _min, double _max);
        Dof(double _val, const char *_name);
        Dof(double _val, const char *_name, double _min, double _max);
	
        virtual ~Dof(){}

        // some helper functions
        void setName(char* _n) { strcpy(mName, _n); }
        inline char* getName() { return mName; }
	
        void setValue(double _v);
        inline double getValue() const { return mVal; }

//        inline void setDefaultValue(double _newDefaultValue) { mDefaultValue = _newDefaultValue; }
//        inline double getDefaultValue() { return mDefaultValue; }
	
        inline double getMin() const { return mMinVal; }
        inline double getMax() const { return mMaxVal; }
        inline void setMin(double _min) { mMinVal = _min; }
        inline void setMax(double _max) { mMaxVal = _max; }
	
        inline int getSkelIndex() const { return mSkelIndex; }
        inline void setSkelIndex(int _idx) { mSkelIndex = _idx; }

        inline bool isVariable() const { return mVariable; }
        inline void setVariable() { mVariable = true; }
	
        inline void setTrans(Transformation *_t){ mTrans = _t; }
        inline Transformation* getTrans() const{ return mTrans; }
	
        inline void setJoint(Joint *_j) { mJoint = _j; }
        inline Joint *getJoint() const { return mJoint; }

    protected:
        void init(double _v, const char * _name, double _min, double _max);
        char mName[MAX_DOF_NAME];
        int mSkelIndex; // Unique to dof in model

//        /// @brief Default value.
//        double mDefaultValue;
        double mVal;	// Value of the joint angle
        double mMinVal;	// Min value allowed
        double mMaxVal;	// Max value allowed

        double mTorque;
        double mMinTorque;
        double mMaxTorque;

        Transformation *mTrans;	// Transformation associated with
        Joint *mJoint;	// Joint to which it belongs

        bool mVariable;	// True when it is a variable and included int he model
    };
} // namespace kinematics

#endif // #ifndef KINEMATICS_DOF_H

