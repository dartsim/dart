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

#include "Humanoid.hpp"

#include "State.hpp"

// Macro for functions not implemented yet
#define NOT_YET(FUNCTION) std::cout << #FUNCTION\
                                  << "Not implemented yet."\
                                  << std::endl;

using namespace std;

using namespace Eigen;

using namespace dart::dynamics;
using namespace dart::constraint;

//==============================================================================
Humanoid::Humanoid(Skeleton* /*_skeleton*/)
  : mSkeleton(nullptr),
    mPelvis(nullptr),
    mLeftThigh(nullptr),
    mRightThigh(nullptr),
    mLeftFoot(nullptr),
    mRightFoot(nullptr)
{

}

//==============================================================================
Humanoid::~Humanoid()
{

}

//==============================================================================
Skeleton* Humanoid::getSkeleton()
{
  return mSkeleton;
}

//==============================================================================
BodyNode* Humanoid::getPelvis()
{
  return mPelvis;
}

//==============================================================================
BodyNode* Humanoid::getLeftThigh()
{
  return mLeftThigh;
}

//==============================================================================
BodyNode* Humanoid::getRightThigh()
{
  return mRightThigh;
}

//==============================================================================
BodyNode* Humanoid::getLeftFoot()
{
  return mLeftFoot;
}

//==============================================================================
BodyNode* Humanoid::getRightFoot()
{
  return mRightFoot;
}

//==============================================================================
AtlasRobot::AtlasRobot(Skeleton* _skeleton)
  : Humanoid(_skeleton)
{

}

//==============================================================================
AtlasRobot::~AtlasRobot()
{
  mPelvis     = mSkeleton->getBodyNode("pelvis");
  mLeftFoot   = mSkeleton->getBodyNode("l_foot");
  mRightFoot  = mSkeleton->getBodyNode("r_foot");
  mLeftThigh  = mSkeleton->getBodyNode("l_uleg");
  mRightThigh = mSkeleton->getBodyNode("r_uleg");

  assert(mPelvis     != nullptr);
  assert(mLeftFoot   != nullptr);
  assert(mRightFoot  != nullptr);
  assert(mLeftThigh  != nullptr);
  assert(mRightThigh != nullptr);
}
