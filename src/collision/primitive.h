/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#ifndef COLLISION_CHECKING_PRIMITIVE_H
#define COLLISION_CHECKING_PRIMITIVE_H

#include "BVH_defs.h"

/** \brief Main namespace */
namespace collision_checking
{

/** \brief Simple triangle with 3 indices for points */
struct Triangle3e
{
  unsigned int vids[3];

  int geom_id; // > 0 (geom_id - 1) is the index for geom object
               // < 0 -(geom_id + 1) is the index for space object
               // == 0 unset

  int sub_geom_id; // the sub geom index within each object
                   // for geom object: > 0 means attached body according to environment setting

  Triangle3e() {}

  Triangle3e(unsigned int p1, unsigned int p2, unsigned int p3)
  {
    set(p1, p2, p3);
  }

  inline void set(unsigned int p1, unsigned int p2, unsigned int p3)
  {
    vids[0] = p1; vids[1] = p2; vids[2] = p3;
  }

  inline unsigned int operator[](int i) const { return vids[i]; }

  inline unsigned int& operator[](int i) { return vids[i]; }
};

typedef Triangle3e Triangle;

/** \brief Simple triangle with three indices for 3 neighboring triangles */
struct Triangle3f
{
  unsigned int fids[3];

  Triangle3f()
  {
    set(-1, -1, -1);
  }

  Triangle3f(unsigned int fid0, unsigned int fid1, unsigned int fid2)
  {
    set(fid0, fid1, fid2);
  }

  inline void set(unsigned int fid0, unsigned int fid1, unsigned int fid2)
  {
    fids[0] = fid0; fids[1] = fid1; fids[2] = fid2;
  }

  inline unsigned int operator[](int i) const { return fids[i]; }

  inline unsigned int& operator[](int j) { return fids[j]; }
};

/** \brief Simple edge with two indices for its endpoints */
struct Edge2f
{
  unsigned int vids[2];
  unsigned int fids[2];

  Edge2f()
  {
    vids[0] = -1; vids[1] = -1;
    fids[0] = -1; fids[1] = -1;
  }

  Edge2f(unsigned int vid0, unsigned int vid1, unsigned int fid)
  {
    vids[0] = vid0;
    vids[1] = vid1;
    fids[0] = fid;
  }

  /** \brief Whether two edges are the same, assuming belongs to the same object */
  bool operator == (const Edge2f& other) const
  {
    return (vids[0] == other.vids[0]) && (vids[1] == other.vids[1]);
  }

  bool operator < (const Edge2f& other) const
  {
    if(vids[0] == other.vids[0])
      return vids[1] < other.vids[1];

    return vids[0] < other.vids[0];
  }

};

}


#endif
