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

#include "BV_fitter.h"
#include <limits>

namespace collision_checking
{

/** \brief Compute the covariance matrix for a set or subset of points. */
void getCovariance(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f M[3])
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  Vec3f S1;
  Vec3f S2[3];

  for(int i = 0; i < n; ++i)
  {
    const Vec3f& p = indirect_index ? ps[indices[i]] : ps[i];
    S1 += p;
    S2[0][0] += (p[0] * p[0]);
    S2[1][1] += (p[1] * p[1]);
    S2[2][2] += (p[2] * p[2]);
    S2[0][1] += (p[0] * p[1]);
    S2[0][2] += (p[0] * p[2]);
    S2[1][2] += (p[1] * p[2]);

    if(ps2) // another frame
    {
      const Vec3f& p = indirect_index ? ps2[indices[i]] : ps2[i];
      S1 += p;
      S2[0][0] += (p[0] * p[0]);
      S2[1][1] += (p[1] * p[1]);
      S2[2][2] += (p[2] * p[2]);
      S2[0][1] += (p[0] * p[1]);
      S2[0][2] += (p[0] * p[2]);
      S2[1][2] += (p[1] * p[2]);
    }
  }

  int n_points = ((ps2 == NULL) ? n : 2*n);

  M[0][0] = S2[0][0] - S1[0]*S1[0] / n_points;
  M[1][1] = S2[1][1] - S1[1]*S1[1] / n_points;
  M[2][2] = S2[2][2] - S1[2]*S1[2] / n_points;
  M[0][1] = S2[0][1] - S1[0]*S1[1] / n_points;
  M[1][2] = S2[1][2] - S1[1]*S1[2] / n_points;
  M[0][2] = S2[0][2] - S1[0]*S1[2] / n_points;
  M[1][0] = M[0][1];
  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
}

/** \brief Compute the covariance matrix for a set or subset of triangles. */
void getCovariance(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f M[3])
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  Vec3f S1;
  Vec3f S2[3];

  for(int i = 0; i < n; ++i)
  {
    const Triangle& t = indirect_index ? ts[indices[i]] : ts[i];

    const Vec3f& p1 = ps[t[0]];
    const Vec3f& p2 = ps[t[1]];
    const Vec3f& p3 = ps[t[2]];

    S1[0] += (p1[0] + p2[0] + p3[0]);
    S1[1] += (p1[1] + p2[1] + p3[1]);
    S1[2] += (p1[2] + p2[2] + p3[2]);

    S2[0][0] += (p1[0] * p1[0] +
                 p2[0] * p2[0] +
                 p3[0] * p3[0]);
    S2[1][1] += (p1[1] * p1[1] +
                 p2[1] * p2[1] +
                 p3[1] * p3[1]);
    S2[2][2] += (p1[2] * p1[2] +
                 p2[2] * p2[2] +
                 p3[2] * p3[2]);
    S2[0][1] += (p1[0] * p1[1] +
                 p2[0] * p2[1] +
                 p3[0] * p3[1]);
    S2[0][2] += (p1[0] * p1[2] +
                 p2[0] * p2[2] +
                 p3[0] * p3[2]);
    S2[1][2] += (p1[1] * p1[2] +
                 p2[1] * p2[2] +
                 p3[1] * p3[2]);

    if(ps2)
    {
      const Vec3f& p1 = ps2[t[0]];
      const Vec3f& p2 = ps2[t[1]];
      const Vec3f& p3 = ps2[t[2]];

      S1[0] += (p1[0] + p2[0] + p3[0]);
      S1[1] += (p1[1] + p2[1] + p3[1]);
      S1[2] += (p1[2] + p2[2] + p3[2]);

      S2[0][0] += (p1[0] * p1[0] +
                   p2[0] * p2[0] +
                   p3[0] * p3[0]);
      S2[1][1] += (p1[1] * p1[1] +
                   p2[1] * p2[1] +
                   p3[1] * p3[1]);
      S2[2][2] += (p1[2] * p1[2] +
                   p2[2] * p2[2] +
                   p3[2] * p3[2]);
      S2[0][1] += (p1[0] * p1[1] +
                   p2[0] * p2[1] +
                   p3[0] * p3[1]);
      S2[0][2] += (p1[0] * p1[2] +
                   p2[0] * p2[2] +
                   p3[0] * p3[2]);
      S2[1][2] += (p1[1] * p1[2] +
                   p2[1] * p2[2] +
                   p3[1] * p3[2]);
    }
  }

  int n_points = ((ps2 == NULL) ? 3*n : 6*n);

  M[0][0] = S2[0][0] - S1[0]*S1[0] / n_points;
  M[1][1] = S2[1][1] - S1[1]*S1[1] / n_points;
  M[2][2] = S2[2][2] - S1[2]*S1[2] / n_points;
  M[0][1] = S2[0][1] - S1[0]*S1[1] / n_points;
  M[1][2] = S2[1][2] - S1[1]*S1[2] / n_points;
  M[0][2] = S2[0][2] - S1[0]*S1[2] / n_points;
  M[1][0] = M[0][1];
  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
}


/** \brief Compute the eigen value and vector for a given matrix a. */
void Meigen(Vec3f a[3], BVH_REAL dout[3], Vec3f vout[3])
{
  int n = 3;
  int j, iq, ip, i;
  BVH_REAL tresh, theta, tau, t, sm, s, h, g, c;
  int nrot;
  BVH_REAL b[3];
  BVH_REAL z[3];
  BVH_REAL v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  BVH_REAL d[3];

  for(ip = 0; ip < n; ++ip)
  {
    b[ip] = a[ip][ip];
    d[ip] = a[ip][ip];
    z[ip] = 0.0;
  }

  nrot = 0;

  for(i = 0; i < 50; ++i)
  {
    sm = 0.0;
    for(ip = 0; ip < n; ++ip)
      for(iq = ip + 1; iq < n; ++iq)
        sm += fabs(a[ip][iq]);
    if(sm == 0.0)
    {
      vout[0] = v[0];
      vout[1] = v[1];
      vout[2] = v[2];
      dout[0] = d[0]; dout[1] = d[1]; dout[2] = d[2];
      return;
    }

    if(i < 3) tresh = 0.2 * sm / (n * n);
    else tresh = 0.0;

    for(ip = 0; ip < n; ++ip)
    {
      for(iq= ip + 1; iq < n; ++iq)
      {
        g = 100.0 * fabs(a[ip][iq]);
        if(i > 3 &&
            fabs(d[ip]) + g == fabs(d[ip]) &&
            fabs(d[iq]) + g == fabs(d[iq]))
          a[ip][iq] = 0.0;
        else if(fabs(a[ip][iq]) > tresh)
        {
          h = d[iq] - d[ip];
          if(fabs(h) + g == fabs(h)) t = (a[ip][iq]) / h;
          else
          {
            theta = 0.5 * h / (a[ip][iq]);
            t = 1.0 /(fabs(theta) + sqrt(1.0 + theta * theta));
            if(theta < 0.0) t = -t;
          }
          c = 1.0 / sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * a[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          d[ip] -= h;
          d[iq] += h;
          a[ip][iq] = 0.0;
          for(j = 0; j < ip; ++j) { g = a[j][ip]; h = a[j][iq]; a[j][ip] = g - s * (h + g * tau); a[j][iq] = h + s * (g - h * tau); }
          for(j = ip + 1; j < iq; ++j) { g = a[ip][j]; h = a[j][iq]; a[ip][j] = g - s * (h + g * tau); a[j][iq] = h + s * (g - h * tau); }
          for(j = iq + 1; j < n; ++j) { g = a[ip][j]; h = a[iq][j]; a[ip][j] = g - s * (h + g * tau); a[iq][j] = h + s * (g - h * tau); }
          for(j = 0; j < n; ++j) { g = v[j][ip]; h = v[j][iq]; v[j][ip] = g - s * (h + g * tau); v[j][iq] = h + s * (g - h * tau); }
          nrot++;
        }
      }
    }
    for(ip = 0; ip < n; ++ip)
    {
      b[ip] += z[ip];
      d[ip] = b[ip];
      z[ip] = 0.0;
    }
  }

  std::cerr << "eigen: too many iterations in Jacobi transform." << std::endl;

  return;
}


/** \brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin.
 * The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, BVH_REAL l[2], BVH_REAL& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = (ps2 == NULL) ? n : (2 * n);

  BVH_REAL (*P)[3] = new BVH_REAL[size_P][3];


  int P_id = 0;
  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vec3f& p = ps[index];
    Vec3f v(p[0], p[1], p[2]);
    P[P_id][0] = axis[0].dot(v);
    P[P_id][1] = axis[1].dot(v);
    P[P_id][2] = axis[2].dot(v);
    P_id++;

    if(ps2)
    {
      const Vec3f& v = ps2[index];
      P[P_id][0] = axis[0].dot(v);
      P[P_id][1] = axis[1].dot(v);
      P[P_id][2] = axis[2].dot(v);
      P_id++;
    }
  }

  BVH_REAL minx, maxx, miny, maxy, minz, maxz;

  BVH_REAL cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (BVH_REAL)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (BVH_REAL)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  BVH_REAL mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL x_value = P[i][0];
    if(x_value < mintmp)
    {
      minindex = i;
      mintmp = x_value;
    }
    else if(x_value > maxtmp)
    {
      maxindex = i;
      maxtmp = x_value;
    }
  }

  BVH_REAL x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - sqrt(std::max(radsqr - dz * dz, 0.0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL y_value = P[i][1];
    if(y_value < mintmp)
    {
      minindex = i;
      mintmp = y_value;
    }
    else if(y_value > maxtmp)
    {
      maxindex = i;
      maxtmp = y_value;
    }
  }

  BVH_REAL y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - sqrt(std::max(radsqr - dz * dz, 0.0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)

  BVH_REAL dx, dy, u, t;
  BVH_REAL a = sqrt((BVH_REAL)0.5);
  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - maxy;
        u = dx * a + dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if(u > 0)
        {
          maxx += u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - miny;
        u = dx * a - dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if(u > 0)
        {
          maxx += u*a;
          miny -= u*a;
        }
      }
    }
    else if(P[i][0] < minx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - maxy;
        u = dy * a - dx * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if(u > 0)
        {
          minx -= u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - miny;
        u = -dx * a - dy * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if (u > 0)
        {
          minx -= u*a;
          miny -= u*a;
        }
      }
    }
  }

  origin = axis[0] * minx + axis[1] * miny + axis[2] * cz;

  l[0] = maxx - minx;
  if(l[0] < 0) l[0] = 0;
  l[1] = maxy - miny;
  if(l[1] < 0) l[1] = 0;
}


/** \brief Compute the RSS bounding volume parameters: radius, rectangle size and the origin.
 * The bounding volume axes are known.
 */
void getRadiusAndOriginAndRectangleSize(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& origin, BVH_REAL l[2], BVH_REAL& r)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  int size_P = (ps2 == NULL) ? n * 3 : (2 * n * 3);

  BVH_REAL (*P)[3] = new BVH_REAL[size_P][3];


  int P_id = 0;
  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vec3f& p = ps[point_id];
      Vec3f v(p[0], p[1], p[2]);
      P[P_id][0] = axis[0].dot(v);
      P[P_id][1] = axis[1].dot(v);
      P[P_id][2] = axis[2].dot(v);
      P_id++;
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vec3f& p = ps2[point_id];
        Vec3f v(p[0], p[1], p[2]);
        P[P_id][0] = axis[0].dot(v);
        P[P_id][1] = axis[0].dot(v);
        P[P_id][2] = axis[1].dot(v);
        P_id++;
      }
    }
  }

  BVH_REAL minx, maxx, miny, maxy, minz, maxz;

  BVH_REAL cz, radsqr;

  minz = maxz = P[0][2];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL z_value = P[i][2];
    if(z_value < minz) minz = z_value;
    else if(z_value > maxz) maxz = z_value;
  }

  r = (BVH_REAL)0.5 * (maxz - minz);
  radsqr = r * r;
  cz = (BVH_REAL)0.5 * (maxz + minz);

  // compute an initial length of rectangle along x direction

  // find minx and maxx as starting points

  int minindex, maxindex;
  minindex = maxindex = 0;
  BVH_REAL mintmp, maxtmp;
  mintmp = maxtmp = P[0][0];

  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL x_value = P[i][0];
    if(x_value < mintmp)
    {
      minindex = i;
      mintmp = x_value;
    }
    else if(x_value > maxtmp)
    {
      maxindex = i;
      maxtmp = x_value;
    }
  }

  BVH_REAL x, dz;
  dz = P[minindex][2] - cz;
  minx = P[minindex][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxx = P[maxindex][0] - sqrt(std::max(radsqr - dz * dz, 0.0));


  // grow minx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] < minx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x < minx) minx = x;
    }
  }

  // grow maxx

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      dz = P[i][2] - cz;
      x = P[i][0] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(x > maxx) maxx = x;
    }
  }

  // compute an initial length of rectangle along y direction

  // find miny and maxy as starting points

  minindex = maxindex = 0;
  mintmp = maxtmp = P[0][1];
  for(int i = 1; i < size_P; ++i)
  {
    BVH_REAL y_value = P[i][1];
    if(y_value < mintmp)
    {
      minindex = i;
      mintmp = y_value;
    }
    else if(y_value > maxtmp)
    {
      maxindex = i;
      maxtmp = y_value;
    }
  }

  BVH_REAL y;
  dz = P[minindex][2] - cz;
  miny = P[minindex][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
  dz = P[maxindex][2] - cz;
  maxy = P[maxindex][1] - sqrt(std::max(radsqr - dz * dz, 0.0));

  // grow miny

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] < miny)
    {
      dz = P[i][2] - cz;
      y = P[i][1] + sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y < miny) miny = y;
    }
  }

  // grow maxy

  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][1] > maxy)
    {
      dz = P[i][2] - cz;
      y = P[i][1] - sqrt(std::max(radsqr - dz * dz, 0.0));
      if(y > maxy) maxy = y;
    }
  }

  // corners may have some points which are not covered - grow lengths if necessary
  // quite conservative (can be improved)
  BVH_REAL dx, dy, u, t;
  BVH_REAL a = sqrt((BVH_REAL)0.5);
  for(int i = 0; i < size_P; ++i)
  {
    if(P[i][0] > maxx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - maxy;
        u = dx * a + dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if(u > 0)
        {
          maxx += u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - maxx;
        dy = P[i][1] - miny;
        u = dx * a - dy * a;
        t = (a*u - dx)*(a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if(u > 0)
        {
          maxx += u*a;
          miny -= u*a;
        }
      }
    }
    else if(P[i][0] < minx)
    {
      if(P[i][1] > maxy)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - maxy;
        u = dy * a - dx * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (a*u - dy)*(a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if(u > 0)
        {
          minx -= u*a;
          maxy += u*a;
        }
      }
      else if(P[i][1] < miny)
      {
        dx = P[i][0] - minx;
        dy = P[i][1] - miny;
        u = -dx * a - dy * a;
        t = (-a*u - dx)*(-a*u - dx) +
            (-a*u - dy)*(-a*u - dy) +
            (cz - P[i][2])*(cz - P[i][2]);
        u = u - sqrt(std::max(radsqr - t, 0.0));
        if (u > 0)
        {
          minx -= u*a;
          miny -= u*a;
        }
      }
    }
  }

  origin = axis[0] * minx + axis[1] * miny + axis[2] * cz;

  l[0] = maxx - minx;
  if(l[0] < 0) l[0] = 0;
  l[1] = maxy - miny;
  if(l[1] < 0) l[1] = 0;
}

/** \brief Compute the bounding volume extent and center for a set or subset of points.
 * The bounding volume axes are known.
 */
void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  BVH_REAL real_max = std::numeric_limits<BVH_REAL>::max();

  BVH_REAL min_coord[3] = {real_max, real_max, real_max};
  BVH_REAL max_coord[3] = {-real_max, -real_max, -real_max};

  for(int i = 0; i < n; ++i)
  {
    int index = indirect_index ? indices[i] : i;

    const Vec3f& p = ps[index];
    Vec3f v(p[0], p[1], p[2]);
    BVH_REAL proj[3];
    proj[0] = axis[0].dot(v);
    proj[1] = axis[1].dot(v);
    proj[2] = axis[2].dot(v);

    for(int j = 0; j < 3; ++j)
    {
      if(proj[j] > max_coord[j]) max_coord[j] = proj[j];
      if(proj[j] < min_coord[j]) min_coord[j] = proj[j];
    }

    if(ps2)
    {
      const Vec3f& v = ps2[index];
      proj[0] = axis[0].dot(v);
      proj[1] = axis[1].dot(v);
      proj[2] = axis[2].dot(v);

      for(int j = 0; j < 3; ++j)
      {
        if(proj[j] > max_coord[j]) max_coord[j] = proj[j];
        if(proj[j] < min_coord[j]) min_coord[j] = proj[j];
      }
    }
  }

  Vec3f o = Vec3f((max_coord[0] + min_coord[0]) / 2,
                 (max_coord[1] + min_coord[1]) / 2,
                 (max_coord[2] + min_coord[2]) / 2);

  center = axis[0] * o[0] + axis[1] * o[1] + axis[2] * o[2];

  extent = Vec3f((max_coord[0] - min_coord[0]) / 2,
                 (max_coord[1] - min_coord[1]) / 2,
                 (max_coord[2] - min_coord[2]) / 2);

}


/** \brief Compute the bounding volume extent and center for a set or subset of points.
 * The bounding volume axes are known.
 */
void getExtentAndCenter(Vec3f* ps, Vec3f* ps2, Triangle* ts, unsigned int* indices, int n, Vec3f axis[3], Vec3f& center, Vec3f& extent)
{
  bool indirect_index = true;
  if(!indices) indirect_index = false;

  BVH_REAL real_max = std::numeric_limits<BVH_REAL>::max();

  BVH_REAL min_coord[3] = {real_max, real_max, real_max};
  BVH_REAL max_coord[3] = {-real_max, -real_max, -real_max};

  for(int i = 0; i < n; ++i)
  {
    unsigned int index = indirect_index? indices[i] : i;
    const Triangle& t = ts[index];

    for(int j = 0; j < 3; ++j)
    {
      int point_id = t[j];
      const Vec3f& p = ps[point_id];
      Vec3f v(p[0], p[1], p[2]);
      BVH_REAL proj[3];
      proj[0] = axis[0].dot(v);
      proj[1] = axis[1].dot(v);
      proj[2] = axis[2].dot(v);

      for(int k = 0; k < 3; ++k)
      {
        if(proj[k] > max_coord[k]) max_coord[k] = proj[k];
        if(proj[k] < min_coord[k]) min_coord[k] = proj[k];
      }
    }

    if(ps2)
    {
      for(int j = 0; j < 3; ++j)
      {
        int point_id = t[j];
        const Vec3f& p = ps2[point_id];
        Vec3f v(p[0], p[1], p[2]);
        BVH_REAL proj[3];
        proj[0] = axis[0].dot(v);
        proj[1] = axis[1].dot(v);
        proj[2] = axis[2].dot(v);

        for(int k = 0; k < 3; ++k)
        {
          if(proj[k] > max_coord[k]) max_coord[k] = proj[k];
          if(proj[k] < min_coord[k]) min_coord[k] = proj[k];
        }
      }
    }
  }

  Vec3f o = Vec3f((max_coord[0] + min_coord[0]) / 2,
                 (max_coord[1] + min_coord[1]) / 2,
                 (max_coord[2] + min_coord[2]) / 2);

  center = axis[0] * o[0] + axis[1] * o[1] + axis[2] * o[2];

  extent = Vec3f((max_coord[0] - min_coord[0]) / 2,
                 (max_coord[1] - min_coord[1]) / 2,
                 (max_coord[2] - min_coord[2]) / 2);

}






OBB BVFitter<OBB>::fit(Vec3f* ps, int n)
{
  switch(n)
  {
    case 1:
      return fit1(ps);
      break;
    case 2:
      return fit2(ps);
      break;
    case 3:
      return fit3(ps);
      break;
    case 6:
      return fit6(ps);
      break;
    default:
      return fitn(ps, n);
  }
}


OBB BVFitter<OBB>::fit(unsigned int* primitive_indices, int num_primitives)
{
  OBB bv;

  Vec3f M[3]; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3]; // three eigen values

  if(type == BVH_MODEL_TRIANGLES)
  {
    getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
    Meigen(M, s, E);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getCovariance(vertices, prev_vertices, primitive_indices, num_primitives, M);
    Meigen(M, s, E);
  }

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  Vec3f R[3]; // column first matrix, as the axis in OBB
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);


  // set obb axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set obb centers and extensions
  Vec3f center, extent;
  if(type == BVH_MODEL_TRIANGLES)
  {
    getExtentAndCenter(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, R, center, extent);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getExtentAndCenter(vertices, prev_vertices, primitive_indices, num_primitives, R, center, extent);
  }

  bv.To = center;
  bv.extent = extent;

  return bv;
}



OBB BVFitter<OBB>::fit1(Vec3f* ps)
{
  OBB bv;
  bv.To = ps[0];
  bv.axis[0] = Vec3f(1, 0, 0);
  bv.axis[1] = Vec3f(0, 1, 0);
  bv.axis[2] = Vec3f(0, 0, 1);
  bv.extent = Vec3f(0, 0, 0);
  return bv;
}

OBB BVFitter<OBB>::fit2(Vec3f* ps)
{
  OBB bv;
  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p1p2 = p1 - p2;
  float len_p1p2 = p1p2.length();
  Vec3f w = p1p2;
  w.normalize();

  // then generate other two axes orthonormal to w
  Vec3f u, v;
  float inv_length;
  if(fabs(w[0]) >= fabs(w[1]))
  {
    inv_length = 1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = 0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = 1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = 0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }

  bv.axis[0] = w;
  bv.axis[1] = u;
  bv.axis[2] = v;

  bv.extent = Vec3f(len_p1p2 * 0.5, 0, 0);
  bv.To = Vec3f(0.5 * (p1[0] + p2[0]),
                    0.5 * (p1[1] + p2[1]),
                    0.5 * (p1[2] + p2[2]));

  return bv;
}

OBB BVFitter<OBB>::fit3(Vec3f* ps)
{
  OBB bv;
  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p3(ps[2][0], ps[2][1], ps[2][2]);
  Vec3f e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  float len[3];
  len[0] = e[0].sqrLength();
  len[1] = e[1].sqrLength();
  len[2] = e[2].sqrLength();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  Vec3f w = e[0].cross(e[1]);
  w.normalize();
  Vec3f u = e[imax];
  u.normalize();
  Vec3f v = w.cross(u);
  bv.axis[0] = u;
  bv.axis[1] = v;
  bv.axis[2] = w;

  getExtentAndCenter(ps, NULL, NULL, 3, bv.axis, bv.To, bv.extent);

  return bv;
}

OBB BVFitter<OBB>::fit6(Vec3f* ps)
{
  OBB bv1, bv2;
  bv1 = fit3(ps);
  bv2 = fit3(ps + 3);
  return (bv1 + bv2);
}


OBB BVFitter<OBB>::fitn(Vec3f* ps, int n)
{
  OBB bv;

  Vec3f M[3]; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3] = {0, 0, 0}; // three eigen values

  getCovariance(ps, NULL, NULL, n, M);
  Meigen(M, s, E);

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  Vec3f R[3]; // column first matrix, as the axis in OBB
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);


  // set obb axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set obb centers and extensions
  Vec3f center, extent;
  getExtentAndCenter(ps, NULL, NULL, n, R, center, extent);

  bv.To = center;
  bv.extent = extent;

  return bv;
}

RSS BVFitter<RSS>::fit(Vec3f* ps, int n)
{
  switch(n)
  {
    case 1:
      return fit1(ps);
      break;
    case 2:
      return fit2(ps);
      break;
    case 3:
      return fit3(ps);
      break;
    default:
      return fit(ps, n);
  }
}


RSS BVFitter<RSS>::fit(unsigned int* primitive_indices, int num_primitives)
{
  RSS bv;

  Vec3f M[3]; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3]; // three eigen values

  if(type == BVH_MODEL_TRIANGLES)
  {
    getCovariance(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, M);
    Meigen(M, s, E);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getCovariance(vertices, prev_vertices, primitive_indices, num_primitives, M);
    Meigen(M, s, E);
  }

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  Vec3f R[3]; // column first matrix, as the axis in OBB
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set rss axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set rss origin, rectangle size and radius

  Vec3f origin;
  BVH_REAL l[2];
  BVH_REAL r;

  if(type == BVH_MODEL_TRIANGLES)
  {
    getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, tri_indices, primitive_indices, num_primitives, R, origin, l, r);
  }
  else if(type == BVH_MODEL_POINTCLOUD)
  {
    getRadiusAndOriginAndRectangleSize(vertices, prev_vertices, primitive_indices, num_primitives, R, origin, l, r);
  }

  bv.Tr = origin;
  bv.l[0] = l[0];
  bv.l[1] = l[1];
  bv.r = r;


  return bv;
}

RSS BVFitter<RSS>::fit1(Vec3f* ps)
{
  RSS bv;
  bv.Tr = ps[0];
  bv.axis[0] = Vec3f(1, 0, 0);
  bv.axis[1] = Vec3f(0, 1, 0);
  bv.axis[2] = Vec3f(0, 0, 1);
  bv.l[0] = 0;
  bv.l[1] = 0;
  bv.r = 0;

  return bv;
}

RSS BVFitter<RSS>::fit2(Vec3f* ps)
{
  RSS bv;

  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p1p2 = p1 - p2;
  float len_p1p2 = p1p2.length();
  Vec3f w = p1p2;
  w.normalize();

  // then generate other two axes orthonormal to w
  Vec3f u, v;
  float inv_length;
  if(fabs(w[0]) >= fabs(w[1]))
  {
    inv_length = 1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
    u[0] = -w[2] * inv_length;
    u[1] = 0;
    u[2] = w[0] * inv_length;
    v[0] = w[1] * u[2];
    v[1] = w[2] * u[0] - w[0] * u[2];
    v[2] = -w[1] * u[0];
  }
  else
  {
    inv_length = 1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
    u[0] = 0;
    u[1] = w[2] * inv_length;
    u[2] = -w[1] * inv_length;
    v[0] = w[1] * u[2] - w[2] * u[1];
    v[1] = -w[0] * u[2];
    v[2] = w[0] * u[1];
  }

  bv.axis[0] = w;
  bv.axis[1] = u;
  bv.axis[2] = v;

  bv.l[0] = len_p1p2;
  bv.l[1] = 0;

  bv.Tr = p2;

  return bv;
}

RSS BVFitter<RSS>::fit3(Vec3f* ps)
{
  RSS bv;

  Vec3f p1(ps[0][0], ps[0][1], ps[0][2]);
  Vec3f p2(ps[1][0], ps[1][1], ps[1][2]);
  Vec3f p3(ps[2][0], ps[2][1], ps[2][2]);
  Vec3f e[3];
  e[0] = p1 - p2;
  e[1] = p2 - p3;
  e[2] = p3 - p1;
  float len[3];
  len[0] = e[0].sqrLength();
  len[1] = e[1].sqrLength();
  len[2] = e[2].sqrLength();

  int imax = 0;
  if(len[1] > len[0]) imax = 1;
  if(len[2] > len[imax]) imax = 2;

  Vec3f w = e[0].cross(e[1]);
  w.normalize();
  Vec3f u = e[imax];
  u.normalize();
  Vec3f v = w.cross(u);
  bv.axis[0] = u;
  bv.axis[1] = v;
  bv.axis[2] = w;

  getRadiusAndOriginAndRectangleSize(ps, NULL, NULL, 3, bv.axis, bv.Tr, bv.l, bv.r);

  return bv;
}

RSS BVFitter<RSS>::fitn(Vec3f* ps, int n)
{
  RSS bv;

  Vec3f M[3]; // row first matrix
  Vec3f E[3]; // row first eigen-vectors
  BVH_REAL s[3] = {0, 0, 0};

  getCovariance(ps, NULL, NULL, n, M);
  Meigen(M, s, E);

  int min, mid, max;
  if(s[0] > s[1]) { max = 0; min = 1; }
  else { min = 0; max = 1; }
  if(s[2] < s[min]) { mid = min; min = 2; }
  else if(s[2] > s[max]) { mid = max; max = 2; }
  else { mid = 2; }

  Vec3f R[3]; // column first matrix, as the axis in RSS
  R[0] = Vec3f(E[0][max], E[1][max], E[2][max]);
  R[1] = Vec3f(E[0][mid], E[1][mid], E[2][mid]);
  R[2] = Vec3f(E[1][max]*E[2][mid] - E[1][mid]*E[2][max],
               E[0][mid]*E[2][max] - E[0][max]*E[2][mid],
               E[0][max]*E[1][mid] - E[0][mid]*E[1][max]);

  // set obb axes
  bv.axis[0] = R[0];
  bv.axis[1] = R[1];
  bv.axis[2] = R[2];

  // set rss origin, rectangle size and radius
  getRadiusAndOriginAndRectangleSize(ps, NULL, NULL, n, R, bv.Tr, bv.l, bv.r);

  return bv;
}


}
