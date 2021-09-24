/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_COLLISION_TRITRIINTERSECTIONTEST_HPP_
#define DART_COLLISION_TRITRIINTERSECTIONTEST_HPP_

#include <cmath>

/* some macros */
#define DART_CROSS(dest, v1, v2)                                               \
  dest[0] = v1[1] * v2[2] - v1[2] * v2[1];                                     \
  dest[1] = v1[2] * v2[0] - v1[0] * v2[2];                                     \
  dest[2] = v1[0] * v2[1] - v1[1] * v2[0];

#define DART_DOT(v1, v2) (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2])

#define DART_SUB(dest, v1, v2)                                                 \
  dest[0] = v1[0] - v2[0];                                                     \
  dest[1] = v1[1] - v2[1];                                                     \
  dest[2] = v1[2] - v2[2];

#define DART_ADD(dest, v1, v2)                                                 \
  dest[0] = v1[0] + v2[0];                                                     \
  dest[1] = v1[1] + v2[1];                                                     \
  dest[2] = v1[2] + v2[2];

#define DART_MULT(dest, v, factor)                                             \
  dest[0] = factor * v[0];                                                     \
  dest[1] = factor * v[1];                                                     \
  dest[2] = factor * v[2];

#define DART_DIV(dest, v1, v2)                                                 \
  dest[0] = v1[0] / v2[0];                                                     \
  dest[1] = v1[1] / 2 [1];                                                     \
  dest[2] = v1[2] / v2[2];

#define DART_SET(dest, src)                                                    \
  dest[0] = src[0];                                                            \
  dest[1] = src[1];                                                            \
  dest[2] = src[2];

/* sort so that a<=b */
#define DART_SORT(a, b)                                                        \
  if (a > b)                                                                   \
  {                                                                            \
    float c;                                                                   \
    c = a;                                                                     \
    a = b;                                                                     \
    b = c;                                                                     \
  }

#define DART_SWAP(a, b)                                                        \
  {                                                                            \
    float c;                                                                   \
    c = a;                                                                     \
    a = b;                                                                     \
    b = c;                                                                     \
  }

#define DART_ISECT(VV0, VV1, VV2, D0, D1, D2, isect0, isect1)                  \
  isect0 = VV0 + (VV1 - VV0) * D0 / (D0 - D1);                                 \
  isect1 = VV0 + (VV2 - VV0) * D0 / (D0 - D2);

#define DART_COMPUTE_INTERVALS(                                                \
    VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, isect0, isect1)                     \
  if (D0D1 > 0.0f)                                                             \
  {                                                                            \
    /* here we know that D0D2<=0.0 */                                          \
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
    DART_ISECT(VV2, VV0, VV1, D2, D0, D1, isect0, isect1);                     \
  }                                                                            \
  else if (D0D2 > 0.0f)                                                        \
  {                                                                            \
    /* here we know that d0d1<=0.0 */                                          \
    DART_ISECT(VV1, VV0, VV2, D1, D0, D2, isect0, isect1);                     \
  }                                                                            \
  else if (D1 * D2 > 0.0f || D0 != 0.0f)                                       \
  {                                                                            \
    /* here we know that d0d1<=0.0 or that D0!=0.0 */                          \
    DART_ISECT(VV0, VV1, VV2, D0, D1, D2, isect0, isect1);                     \
  }                                                                            \
  else if (D1 != 0.0f)                                                         \
  {                                                                            \
    DART_ISECT(VV1, VV0, VV2, D1, D0, D2, isect0, isect1);                     \
  }                                                                            \
  else if (D2 != 0.0f)                                                         \
  {                                                                            \
    DART_ISECT(VV2, VV0, VV1, D2, D0, D1, isect0, isect1);                     \
  }                                                                            \
  else                                                                         \
  {                                                                            \
    /* triangles are coplanar */                                               \
    return COPLANAR_CONTACT;                                                   \
  }

namespace dart {
namespace collision {

/* if USE_EPSILON_TEST is true then we do a check:
         if |dv|<EPSILON then dv=0.0;
   else no check is done (which is less robust)
*/
constexpr bool USE_EPSILON_TEST = true;
constexpr double EPSILON = 1e-6;

constexpr int NO_CONTACT = 0;
constexpr int COPLANAR_CONTACT = -1;
constexpr int INTERIAL_CONTACT = 1;

/* implement as is fastest on your machine */
inline float FABS(float x)
{
  return ((float)fabs(x));
}

inline void edge_tri_intersect(
    float V0[3], float V1[3], float DV0, float DV1, float V[3])
{
  float VV0[3], VV1[3];
  DART_MULT(VV0, V1, DV0);
  DART_MULT(VV1, V0, DV1);
  float U[3], D;
  DART_SUB(U, VV0, VV1);
  D = DV0 - DV1;
  DART_MULT(V, U, 1.0 / D);
}

inline int tri_tri_intersect(
    float V0[3],
    float V1[3],
    float V2[3],
    float U0[3],
    float U1[3],
    float U2[3],
    float res1[3],
    float res2[3])
{
  float E1[3], E2[3];
  float N1[3], N2[3], d1, d2;
  float du0, du1, du2, dv0, dv1, dv2;
  float D[3];
  float isect1[2], isect2[2];
  float du0du1, du0du2, dv0dv1, dv0dv2, du1du2, dv1dv2;
  short index;
  float vp0, vp1, vp2;
  float up0, up1, up2;
  float b, c, max;

  /* compute plane equation of triangle(V0,V1,V2) */
  DART_SUB(E1, V1, V0);
  DART_SUB(E2, V2, V0);
  DART_CROSS(N1, E1, E2);
  d1 = -DART_DOT(N1, V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the
   * plane*/
  du0 = DART_DOT(N1, U0) + d1;
  du1 = DART_DOT(N1, U1) + d1;
  du2 = DART_DOT(N1, U2) + d1;

  /* coplanarity robustness check */
#if USE_EPSILON_TEST == TRUE
  if (fabs(du0) < EPSILON)
    du0 = 0.0;
  if (fabs(du1) < EPSILON)
    du1 = 0.0;
  if (fabs(du2) < EPSILON)
    du2 = 0.0;
  if (du1 == 0 && du2 == 0 && fabs(du0) < 1e-4)
    du0 = 0.0;
  if (du0 == 0 && du2 == 0 && fabs(du1) < 1e-4)
    du1 = 0.0;
  if (du0 == 0 && du1 == 0 && fabs(du2) < 1e-4)
    du2 = 0.0;
#endif
  du0du1 = du0 * du1;
  du0du2 = du0 * du2;
  du1du2 = du1 * du2;

  if (du0du1 > 0.0f && du0du2 > 0.0f)
  {                    /* same sign on all of them + not equal 0 ? */
    return NO_CONTACT; /* no intersection occurs */
  }
  /* compute plane of triangle (U0,U1,U2) */
  DART_SUB(E1, U1, U0);
  DART_SUB(E2, U2, U0);
  DART_CROSS(N2, E1, E2);
  d2 = -DART_DOT(N2, U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0 = DART_DOT(N2, V0) + d2;
  dv1 = DART_DOT(N2, V1) + d2;
  dv2 = DART_DOT(N2, V2) + d2;

#if USE_EPSILON_TEST == TRUE
  if (fabs(dv0) < EPSILON)
    dv0 = 0.0;
  if (fabs(dv1) < EPSILON)
    dv1 = 0.0;
  if (fabs(dv2) < EPSILON)
    dv2 = 0.0;
  if (dv1 == 0 && dv2 == 0 && fabs(dv0) < 1e-5)
    dv0 = 0.0;
  if (dv0 == 0 && dv2 == 0 && fabs(dv1) < 1e-5)
    dv1 = 0.0;
  if (dv0 == 0 && dv1 == 0 && fabs(dv2) < 1e-5)
    dv2 = 0.0;
#endif
  dv0dv1 = dv0 * dv1;
  dv0dv2 = dv0 * dv2;
  dv1dv2 = dv1 * dv2;

  if (dv0dv1 > 0.0f && dv0dv2 > 0.0f)
  {                    /* same sign on all of them + not equal 0 ? */
    return NO_CONTACT; /* no intersection occurs */
  }
  /* compute direction of intersection line */
  DART_CROSS(D, N1, N2);

  /* compute and index to the largest component of D */
  max = fabs(D[0]);
  index = 0;
  b = fabs(D[1]);
  c = fabs(D[2]);
  if (b > max)
    max = b, index = 1;
  if (c > max)
    max = c, index = 2;

  /* this is the simplified projection onto L*/
  vp0 = V0[index];
  vp1 = V1[index];
  vp2 = V2[index];

  up0 = U0[index];
  up1 = U1[index];
  up2 = U2[index];

  /* compute interval for triangle 1 */
  DART_COMPUTE_INTERVALS(
      vp0, vp1, vp2, dv0, dv1, dv2, dv0dv1, dv0dv2, isect1[0], isect1[1]);

  /* compute interval for triangle 2 */
  DART_COMPUTE_INTERVALS(
      up0, up1, up2, du0, du1, du2, du0du1, du0du2, isect2[0], isect2[1]);

  DART_SORT(isect1[0], isect1[1]);
  DART_SORT(isect2[0], isect2[1]);

  // if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return NO_CONTACT;

  float res[4][3];
  if (du0du1 > 0)
  {
    edge_tri_intersect(U2, U0, du2, du0, res[0]);
    edge_tri_intersect(U2, U1, du2, du1, res[1]);
  }
  else if (du0du2 > 0)
  {
    edge_tri_intersect(U1, U0, du1, du0, res[0]);
    edge_tri_intersect(U1, U2, du1, du2, res[1]);
  }
  else if (du1du2 > 0)
  {
    edge_tri_intersect(U0, U1, du0, du1, res[0]);
    edge_tri_intersect(U0, U2, du0, du2, res[1]);
  }
  else if (du0 == 0)
  {
    DART_SET(res[0], U0);
    edge_tri_intersect(U1, U2, du1, du2, res[1]);
  }
  else if (du1 == 0)
  {
    DART_SET(res[0], U1);
    edge_tri_intersect(U0, U2, du0, du2, res[1]);
  }
  else if (du2 == 0)
  {
    DART_SET(res[0], U2);
    edge_tri_intersect(U0, U1, du0, du1, res[1]);
  }
  else
  {
    std::cerr << "contact error" << std::endl;
  }

  if (dv0dv1 > 0)
  {
    edge_tri_intersect(V2, V0, dv2, dv0, res[2]);
    edge_tri_intersect(V2, V1, dv2, dv1, res[3]);
  }
  else if (dv0dv2 > 0)
  {
    edge_tri_intersect(V1, V0, dv1, dv0, res[2]);
    edge_tri_intersect(V1, V2, dv1, dv2, res[3]);
  }
  else if (dv1dv2 > 0)
  {
    edge_tri_intersect(V0, V1, dv0, dv1, res[2]);
    edge_tri_intersect(V0, V2, dv0, dv2, res[3]);
  }
  else if (dv0 == 0)
  {
    DART_SET(res[2], V0);
    edge_tri_intersect(V1, V2, dv1, dv2, res[3]);
  }
  else if (dv1 == 0)
  {
    DART_SET(res[2], V1);
    edge_tri_intersect(V0, V2, dv0, dv2, res[3]);
  }
  else if (dv2 == 0)
  {
    DART_SET(res[2], V2);
    edge_tri_intersect(V0, V1, dv0, dv1, res[3]);
  }
  else
  {
    std::cerr << "contact error" << std::endl;
  }

  for (int i = 3; i > 0; i--)
    for (int j = 0; j < i; j++)
    {
      if (res[j][index] > res[j + 1][index])
      {
        for (int k = 0; k < 3; k++)
          DART_SWAP(res[j][k], res[j + 1][k]);
      }
    }
  DART_SET(res1, res[1]);
  DART_SET(res2, res[2]);

  return 1;
}

} // namespace collision
} // namespace dart

#endif // DART_COLLISION_TRITRIINTERSECTIONTEST_HPP_
