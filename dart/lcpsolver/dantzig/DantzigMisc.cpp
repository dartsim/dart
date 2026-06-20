/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

/*
 * This file contains code derived from Open Dynamics Engine (ODE).
 * Original copyright notice:
 *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of EITHER:
 *   (1) The GNU Lesser General Public License as published by the Free
 *       Software Foundation; either version 2.1 of the License, or (at
 *       your option) any later version. The text of the GNU Lesser
 *       General Public License is included with this library in the
 *       file LICENSE.TXT.
 *   (2) The BSD-style license that is included with this library in
 *       the file LICENSE-BSD.TXT.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.
 */

#include "dart/lcpsolver/dantzig/DantzigMisc.hpp"

namespace dart::lcpsolver::dantzig {

//==============================================================================
// Random number generation
//==============================================================================

static unsigned long rng_seed = 0;

unsigned long dRand()
{
  rng_seed = (rng_seed * 1103515245 + 12345) & 0xffffffff;
  return rng_seed;
}

unsigned long dRandGetSeed()
{
  return rng_seed;
}

void dRandSetSeed(unsigned long s)
{
  rng_seed = s;
}

int dRandInt(int n)
{
  DART_ASSERT(n > 0);
  const unsigned long un = n;
  volatile unsigned long rawR = dRand();
  unsigned long r = rawR;

  if (un <= 0x00000010UL) {
    r ^= (r >> 16);
    r ^= (r >> 8);
    r ^= (r >> 4);
    if (un <= 0x00000002UL) {
      r ^= (r >> 2);
      r ^= (r >> 1);
    } else if (un <= 0x00000004UL) {
      r ^= (r >> 2);
    }
  } else if (un <= 0x00000100UL) {
    r ^= (r >> 16);
    r ^= (r >> 8);
  } else if (un <= 0x00010000UL) {
    r ^= (r >> 16);
  }

  return static_cast<int>(r % un);
}

double dRandReal()
{
  return static_cast<double>(dRand()) / static_cast<double>(0xffffffff);
}

} // namespace dart::lcpsolver::dantzig
