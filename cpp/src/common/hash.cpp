/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/common/hash.hpp"

#include <climits>

#include "dart/common/macro.hpp"

namespace dart::common {

//==============================================================================
// Thomas Wang's 32 Bit Mix Function:
// http://www.cris.com/~Ttwang/tech/inthash.htm
// https://c42f.github.io/2015/09/21/inverting-32-bit-wang-hash.html
uint32_t hash_thomas_wang(uint32_t key)
{
  // uint32_t k0 = key;
  // uint32_t k1 =
  //     k0 + ~(k0 << 15); // k1 = (1-(1<<15))*k0 - 1 = ~(((1<<15) - 1) * k0)
  // uint32_t k2 = k1 ^ (k1 >> 10);
  // uint32_t k3 = k2 + (k2 << 3); // k3 = 9*k2
  // uint32_t k4 = k3 ^ (k3 >> 6);
  // uint32_t k5 =
  //     k4 + ~(k4 << 11); // k5 = (1-(1<<11))*k4 - 1 = ~(((1<<11) - 1) * k4)
  // uint32_t k6 = k5 ^ (k5 >> 16);

  key += ~(key << 15);
  key ^= (key >> 10);
  key += (key << 3);
  key ^= (key >> 6);
  key += ~(key << 11);
  key ^= (key >> 16);
  return key;
}

//==============================================================================
uint32_t hash_thomas_wang(uint8_t key8)
{
  return hash_thomas_wang(static_cast<uint32_t>(key8));
}

//==============================================================================
uint32_t hash_thomas_wang(uint16_t key8)
{
  return hash_thomas_wang(static_cast<uint32_t>(key8));
}

//==============================================================================
uint32_t hash_pair_thomas_wang(uint16_t a, uint16_t b)
{
  uint32_t key = a | (static_cast<uint32_t>(b) << CHAR_BIT * sizeof(b));
  return hash_thomas_wang(key);
}

//==============================================================================
uint32_t hash_pair_thomas_wang(uint32_t a, uint32_t b)
{
  DART_ASSERT(a >> CHAR_BIT * sizeof(uint16_t) == 0);
  DART_ASSERT(b >> CHAR_BIT * sizeof(uint16_t) == 0);
  uint16_t a32 = a;
  uint16_t b32 = b;
  return hash_pair_thomas_wang(a32, b32);
}

} // namespace dart::common
