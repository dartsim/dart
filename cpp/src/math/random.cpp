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

#include "dart/math/random.hpp"

namespace dart {
namespace math {

//==============================================================================
Random::GeneratorType& Random::getGenerator()
{
  static GeneratorType randGenerator(getSeed());
  return randGenerator;
}

//==============================================================================
void Random::setSeed(unsigned int seed)
{
  std::seed_seq seq{seed};
  getSeedMutable() = seed;
  getGenerator().seed(seq);
}

//==============================================================================
unsigned int Random::generateSeed(bool applyGeneratedSeed)
{
  const unsigned int seed = std::random_device{}();
  if (applyGeneratedSeed)
    setSeed(seed);
  return seed;
}

//==============================================================================
unsigned int Random::getSeed()
{
  return getSeedMutable();
}

//==============================================================================
unsigned int& Random::getSeedMutable()
{
  static uint32_t seed = generateSeed(false);
  return seed;
}

} // namespace math
} // namespace dart
