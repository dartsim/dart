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

#include "dart/common/stopwatch.hpp"

namespace dart::common {

namespace {
auto sw = StopwatchNS(true);
}

//==============================================================================
void tic()
{
  sw.reset();
}

//==============================================================================
double toc(bool print)
{
  return toc_s(print);
}

//==============================================================================
double toc_s(bool print)
{
  if (!print) {
    return sw.elapsed_s();
  }

  sw.stop();
  const double elapsed = sw.elapsed_s();
  std::cout << "Elapsed time is " << elapsed << " s." << std::endl;
  sw.start();
  return elapsed;
}

//==============================================================================
double toc_ms(bool print)
{
  if (!print) {
    return sw.elapsed_ms();
  }

  sw.stop();
  const double elapsed = sw.elapsed_ms();
  std::cout << "Elapsed time is " << elapsed << " ms." << std::endl;
  sw.start();
  return elapsed;
}

//==============================================================================
double toc_us(bool print)
{
  if (!print) {
    return sw.elapsed_us();
  }

  sw.stop();
  const double elapsed = sw.elapsed_us();
  std::cout << "Elapsed time is " << elapsed << " us." << std::endl;
  sw.start();
  return elapsed;
}

//==============================================================================
double toc_ns(bool print)
{
  if (!print) {
    return sw.elapsed_ns();
  }

  sw.stop();
  const double elapsed = sw.elapsed_ns();
  std::cout << "Elapsed time is " << elapsed << " ns." << std::endl;
  sw.start();
  return elapsed;
}

} // namespace dart::common
