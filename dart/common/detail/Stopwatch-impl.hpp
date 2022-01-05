/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/common/Stopwatch.hpp"

namespace dart::common {

namespace {
//==============================================================================
template <class duration_t>
void print_duration_if_non_zero(
    std::ostream& os,
    const duration_t& duration,
    const std::string& unit_postfix)
{
  if (duration == duration_t::zero())
  {
    return;
  }

  os << duration.count() << unit_postfix;
}
} // namespace

//==============================================================================
template <typename UnitType, typename ClockType>
Stopwatch<UnitType, ClockType>::Stopwatch(bool start)
  : mStart(ClockType::now()), mElapsed(0), mPaused(!start)
{
  // clang-format off
  static_assert(std::is_same_v<UnitType, std::chrono::seconds>
      || std::is_same_v<UnitType, std::chrono::milliseconds>
      || std::is_same_v<UnitType, std::chrono::microseconds>
      || std::is_same_v<UnitType, std::chrono::nanoseconds>,
      "Invalid unit");
  // clang-format on
}

//==============================================================================
template <typename UnitType, typename ClockType>
Stopwatch<UnitType, ClockType>::~Stopwatch()
{
  // Do nothing
}

//==============================================================================
template <typename UnitType, typename ClockType>
bool Stopwatch<UnitType, ClockType>::isStarted() const
{
  return !mPaused;
}

//==============================================================================
template <typename UnitType, typename ClockType>
void Stopwatch<UnitType, ClockType>::start()
{
  if (!mPaused)
  {
    return;
  }

  mStart = ClockType::now();
  mPaused = false;
}

//==============================================================================
template <typename UnitType, typename ClockType>
void Stopwatch<UnitType, ClockType>::stop()
{
  if (mPaused)
  {
    return;
  }

  mElapsed += std::chrono::duration_cast<UnitType>(ClockType::now() - mStart);
  mPaused = true;
}

//==============================================================================
template <typename UnitType, typename ClockType>
void Stopwatch<UnitType, ClockType>::reset()
{
  mStart = ClockType::now();
  mElapsed = UnitType(0);
}

//==============================================================================
template <typename UnitType, typename ClockType>
double Stopwatch<UnitType, ClockType>::elapsedS() const
{
  if constexpr (std::is_same_v<UnitType, std::chrono::nanoseconds>)
  {
    return duration().count() * 1e-9;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::microseconds>)
  {
    return duration().count() * 1e-6;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::milliseconds>)
  {
    return duration().count() * 1e-3;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::seconds>)
  {
    return duration().count();
  }
}

//==============================================================================
template <typename UnitType, typename ClockType>
double Stopwatch<UnitType, ClockType>::elapsedMS() const
{
  if constexpr (std::is_same_v<UnitType, std::chrono::nanoseconds>)
  {
    return duration().count() * 1e-6;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::microseconds>)
  {
    return duration().count() * 1e-3;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::milliseconds>)
  {
    return duration().count();
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::seconds>)
  {
    return duration().count() * 1e+3;
    ;
  }
}

//==============================================================================
template <typename UnitType, typename ClockType>
double Stopwatch<UnitType, ClockType>::elapsedUS() const
{
  if constexpr (std::is_same_v<UnitType, std::chrono::nanoseconds>)
  {
    return duration().count() * 1e-3;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::microseconds>)
  {
    return duration().count();
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::milliseconds>)
  {
    return duration().count() * 1e+3;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::seconds>)
  {
    return duration().count() * 1e+6;
    ;
  }
}

//==============================================================================
template <typename UnitType, typename ClockType>
double Stopwatch<UnitType, ClockType>::elapsedNS() const
{
  if constexpr (std::is_same_v<UnitType, std::chrono::nanoseconds>)
  {
    return duration().count();
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::microseconds>)
  {
    return duration().count() * 1e+3;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::milliseconds>)
  {
    return duration().count() * 1e+6;
  }
  else if constexpr (std::is_same_v<UnitType, std::chrono::seconds>)
  {
    return duration().count() * 1e+9;
  }
}

//==============================================================================
template <typename UnitType, typename ClockType>
UnitType Stopwatch<UnitType, ClockType>::duration() const
{
  if (mPaused)
  {
    return mElapsed;
  }

  const auto end = ClockType::now();
  const auto duration = std::chrono::duration_cast<UnitType>(end - mStart);
  return duration + mElapsed;
}

//==============================================================================
template <typename UnitType, typename ClockType>
void Stopwatch<UnitType, ClockType>::print(std::ostream& os) const
{
  auto t = duration();

  const auto h = std::chrono::duration_cast<std::chrono::hours>(t);
  print_duration_if_non_zero(os, h, "h ");
  t -= h;

  const auto m = std::chrono::duration_cast<std::chrono::minutes>(t);
  print_duration_if_non_zero(os, m, "min ");
  t -= m;

  const auto s = std::chrono::duration_cast<std::chrono::seconds>(t);
  print_duration_if_non_zero(os, s, "s ");
  t -= s;

  if constexpr (!std::is_same_v<UnitType, std::chrono::seconds>)
  {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t);
    print_duration_if_non_zero(os, ms, "ms ");
    t -= ms;

    if constexpr (!std::is_same_v<UnitType, std::chrono::milliseconds>)
    {
      const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t);
      print_duration_if_non_zero(os, us, "us ");
      t -= us;

      if constexpr (!std::is_same_v<UnitType, std::chrono::microseconds>)
      {
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t);
        print_duration_if_non_zero(os, ns, "ns ");
        if (us == std::chrono::nanoseconds::zero())
        {
          os << "0 ns\n";
        }
      }
      else if (us == std::chrono::microseconds::zero())
      {
        os << "0 us\n";
      }
    }
    else if (ms == std::chrono::milliseconds::zero())
    {
      os << "0 ms\n";
    }
  }
  else if (s == std::chrono::seconds::zero())
  {
    os << "0 s\n";
  }
}

//==============================================================================
template <typename UnitType, typename ClockType>
std::ostream& operator<<(
    std::ostream& os, const Stopwatch<UnitType, ClockType>& sw)
{
  sw.print(os);
  return os;
}

} // namespace dart::common
