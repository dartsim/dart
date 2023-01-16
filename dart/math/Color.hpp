/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#pragma once

#include <dart/math/Fwd.hpp>
#include <dart/math/Random.hpp>

namespace dart::math {

template <typename S>
class Color
{
public:
  using Scalar = S;

  static Vector4<S> Red(S alpha)
  {
    return Vector4<S>(0.9, 0.1, 0.1, alpha);
  }

  static Vector3<S> Red()
  {
    return Vector3<S>(0.9, 0.1, 0.1);
  }

  static Vector3<S> Fuchsia()
  {
    return Vector3<S>(1.0, 0.0, 0.5);
  }

  static Vector4<S> Fuchsia(S alpha)
  {
    return Vector4<S>(1.0, 0.0, 0.5, alpha);
  }

  static Vector4<S> Orange(S alpha)
  {
    return Vector4<S>(1.0, 0.63, 0.0, alpha);
  }

  static Vector3<S> Orange()
  {
    return Vector3<S>(1.0, 0.63, 0.0);
  }

  static Vector4<S> Green(S alpha)
  {
    return Vector4<S>(0.1, 0.9, 0.1, alpha);
  }

  static Vector3<S> Green()
  {
    return Vector3<S>(0.1, 0.9, 0.1);
  }

  static Vector4<S> Blue(S alpha)
  {
    return Vector4<S>(0.1, 0.1, 0.9, alpha);
  }

  static Vector3<S> Blue()
  {
    return Vector3<S>(0.1, 0.1, 0.9);
  }

  static Vector4<S> White(S alpha)
  {
    return Vector4<S>(1.0, 1.0, 1.0, alpha);
  }

  static Vector3<S> White()
  {
    return Vector3<S>(1.0, 1.0, 1.0);
  }

  static Vector4<S> Black(S alpha)
  {
    return Vector4<S>(0.05, 0.05, 0.05, alpha);
  }

  static Vector3<S> Black()
  {
    return Vector3<S>(0.05, 0.05, 0.05);
  }

  static Vector4<S> LightGray(S alpha)
  {
    return Vector4<S>(0.9, 0.9, 0.9, alpha);
  }

  static Vector3<S> LightGray()
  {
    return Vector3<S>(0.9, 0.9, 0.9);
  }

  static Vector4<S> Gray(S alpha)
  {
    return Vector4<S>(0.6, 0.6, 0.6, alpha);
  }

  static Vector3<S> Gray()
  {
    return Vector3<S>(0.6, 0.6, 0.6);
  }

  static Vector4<S> Random(S alpha)
  {
    return Vector4<S>(
        Random::uniform(0.0, 1.0),
        Random::uniform(0.0, 1.0),
        Random::uniform(0.0, 1.0),
        alpha);
  }

  static Vector3<S> Random()
  {
    return Vector3<S>(
        Random::uniform(0.0, 1.0),
        Random::uniform(0.0, 1.0),
        Random::uniform(0.0, 1.0));
  }
};

} // namespace dart::math
