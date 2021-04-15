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

#ifndef DART_IO_MJCF_DETAIL_SIZE_HPP_
#define DART_IO_MJCF_DETAIL_SIZE_HPP_

#include <tinyxml2.h>

#include "dart/io/mjcf/detail/Error.hpp"

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

class Size final
{
public:
  Size() = default;

  int getMJMax() const;
  int getNConMax() const;
  int getNStack() const;
  int getNUserData() const;
  int getNKey() const;
  int getNUserBody() const;
  int getNUserJnt() const;
  int getNUserGeom() const;
  int getNUserSite() const;
  int getNUserCam() const;
  int getNUserTendon() const;
  int getNUserActuator() const;
  int getNUserSensor() const;

private:
  // Private memebers used by MujocoModel class
  friend class MujocoModel;
  Errors read(tinyxml2::XMLElement* element);

private:
  int mMJMax{-1};
  int mNConMax{-1};
  int mNStack{-1};
  int mNUserData{0};
  int mNKey{0};
  int mNUserBody{0};
  int mNUserJnt{0};
  int mNUserGeom{0};
  int mNUserSite{0};
  int mNUserCam{0};
  int mNUserTendon{0};
  int mNUserActuator{0};
  int mNUserSensor{0};
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_IO_MJCF_DETAIL_SIZE_HPP_
