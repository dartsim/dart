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

#pragma once

#include "dart/multibody/single_body.hpp"

namespace dart::multibody {

//==============================================================================
template <typename Scalar>
SingleBody<Scalar>::SingleBody() : RelativeFrame<Scalar>()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void SingleBody<Scalar>::set_name(const std::string& name)
{
  m_name = name;
}

//==============================================================================
template <typename Scalar>
const std::string& SingleBody<Scalar>::get_name() const
{
  return m_name;
}

//==============================================================================
template <typename Scalar>
const math::SE3<Scalar>& SingleBody<Scalar>::get_local_pose() const
{
  return m_local_pose;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>& SingleBody<Scalar>::get_local_spatial_velocity()
    const
{
  return m_local_spatial_velocity;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>&
SingleBody<Scalar>::get_local_spatial_acceleration() const
{
  return m_local_spatial_acceleration;
}

//==============================================================================
template <typename Scalar>
void SingleBody<Scalar>::set_pose(const math::SE3<Scalar>& pose)
{
  m_local_pose = pose;
  this->dirty_pose();
}

} // namespace dart::multibody
