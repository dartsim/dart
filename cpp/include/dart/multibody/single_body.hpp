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

#include "dart/multibody/frame.hpp"

namespace dart::multibody {

template <typename Scalar_>
class SingleBody : public RelativeFrame<Scalar_>
{
public:
  using Scalar = Scalar_;

  SingleBody();

  // Implementation of Frame<Scalar>
  void set_name(const std::string& name) override;
  const std::string& get_name() const override;

private:
  // Implementation of Frame<Scalar>
  const math::SE3<Scalar>& get_local_pose() const override;
  const math::SE3Tangent<Scalar>& get_local_spatial_velocity() const override;
  const math::SE3Tangent<Scalar>& get_local_spatial_acceleration()
      const override;

public:
  void set_pose(const math::SE3<Scalar>& pose);

protected:
  std::string m_name;

  math::SE3<Scalar> m_local_pose;
  math::SE3Tangent<Scalar> m_local_spatial_velocity;
  math::SE3Tangent<Scalar> m_local_spatial_acceleration;

private:
};

} // namespace dart::multibody

#include "dart/multibody/detail/single_body_impl.hpp"
