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

#include "dart/math/lie_group/function.hpp"
#include "dart/multibody/frame.hpp"

namespace dart::multibody {

//==============================================================================
template <typename Scalar>
Frame<Scalar>* Frame<Scalar>::GetInertialFrame()
{
  static InertialFrame<Scalar> inertial_frame;
  return &inertial_frame;
}

//==============================================================================
template <typename Scalar>
Frame<Scalar>::Frame()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Frame<Scalar>::~Frame()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool Frame<Scalar>::is_inertial_frame() const
{
  return false;
}

//==============================================================================
template <typename Scalar>
math::SE3<Scalar> Frame<Scalar>::get_pose(
    const Frame<Scalar>* with_respect_to) const
{
  if (!with_respect_to) {
    DART_ERROR(
        "Invalid parameter. Not allowed nullptr for with_respect_to. Returning "
        "the global pose of [{}] instead.",
        this->get_name());
    return get_pose();
  }

  if (with_respect_to->is_inertial_frame()) {
    return get_pose();
  } else if (with_respect_to == this) {
    return math::SE3<Scalar>::Identity();
  }

  return with_respect_to->get_pose().inverse() * get_pose();
}

//==============================================================================
template <typename Scalar>
const Eigen::Map<const math::SO3<Scalar>> Frame<Scalar>::get_orientation() const
{
  return get_pose().orientation();
}

//==============================================================================
template <typename Scalar>
const Eigen::Map<const math::R3<Scalar>> Frame<Scalar>::get_position() const
{
  return get_pose().position();
}

//==============================================================================
template <typename Scalar>
math::SE3Tangent<Scalar> Frame<Scalar>::get_spatial_velocity(
    const Frame<Scalar>* relative_to,
    const Frame<Scalar>* in_coordinates_of) const
{
  if (relative_to == this) {
    return math::SE3Tangent<Scalar>();
  }

  if (relative_to->is_inertial_frame()) {
    if (in_coordinates_of == this) {
      return get_spatial_velocity();
    }

    if (in_coordinates_of->is_inertial_frame()) {
      return math::Ad_R(get_pose(), get_spatial_velocity());
    }

    return math::Ad_R(get_pose(in_coordinates_of), get_spatial_velocity());
  }

  const math::SE3Tangent<Scalar> out
      = get_spatial_velocity()
        - math::Ad(
            relative_to->get_pose(this), relative_to->get_spatial_velocity());

  if (in_coordinates_of == this) {
    return out;
  }

  return math::Ad_R(get_pose(in_coordinates_of), out);
}

//==============================================================================
template <typename Scalar>
math::SE3Tangent<Scalar> Frame<Scalar>::get_spatial_velocity(
    const math::R3<Scalar>& offset, const Frame<Scalar>* relative_to) const
{
  return get_spatial_velocity(offset, relative_to, this);
}

//==============================================================================
template <typename Scalar>
math::SE3Tangent<Scalar> Frame<Scalar>::get_spatial_velocity(
    const math::R3<Scalar>& offset,
    const Frame<Scalar>* relative_to,
    const Frame<Scalar>* in_coordinates_of) const
{
  if (relative_to == this) {
    return math::SE3Tangent<Scalar>();
  }

  math::SE3Tangent<Scalar> V = get_spatial_velocity();
  V.linear_coeffs() = V.angular_coeffs().cross(offset.coeffs());
  // TODO(JS): Implement .noalias() for V.linear()

  if (relative_to->is_inertial_frame()) {
    if (in_coordinates_of == this) {
      return V;
    }

    if (in_coordinates_of->is_inertial_frame()) {
      return math::Ad_R(get_pose(), V);
    }

    return math::Ad_R(get_pose(in_coordinates_of), V);
  }

  math::SE3Tangent<Scalar> V0 = math::Ad(
      relative_to->get_pose(this), relative_to->get_spatial_velocity());
  V0.linear_coeffs() += V0.angular_coeffs().cross(offset.coeffs());
  // TODO(JS): Implement .noalias() for V.linear()

  V -= V0;

  if (in_coordinates_of == this) {
    return V;
  }

  return math::Ad_R(get_pose(in_coordinates_of), V);
}

//==============================================================================
template <typename Scalar>
math::SO3Tangent<Scalar> Frame<Scalar>::get_angular_velocity(
    const Frame* relative_to, const Frame* in_coordinates_of) const
{
  return get_spatial_velocity(relative_to, in_coordinates_of).angular_coeffs();
}

//==============================================================================
template <typename Scalar>
math::R3Tangent<Scalar> Frame<Scalar>::get_linear_velocity(
    const Frame* relative_to, const Frame* in_coordinates_of) const
{
  return get_spatial_velocity(relative_to, in_coordinates_of).linear_coeffs();
}

//==============================================================================
template <typename Scalar>
math::R3Tangent<Scalar> Frame<Scalar>::get_linear_velocity(
    const math::R3<Scalar>& offset,
    const Frame* relative_to,
    const Frame* in_coordinates_of) const
{
  return get_spatial_velocity(offset, relative_to, in_coordinates_of)
      .linear_coeffs();
}

//==============================================================================
template <typename Scalar>
math::SO3Tangent<Scalar> Frame<Scalar>::get_angular_acceleration() const
{
  return get_spatial_acceleration().angular();
}

//==============================================================================
template <typename Scalar>
math::R3Tangent<Scalar> Frame<Scalar>::get_linear_acceleration() const
{
  return get_spatial_acceleration().linear();
}

//==============================================================================
template <typename Scalar>
void Frame<Scalar>::dirty_pose()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void Frame<Scalar>::dirty_spatial_velocity()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void Frame<Scalar>::dirty_spatial_acceleration()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
const math::SE3<Scalar> InertialFrame<Scalar>::m_identity_pose;
template <typename Scalar>
const math::SE3Tangent<Scalar> InertialFrame<Scalar>::m_zero_spatial_motion;

//==============================================================================
template <typename Scalar>
const std::string& InertialFrame<Scalar>::GetName()
{
  static const std::string name = "Inertial Frame";
  return name;
}

//==============================================================================
template <typename Scalar>
InertialFrame<Scalar>::InertialFrame() : Frame<Scalar>()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void InertialFrame<Scalar>::set_name(const std::string& /*name*/)
{
  DART_ERROR("Not allowed to set name of the inertial frame");
}

//==============================================================================
template <typename Scalar>
bool InertialFrame<Scalar>::is_inertial_frame() const
{
  return true;
}

//==============================================================================
template <typename Scalar>
const std::string& InertialFrame<Scalar>::get_name() const
{
  return GetName();
}

//==============================================================================
template <typename Scalar>
const math::SE3<Scalar>& InertialFrame<Scalar>::get_pose() const
{
  return m_identity_pose;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>& InertialFrame<Scalar>::get_spatial_velocity()
    const
{
  return m_zero_spatial_motion;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>&
InertialFrame<Scalar>::get_spatial_acceleration() const
{
  return m_zero_spatial_motion;
}

//==============================================================================
template <typename Scalar>
FreeFrame<Scalar>::FreeFrame()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void FreeFrame<Scalar>::set_name(const std::string& name)
{
  m_name = name;
}

//==============================================================================
template <typename Scalar>
const std::string& FreeFrame<Scalar>::get_name() const
{
  return m_name;
}

//==============================================================================
template <typename Scalar>
const math::SE3<Scalar>& FreeFrame<Scalar>::get_pose() const
{
  return m_pose;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>& FreeFrame<Scalar>::get_spatial_velocity() const
{
  return m_spatial_velocity;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>& FreeFrame<Scalar>::get_spatial_acceleration()
    const
{
  return m_spatial_acceleration;
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_pose(const math::SE3Base<Derived>& pose)
{
  m_pose = pose;
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_orientation(
    const math::SO3Base<Derived>& orientation)
{
  m_pose.orientation() = orientation;
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_orientation(math::SO3Base<Derived>&& orientation)
{
  m_pose.orientation() = std::move(orientation);
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_position(const math::RBase<Derived>& position)
{
  m_pose.position() = position;
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_position(math::RBase<Derived>&& position)
{
  m_pose.position() = std::move(position);
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_spatial_velocity(
    const math::SE3TangentBase<Derived>& spatial_velocity)
{
  m_spatial_velocity = spatial_velocity;
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_angular_velocity(
    const math::SO3TangentBase<Derived>& angular_velocity)
{
  m_spatial_velocity.angular_coeffs() = angular_velocity;
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_linear_velocity(
    const math::RTangentBase<Derived>& linear_velocity)
{
  m_spatial_velocity.linear_coeffs() = linear_velocity;
}

//==============================================================================
template <typename Scalar>
template <typename Derived>
void FreeFrame<Scalar>::set_spatial_acceleration(
    const math::SE3TangentBase<Derived>& spatial_acceleration)
{
  m_spatial_acceleration = spatial_acceleration;
}

//==============================================================================
template <typename Scalar>
RelativeFrame<Scalar>::RelativeFrame(Frame<Scalar>* parent_frame)
  : Frame<Scalar>(), m_parent_frame(parent_frame)
{
  DART_ASSERT(parent_frame);
}

//==============================================================================
template <typename Scalar>
const Frame<Scalar>* RelativeFrame<Scalar>::get_parent_frame() const
{
  return m_parent_frame;
}

//==============================================================================
template <typename Scalar>
Frame<Scalar>* RelativeFrame<Scalar>::get_mutable_parent_frame()
{
  return m_parent_frame;
}

//==============================================================================
template <typename Scalar>
int RelativeFrame<Scalar>::get_num_child_frames() const
{
  return m_child_frames.size();
}

//==============================================================================
template <typename Scalar>
const Frame<Scalar>* RelativeFrame<Scalar>::get_child_frame_by_index(
    int index) const
{
  if (index < 0 || m_child_frames.size() <= static_cast<std::size_t>(index)) {
    DART_ERROR(
        "Invalid index of child frame [{}] when the number of child frames is "
        "[{}]. Returning nullptr.",
        index,
        m_child_frames.size());
    return nullptr;
  }

  return m_child_frames[index];
}

//==============================================================================
template <typename Scalar>
const Frame<Scalar>* RelativeFrame<Scalar>::get_child_frame_by_name(
    const std::string& name) const
{
  const auto it = std::find_if(
      m_child_frames.begin(),
      m_child_frames.end(),
      [&](const Frame<Scalar>* child_frame) {
        return (child_frame->get_name() == name);
      });

  return it != m_child_frames.end() ? (*it) : nullptr;
}

//==============================================================================
template <typename Scalar>
const math::SE3<Scalar>& RelativeFrame<Scalar>::get_pose() const
{
  if (this->is_inertial_frame()) {
    return m_pose;
  }

  if (m_pose_dirty) {
    m_pose = m_parent_frame->get_pose() * get_local_pose();
    m_pose_dirty = false;
  }

  return m_pose;
}

//==============================================================================
template <typename Scalar>
math::SE3<Scalar> RelativeFrame<Scalar>::get_pose(
    const Frame<Scalar>* with_respect_to) const
{
  if (with_respect_to == m_parent_frame) {
    return get_local_pose();
  }

  return Frame<Scalar>::get_pose(with_respect_to);
}

//==============================================================================
template <typename Scalar>
math::SE3<Scalar> RelativeFrame<Scalar>::get_pose(
    const Frame<Scalar>* with_respect_to,
    const Frame<Scalar>* in_coordinates_of) const
{
  if (!with_respect_to) {
    DART_ERROR(
        "Invalid parameter. Not allowed nullptr for with_respect_to. Returning "
        "the global pose of [{}] instead.",
        this->get_name());
    return get_pose();
  }

  if (!in_coordinates_of) {
    DART_ERROR(
        "Invalid parameter. Not allowed nullptr for in_coordinates_of. "
        "Returning the global pose of [{}] instead.",
        this->get_name());
    return get_pose();
  }

  if (with_respect_to == in_coordinates_of) {
    return get_pose(with_respect_to);
  }

  const math::SE3<Scalar> reference = math::SE3<Scalar>(
      in_coordinates_of->get_orientation(), with_respect_to->get_position());
  return reference.inverse() * get_pose();
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>& RelativeFrame<Scalar>::get_spatial_velocity()
    const
{
  if (this->is_inertial_frame()) {
    return m_spatial_velocity;
  }

  if (m_spatial_velocity_dirty) {
    m_spatial_velocity = math::Ad(
                             get_local_pose().inverse(),
                             get_parent_frame()->get_spatial_velocity())
                         + get_local_spatial_velocity();
    m_spatial_velocity_dirty = false;
  }

  return m_spatial_velocity;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>&
RelativeFrame<Scalar>::get_spatial_acceleration() const
{
  if (this->is_inertial_frame()) {
    return m_spatial_acceleration;
  }

  if (m_spatial_acceleration_dirty) {
    m_spatial_acceleration
        = math::Ad(
              get_local_pose().inverse(),
              get_parent_frame()->get_spatial_acceleration())
          + math::ad(get_spatial_velocity(), get_local_spatial_velocity())
          + get_local_spatial_acceleration();

    m_spatial_acceleration_dirty = false;
  }

  return m_spatial_acceleration;
}

//==============================================================================
template <typename Scalar>
void RelativeFrame<Scalar>::dirty_pose()
{
  if (m_pose_dirty) {
    return;
  }

  dirty_spatial_velocity();

  m_pose_dirty = true;

  for (Frame<Scalar>* child_frame : m_child_frames) {
    child_frame->dirty_pose();
  }
}

//==============================================================================
template <typename Scalar>
void RelativeFrame<Scalar>::dirty_spatial_velocity()
{
  if (m_spatial_velocity_dirty) {
    return;
  }

  dirty_spatial_acceleration();

  m_spatial_velocity_dirty = true;

  for (Frame<Scalar>* child_frame : m_child_frames) {
    child_frame->dirty_spatial_velocity();
  }
}

//==============================================================================
template <typename Scalar>
void RelativeFrame<Scalar>::dirty_spatial_acceleration()
{
  if (m_spatial_acceleration_dirty) {
    return;
  }

  m_spatial_acceleration_dirty = true;

  for (Frame<Scalar>* child_frame : m_child_frames) {
    child_frame->dirty_spatial_acceleration();
  }
}

//==============================================================================
template <typename Scalar>
FreeRelativeFrame<Scalar>::FreeRelativeFrame()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void FreeRelativeFrame<Scalar>::set_name(const std::string& name)
{
  m_name = name;
}

//==============================================================================
template <typename Scalar>
const std::string& FreeRelativeFrame<Scalar>::get_name() const
{
  return m_name;
}

//==============================================================================
template <typename Scalar>
const math::SE3<Scalar>& FreeRelativeFrame<Scalar>::get_local_pose() const
{
  return m_local_pose;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>&
FreeRelativeFrame<Scalar>::get_local_spatial_velocity() const
{
  return m_local_spatial_velocity;
}

//==============================================================================
template <typename Scalar>
const math::SE3Tangent<Scalar>&
FreeRelativeFrame<Scalar>::get_local_spatial_acceleration() const
{
  return m_local_spatial_acceleration;
}

} // namespace dart::multibody
