/*
 * Copyright (c) 2011-2022, The DART development contributors:
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

#include <string>
#include <unordered_set>

#include "dart/math/all.hpp"
#include "dart/multibody/export.hpp"
#include "dart/multibody/type.hpp"

namespace dart::multibody {

// Forward declaration
template <typename Scalar>
class InertialFrame;
template <typename Scalar>
class RelativeFrame;

template <typename Scalar>
class Frame
{
public:
  friend class InertialFrame<Scalar>;

  /// Returns the static inertial frame instance
  static Frame* GetInertialFrame();

  /// Default constructor.
  Frame();

  /// Destructor.
  virtual ~Frame();

  /// Sets the frame name.
  ///
  /// @param[in] name: Frame name.
  virtual void set_name(const std::string& name) = 0;

  /// Returns the fame name.
  [[nodiscard]] virtual const std::string& get_name() const = 0;

  /// Returns true if this frame is the inertial frame.
  [[nodiscard]] virtual bool is_inertial_frame() const;

  /// @{ @name Pose

  /// Returns the global pose in the coordinates of the inertial frame.
  [[nodiscard]] virtual const math::SE3<Scalar>& get_pose() const = 0;

  /// Returns the relative pose with respect to @c with_respect_to in the
  /// coordinates of @c with_respect_to.
  ///
  /// The output is defined as @f$ T = T_B^{-1} * T_A @f$ where @f$ T_A @f$ is
  /// the global pose of this frame, and @f$ T_B @f$ is with_respect_to.
  ///
  /// @param[in] with_respect_to: The reference frame to compute the relative
  /// pose.
  [[nodiscard]] virtual math::SE3<Scalar> get_pose(
      const Frame<Scalar>* with_respect_to) const;

  /// Returns the global orientation of this frame.
  [[nodiscard]] const Eigen::Map<const math::SO3<Scalar>> get_orientation()
      const;

  /// Returns the global position of this frame.
  [[nodiscard]] const Eigen::Map<const math::R3<Scalar>> get_position() const;

  /// @}

  /// @{ @name Velocity

  /// Returns the global spatial velocity expressed in this frame.
  [[nodiscard]] virtual const math::SE3Tangent<Scalar>& get_spatial_velocity()
      const = 0;

  /// Returns the global spatial velocity in the coordinates of a given frame.
  ///
  /// @param[in] relative_to: The reference frame of the relative velocity.
  /// @param[in] in_coordinates_of: The reference frame that the output is
  /// expressed in.
  [[nodiscard]] math::SE3Tangent<Scalar> get_spatial_velocity(
      const Frame<Scalar>* relative_to,
      const Frame<Scalar>* in_coordinates_of) const;

  /// Returns the global spatial velocity in the coordinates of a given frame.
  ///
  /// @param[in] offset: The offset of the point of interest from the origin of
  /// the frame, where the offset is in the coordinates of this frame.
  /// @param[in] relative_to: The reference frame of the relative velocity.
  [[nodiscard]] math::SE3Tangent<Scalar> get_spatial_velocity(
      const math::R3<Scalar>& offset,
      const Frame* relative_to = Frame::GetInertialFrame()) const;

  /// Returns the global spatial velocity in the coordinates of a given frame.
  ///
  /// @param[in] offset: The offset of the point of interest from the origin of
  /// the frame, where the offset is in the coordinates of this frame.
  /// @param[in] relative_to: The reference frame of the relative velocity.
  /// @param[in] in_coordinates_of: The reference frame that the output is
  /// expressed in.
  [[nodiscard]] math::SE3Tangent<Scalar> get_spatial_velocity(
      const math::R3<Scalar>& offset,
      const Frame* relative_to,
      const Frame* in_coordinates_of) const;

  /// Returns the angular velocity in the coordinates of inertial frame.
  [[nodiscard]] math::SO3Tangent<Scalar> get_angular_velocity(
      const Frame* relative_to = Frame::GetInertialFrame(),
      const Frame* in_coordinates_of = Frame::GetInertialFrame()) const;

  /// Returns the linear velocity of the origin of this frame in the coordinates
  /// of inertial frame.
  [[nodiscard]] math::R3Tangent<Scalar> get_linear_velocity(
      const Frame* relative_to = Frame::GetInertialFrame(),
      const Frame* in_coordinates_of = Frame::GetInertialFrame()) const;

  /// Returns the linear velocity of a point on this frame in the coordinates
  /// of inertial frame.
  [[nodiscard]] math::R3Tangent<Scalar> get_linear_velocity(
      const math::R3<Scalar>& offset,
      const Frame* relative_to = Frame::GetInertialFrame(),
      const Frame* in_coordinates_of = Frame::GetInertialFrame()) const;

  /// @}

  /// @{ @name Acceleration

  /// Returns the global spatial acceleration expressed in this frame.
  [[nodiscard]] virtual const math::SE3Tangent<Scalar>&
  get_spatial_acceleration() const = 0;

  /// Returns the angular acceleration.
  [[nodiscard]] math::SO3Tangent<Scalar> get_angular_acceleration() const;

  /// Returns the linear acceleration.
  [[nodiscard]] math::R3Tangent<Scalar> get_linear_acceleration() const;

  /// @}

protected:
  friend class RelativeFrame<Scalar>;

  /// Marks the global pose needs to be update.
  virtual void dirty_pose();

  /// Marks the global spatial velocity needs to be update.
  virtual void dirty_spatial_velocity();

  /// Marks the global spatial acceleration needs to be update.
  virtual void dirty_spatial_acceleration();
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, Frame)

/// Implementation of the inertial frame that is a frame of reference that is
/// not undergoing acceleration.
template <typename Scalar>
class InertialFrame : public Frame<Scalar>
{
private:
  friend class Frame<Scalar>;

public:
  /// Returns the name of the inertial frame.
  [[nodiscard]] static const std::string& GetName();

protected:
  /// Default constructor that is only allowed to be called by Frame.
  InertialFrame();

private:
  // Implementation of Frame<Scalar>
  void set_name(const std::string& name) override;
  // Disallow setting the name of the inertial frame

public:
  // Implementation of Frame<Scalar>
  bool is_inertial_frame() const override;
  const std::string& get_name() const override;
  const math::SE3<Scalar>& get_pose() const override;
  const math::SE3Tangent<Scalar>& get_spatial_velocity() const override;
  const math::SE3Tangent<Scalar>& get_spatial_acceleration() const override;

private:
  /// Identity pose
  static const math::SE3<Scalar> m_identity_pose;

  /// Zero spatial motion
  static const math::SE3Tangent<Scalar> m_zero_spatial_motion;
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, InertialFrame)

template <typename Scalar_>
class FreeFrame : public Frame<Scalar_>
{
public:
  using Scalar = Scalar_;

  /// Default constructor.
  FreeFrame();

  // Implementation of Frame<Scalar>
  void set_name(const std::string& name) override;
  [[nodiscard]] const std::string& get_name() const override;
  [[nodiscard]] const math::SE3<Scalar>& get_pose() const override;
  [[nodiscard]] const math::SE3Tangent<Scalar>& get_spatial_velocity()
      const override;
  [[nodiscard]] const math::SE3Tangent<Scalar>& get_spatial_acceleration()
      const override;

  /// Sets the global pose.
  ///
  /// @param[in] pose: Global pose.
  template <typename Derived>
  void set_pose(const math::SE3Base<Derived>& pose);

  /// Sets the global orientation.
  ///
  /// @param[in] orientation: Global orientation.
  template <typename Derived>
  void set_orientation(const math::SO3Base<Derived>& orientation);

  /// Sets the global orientation.
  ///
  /// @param[in] orientation: Global orientation.
  template <typename Derived>
  void set_orientation(math::SO3Base<Derived>&& orientation);

  /// Sets the global position.
  ///
  /// @param[in] position: Global position.
  void set_position(const math::R3<Scalar>& position);

  /// Sets the global position.
  ///
  /// @param[in] position: Global position.
  void set_position(math::R3<Scalar>&& position);

  /// Sets the global spatial velocity in the coordinates of this frame.
  ///
  /// @param[in] spatial_velocity: Spatial velocity.
  template <typename Derived>
  void set_spatial_velocity(
      const math::SE3TangentBase<Derived>& spatial_velocity);

  /// Sets the global angular velocity in the coordinates of inertial frame.
  ///
  /// @param[in] spatial_velocity: Angular velocity.
  template <typename Derived>
  void set_angular_velocity(
      const math::SO3TangentBase<Derived>& angular_velocity);

  /// Sets the global linear velocity in the coordinates of inertial frame.
  ///
  /// @param[in] spatial_velocity: Linear velocity.
  template <typename Derived>
  void set_linear_velocity(const math::RTangentBase<Derived>& linear_velocity);

  /// Sets the global spatial acceleration in the coordinates of this frame.
  ///
  /// @param[in] spatial_acceleration: Spatial acceleration.
  template <typename Derived>
  void set_spatial_acceleration(
      const math::SE3TangentBase<Derived>& spatial_acceleration);

protected:
  /// Global pose.
  math::SE3<Scalar> m_pose;

  /// Global Spatial velocity in the coordinates of this frame.
  math::SE3Tangent<Scalar> m_spatial_velocity;

  /// Global Spatial acceleration in the coordinates of this frame.
  math::SE3Tangent<Scalar> m_spatial_acceleration;

  /// Name of this frame.
  std::string m_name;
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, FreeFrame)

/// Base class for frames that are in a tree structure.
template <typename Scalar_>
class RelativeFrame : public Frame<Scalar_>
{
public:
  using Scalar = Scalar_;
  using Frame<Scalar>::GetInertialFrame;

protected:
  /// Constructor
  ///
  /// @param[in] parent_frame: (optional) Parent frame of this relative frame.
  explicit RelativeFrame(Frame<Scalar>* parent_frame = GetInertialFrame());

public:
  /// @{ @name Hierarchy

  /// Returns the parent frame.
  [[nodiscard]] const Frame<Scalar>* get_parent_frame() const;

  /// Returns the mutable parent frame.
  [[nodiscard]] Frame<Scalar>* get_mutable_parent_frame();

  /// Returns the number of child frames.
  [[nodiscard]] int get_num_child_frames() const;

  /// Returns a child frame by index.
  ///
  /// nullptr is returned when the index is invalid.
  [[nodiscard]] const Frame<Scalar>* get_child_frame_by_index(int index) const;

  /// Returns a child frame by name.
  ///
  /// nullptr is returns when there is no child frame has name, @c name.
  [[nodiscard]] const Frame<Scalar>* get_child_frame_by_name(
      const std::string& name) const;

  /// @}

  /// @{ @name Pose (transformation)

  /// Returns the relative pose with respect to the parent frame.
  [[nodiscard]] virtual const math::SE3<Scalar>& get_local_pose() const = 0;

  /// Returns the global pose in coordinates of the inertial frame.
  [[nodiscard]] const math::SE3<Scalar>& get_pose() const final;

  /// Returns the relative pose with respect to @c with_respect_to in the
  /// coordinates of @c with_respect_to.
  ///
  /// The output is defined as @f$ T = T_B^{-1} * T_A @f$ where @f$ T_A @f$ is
  /// the global pose of this frame, and @f$ T_B @f$ is with_respect_to.
  ///
  /// @param[in] with_respect_to: The reference frame to compute the relative
  /// pose.
  [[nodiscard]] math::SE3<Scalar> get_pose(
      const Frame<Scalar>* with_respect_to) const final;

  /// Returns the relative pose with respect to @c with_respect_to in the
  /// coordinates of @c in_coordinates_of.
  ///
  /// The output is defined as @f$ T = T_B^{-1} * T_A @f$ where @f$ T_A @f$ is
  /// the global pose of this frame, and @f$ T_B = (R_C, p_D) @f$ where
  /// @f$ R_C @f$ is the orientation component of @c in_coordinates_of, and
  /// @f$ p_D @f$ is the position component of @c with_respect_to.
  ///
  /// @param[in] with_respect_to: The reference frame of the relative pose.
  /// @param[in] in_coordinates_of: The reference frame that the output is
  /// expressed in.
  [[nodiscard]] math::SE3<Scalar> get_pose(
      const Frame<Scalar>* with_respect_to,
      const Frame<Scalar>* in_coordinates_of) const;

  /// @}

  /// @{ @name Velocity

  /// Returns the spatial velocity of this frame.
  [[nodiscard]] virtual const math::SE3Tangent<Scalar>&
  get_local_spatial_velocity() const = 0;

  /// Returns the global spatial velocity in the coordinates of this frame.
  [[nodiscard]] const math::SE3Tangent<Scalar>& get_spatial_velocity()
      const override;

  /// @}

  /// @{ @name Acceleration

  [[nodiscard]] virtual const math::SE3Tangent<Scalar>&
  get_local_spatial_acceleration() const = 0;

  /// Returns the global spatial acceleration expressed in this frame.
  [[nodiscard]] const math::SE3Tangent<Scalar>& get_spatial_acceleration()
      const override;

  /// @}

protected:
  /// Marks the global pose needs to be update.
  void dirty_pose() override;

  /// Marks the global spatial velocity needs to be update.
  void dirty_spatial_velocity() override;

  /// Marks the global spatial acceleration needs to be update.
  void dirty_spatial_acceleration() override;

  /// The global pose in the global coordinates.
  mutable math::SE3<Scalar> m_pose;

  /// The global spatial acceleration in the local coordiantes.
  mutable math::SE3Tangent<Scalar> m_spatial_velocity;

  /// The global spatial acceleration in the local coordiantes.
  mutable math::SE3Tangent<Scalar> m_spatial_acceleration;

  /// The parent frame.
  Frame<Scalar>* m_parent_frame = nullptr;

  /// The child frames.
  std::vector<Frame<Scalar>*> m_child_frames;

  /// Flag whether the global pose needs to be updated.
  mutable bool m_pose_dirty = true;

  /// Flag whether the global spatial velocity needs to be updated.
  mutable bool m_spatial_velocity_dirty = true;

  /// Flag whether the global spatial acceleration needs to be updated.
  mutable bool m_spatial_acceleration_dirty = true;
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, RelativeFrame)

template <typename Scalar_>
class FreeRelativeFrame : public RelativeFrame<Scalar_>
{
public:
  using Scalar = Scalar_;

  /// Default constructor.
  FreeRelativeFrame();

  // Implementation of Frame<Scalar>
  void set_name(const std::string& name) override;
  const std::string& get_name() const override;
  const math::SE3<Scalar>& get_local_pose() const override;
  const math::SE3Tangent<Scalar>& get_local_spatial_velocity() const override;
  const math::SE3Tangent<Scalar>& get_local_spatial_acceleration()
      const override;

protected:
  /// Pose relative to its parent frame.
  math::SE3<Scalar> m_local_pose;

  /// Spatial velocity relative to its parent frame in the coordinates of this
  /// frame.
  math::SE3Tangent<Scalar> m_local_spatial_velocity;

  /// Spatial acceleration relative to its parent frame in the coordinates of
  /// this frame.
  math::SE3Tangent<Scalar> m_local_spatial_acceleration;

  /// Name of this frame.
  std::string m_name;
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, FreeRelativeFrame)

} // namespace dart::multibody

#include "dart/multibody/detail/frame_impl.hpp"
