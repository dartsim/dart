/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <Eigen/Core>
#include <dartsim_engine/scene_object.hpp>
#include <dartsim_engine/sim_engine.hpp>

#include <optional>
#include <string>
#include <vector>

namespace dartsim::ui {

enum class InspectorNumericPropertyKind
{
  TranslationX,
  TranslationY,
  TranslationZ,
  Mass,
  ShapeDimensionX,
  ShapeDimensionY,
  ShapeDimensionZ,
  JointPosition,
  JointAxisX,
  JointAxisY,
  JointAxisZ,
};

enum class InspectorEnumPropertyKind
{
  ShapeType,
  JointKind,
};

struct InspectorNumericProperty
{
  InspectorNumericPropertyKind kind
      = InspectorNumericPropertyKind::TranslationX;
  std::string label;
  double value = 0.0;
  double minimum = 0.0;
  double maximum = 1.0;
  bool editable = false;
};

struct InspectorColorProperty
{
  std::string label;
  Eigen::Vector4d rgba = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
  bool editable = false;
};

struct InspectorEnumChoice
{
  std::string label;
  int value = 0;
};

struct InspectorEnumProperty
{
  InspectorEnumPropertyKind kind = InspectorEnumPropertyKind::ShapeType;
  std::string label;
  int value = 0;
  std::vector<InspectorEnumChoice> choices;
  bool editable = false;
};

struct InspectorStatus
{
  bool hasSelection = false;
  bool locked = false;
  ObjectId object = kNoObject;
  std::string name;
  std::string type;
  std::vector<InspectorEnumProperty> enumProperties;
  std::vector<InspectorNumericProperty> numericProperties;
  std::optional<InspectorColorProperty> colorProperty;
  bool canDelete = false;
};

struct InspectorActionResult
{
  bool ok = false;
  std::string message;
};

/// Build a typed inspector view model for the current primary selection.
[[nodiscard]] InspectorStatus buildInspectorStatus(const SimEngine& engine);

/// Apply one numeric inspector edit through the undoable command stack.
InspectorActionResult setInspectorNumericProperty(
    SimEngine& engine, InspectorNumericPropertyKind kind, double value);

/// Apply one enum inspector edit through the undoable command stack.
InspectorActionResult setInspectorEnumProperty(
    SimEngine& engine, InspectorEnumPropertyKind kind, int value);

/// Apply the selected object's shape color through the undoable command stack.
InspectorActionResult setInspectorShapeColor(
    SimEngine& engine, const Eigen::Vector4d& rgba);

/// Delete the current primary selection through the undoable command stack.
InspectorActionResult deleteInspectorSelection(SimEngine& engine);

} // namespace dartsim::ui
