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

#ifndef DART_GUI_APP_CONTROL_HPP_
#define DART_GUI_APP_CONTROL_HPP_

#include <Eigen/Core>

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cstddef>

namespace dart::gui::app {

/// One node of a scene's declarative panel.
///
/// Scenes build a tree of controls; the host draws it with ImGui, validates
/// every value, and runs change callbacks between frames, never while drawing,
/// so a failing callback cannot leave ImGui unbalanced. Scenes update text,
/// values and plots from their hooks; those writes never run callbacks.
class Control
{
public:
  enum class Kind
  {
    Group,
    Text,
    Button,
    Checkbox,
    Slider,
    Input,
    Vec3,
    Choice,
    Color,
    Swatch,
    Plot,
    Separator
  };

  /// Creates the root group of a panel; the host draws it without a header.
  static std::shared_ptr<Control> makePanel();

  Kind getKind() const;

  /// The control's label; for Text, the text itself.
  const std::string& getLabel() const;
  void setLabel(std::string label);

  const std::string& getTooltip() const;
  void setTooltip(std::string tooltip);

  bool isEnabled() const;
  void setEnabled(bool enabled);

  bool isVisible() const;
  void setVisible(bool visible);

  /// Places the control on the same line as the previous one.
  bool isSameLine() const;
  void setSameLine(bool sameLine);

  /// Text tint in RGBA; an alpha of 0 keeps the theme color.
  const Eigen::Vector4d& getColor() const;
  void setColor(const Eigen::Vector4d& rgba);

  /// The value: 0 or 1 for Checkbox, the option index for Choice, the number
  /// for Slider and Input (in x), xyz for Vec3, and RGBA for Color and Swatch.
  const Eigen::Vector4d& getValue() const;

  /// Validates and stores a value: rejects non-finite components with
  /// std::invalid_argument, clamps to the range, and rounds integer kinds.
  /// Never runs the change callback.
  void setValue(const Eigen::Vector4d& value);

  double getMin() const;
  double getMax() const;

  /// Sets the range of a Slider, Input or Vec3 and clamps the current value.
  /// Throws std::invalid_argument unless min <= max and a Slider's bounds lie
  /// within half of its value type's span; integer ranges are clipped to int.
  void setRange(double min, double max);

  /// Choice options.
  const std::vector<std::string>& getOptions() const;

  /// Appends a sample to a Plot's ring buffer; non-finite samples throw.
  void push(double sample);

  /// Removes every Plot sample.
  void clear();

  /// Plot samples, oldest first.
  std::vector<double> getSamples() const;

  /// Runs after a user edit or click, between frames.
  void setOnChange(std::function<void()> callback);

  //----------------------------------------------------------------------------
  /// \{ \name Builders (groups only; throw std::logic_error otherwise)
  //----------------------------------------------------------------------------

  std::shared_ptr<Control> addGroup(std::string label, bool open = true);
  std::shared_ptr<Control> addText(std::string text, bool wrap = false);
  std::shared_ptr<Control> addButton(std::string label);
  std::shared_ptr<Control> addCheckbox(std::string label, bool value = false);
  std::shared_ptr<Control> addSlider(
      std::string label,
      double value,
      double min,
      double max,
      int decimals = 3,
      bool integer = false,
      bool notifyOnRelease = false);
  std::shared_ptr<Control> addInput(
      std::string label,
      double value,
      double step,
      int decimals = 3,
      bool integer = false,
      double min = -std::numeric_limits<double>::infinity(),
      double max = std::numeric_limits<double>::infinity());
  std::shared_ptr<Control> addVec3(
      std::string label,
      const Eigen::Vector3d& value,
      double speed = 0.01,
      double min = -std::numeric_limits<double>::infinity(),
      double max = std::numeric_limits<double>::infinity());
  std::shared_ptr<Control> addChoice(
      std::string label,
      std::vector<std::string> options,
      int index = 0,
      bool combo = false);
  std::shared_ptr<Control> addColor(
      std::string label, const Eigen::Vector4d& rgba, bool alpha = true);
  std::shared_ptr<Control> addSwatch(const Eigen::Vector4d& rgba);
  std::shared_ptr<Control> addPlot(
      std::string label, std::size_t capacity, double min, double max);
  void addSeparator();

  /// \}

  const std::vector<std::shared_ptr<Control>>& getChildren() const;

  //----------------------------------------------------------------------------
  /// \{ \name Host side
  //----------------------------------------------------------------------------

  /// Draws this control, and a group's children, inside an ImGui window.
  void render();

  /// Applies a value the user entered: non-finite input is ignored, the rest
  /// is sanitized like setValue(), and the change is queued for dispatch().
  /// With notify false, the change waits for release() instead.
  void edit(const Eigen::Vector4d& value, bool notify = true);

  /// Ends an edit made with notify false, such as a slider drag that reports
  /// on release: queues the change for dispatch() only if an edit since the
  /// last release changed the value.
  void release();

  /// Marks a button click (or any change) for dispatch().
  void markChanged();

  /// Runs queued change callbacks, depth first. A callback that throws is
  /// reported to \c onError, after which the control is disabled and its
  /// callback cleared. Returns the number of failed callbacks.
  std::size_t dispatch(
      const std::function<void(Control&, const std::string&)>& onError);

  /// Drops every callback in the tree; used at scene teardown.
  void clearCallbacks();

  /// \}

private:
  Control(Kind kind, std::string label);

  std::shared_ptr<Control> append(std::shared_ptr<Control> child);
  void requireGroup() const;
  Eigen::Vector4d sanitize(const Eigen::Vector4d& value) const;
  void renderChildren();
  void renderSelf();

  Kind mKind;
  std::string mLabel;
  std::string mTooltip;
  bool mEnabled = true;
  bool mVisible = true;
  bool mSameLine = false;
  bool mRoot = false;
  bool mOpen = true;
  bool mWrap = false;
  bool mInteger = false;
  bool mNotifyOnRelease = false;
  bool mCombo = false;
  bool mAlpha = true;
  int mDecimals = 3;
  double mMin = 0.0;
  double mMax = 0.0;
  double mStep = 0.0;
  double mSpeed = 0.01;
  Eigen::Vector4d mColor = Eigen::Vector4d::Zero();
  Eigen::Vector4d mValue = Eigen::Vector4d::Zero();
  std::vector<std::string> mOptions;
  std::vector<float> mSamples;
  std::size_t mSampleHead = 0;
  std::size_t mSampleCount = 0;
  std::shared_ptr<std::function<void()>> mOnChange;
  bool mPending = false;
  bool mUnreleasedEdit = false;
  std::vector<std::shared_ptr<Control>> mChildren;
};

} // namespace dart::gui::app

#endif // DART_GUI_APP_CONTROL_HPP_
