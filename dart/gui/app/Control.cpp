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

#include "Control.hpp"

#include <dart/external/imgui/imgui.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

#include <cmath>
#include <cstdio>

namespace dart::gui::app {

namespace {

constexpr int kMaxDecimals = 15;

/// The ImGui ID of a value widget, whose label is drawn on its own line.
constexpr const char* kValueId = "##value";
// Appended to a label shown inside its widget: the ID ignores the label, so
// renaming a control keeps its click and open state.
constexpr const char* kStableId = "###control";

std::string formatFor(int decimals)
{
  char format[16];
  std::snprintf(format, sizeof(format), "%%.%df", decimals);
  return format;
}

void requireFinite(double value, const char* what)
{
  if (!std::isfinite(value))
    throw std::invalid_argument(std::string(what) + " must be finite");
}

// Plots store floats: a larger finite double would become infinite.
void requireFloat(double value, const char* what)
{
  if (!(std::abs(value) <= std::numeric_limits<float>::max()))
    throw std::invalid_argument(
        std::string(what) + " must be finite and fit in a float");
}

} // namespace

//==============================================================================
std::shared_ptr<Control> Control::makePanel()
{
  auto panel = std::shared_ptr<Control>(new Control(Kind::Group, ""));
  panel->mRoot = true;
  return panel;
}

//==============================================================================
Control::Control(Kind kind, std::string label)
  : mKind(kind), mLabel(std::move(label))
{
}

//==============================================================================
Control::Kind Control::getKind() const
{
  return mKind;
}

//==============================================================================
const std::string& Control::getLabel() const
{
  return mLabel;
}

//==============================================================================
void Control::setLabel(std::string label)
{
  mLabel = std::move(label);
}

//==============================================================================
const std::string& Control::getTooltip() const
{
  return mTooltip;
}

//==============================================================================
void Control::setTooltip(std::string tooltip)
{
  mTooltip = std::move(tooltip);
}

//==============================================================================
bool Control::isEnabled() const
{
  return mEnabled;
}

//==============================================================================
void Control::setEnabled(bool enabled)
{
  mEnabled = enabled;
}

//==============================================================================
bool Control::isVisible() const
{
  return mVisible;
}

//==============================================================================
void Control::setVisible(bool visible)
{
  mVisible = visible;
}

//==============================================================================
bool Control::isSameLine() const
{
  return mSameLine;
}

//==============================================================================
void Control::setSameLine(bool sameLine)
{
  mSameLine = sameLine;
}

//==============================================================================
const Eigen::Vector4d& Control::getColor() const
{
  return mColor;
}

//==============================================================================
void Control::setColor(const Eigen::Vector4d& rgba)
{
  if (!rgba.allFinite())
    throw std::invalid_argument("color components must be finite");
  mColor = rgba.cwiseMax(0.0).cwiseMin(1.0);
}

//==============================================================================
const Eigen::Vector4d& Control::getValue() const
{
  return mValue;
}

//==============================================================================
void Control::setValue(const Eigen::Vector4d& value)
{
  if (!value.allFinite())
    throw std::invalid_argument("control values must be finite");
  mValue = sanitize(value);
}

//==============================================================================
double Control::getMin() const
{
  return mMin;
}

//==============================================================================
double Control::getMax() const
{
  return mMax;
}

//==============================================================================
void Control::setRange(double min, double max)
{
  if (mKind != Kind::Slider && mKind != Kind::Input && mKind != Kind::Vec3)
    throw std::logic_error("only sliders, inputs and vectors have a range");
  if (std::isnan(min) || std::isnan(max) || min > max
      || min == std::numeric_limits<double>::infinity()
      || max == -std::numeric_limits<double>::infinity())
    throw std::invalid_argument(
        "range bounds must satisfy min <= max and admit a finite value");
  if (mKind == Kind::Slider) {
    // ImGui asserts on slider ranges wider than half of the value type's span.
    const double limit = mInteger ? std::numeric_limits<int>::max() / 2
                                  : std::numeric_limits<double>::max() / 2;
    if (!(std::abs(min) <= limit && std::abs(max) <= limit))
      throw std::invalid_argument("slider bounds are out of range");
  }
  if (mInteger) {
    // Whole-number bounds, so clamping keeps a rounded value whole.
    constexpr double kIntMin = std::numeric_limits<int>::min();
    constexpr double kIntMax = std::numeric_limits<int>::max();
    min = std::ceil(std::clamp(min, kIntMin, kIntMax));
    max = std::floor(std::clamp(max, kIntMin, kIntMax));
    if (min > max)
      throw std::invalid_argument("an integer range must contain an integer");
  }
  mMin = min;
  mMax = max;
  mValue = sanitize(mValue);
}

//==============================================================================
const std::vector<std::string>& Control::getOptions() const
{
  return mOptions;
}

//==============================================================================
void Control::push(double sample)
{
  if (mKind != Kind::Plot)
    throw std::logic_error("only plots take samples");
  requireFloat(sample, "plot samples");
  mSamples[mSampleHead] = static_cast<float>(sample);
  mSampleHead = (mSampleHead + 1) % mSamples.size();
  mSampleCount = std::min(mSampleCount + 1, mSamples.size());
}

//==============================================================================
void Control::clear()
{
  if (mKind != Kind::Plot)
    throw std::logic_error("only plots take samples");
  std::fill(mSamples.begin(), mSamples.end(), 0.0f);
  mSampleHead = 0;
  mSampleCount = 0;
}

//==============================================================================
std::vector<double> Control::getSamples() const
{
  std::vector<double> samples;
  samples.reserve(mSampleCount);
  const std::size_t start = (mSampleHead + mSamples.size() - mSampleCount)
                            % std::max<std::size_t>(mSamples.size(), 1);
  for (std::size_t i = 0; i < mSampleCount; ++i)
    samples.push_back(mSamples[(start + i) % mSamples.size()]);
  return samples;
}

//==============================================================================
void Control::setOnChange(std::function<void()> callback)
{
  mOnChange = callback
                  ? std::make_shared<std::function<void()>>(std::move(callback))
                  : nullptr;
}

//==============================================================================
std::shared_ptr<Control> Control::addGroup(std::string label, bool open)
{
  auto group = std::shared_ptr<Control>(new Control(Kind::Group, label));
  group->mOpen = open;
  return append(std::move(group));
}

//==============================================================================
std::shared_ptr<Control> Control::addText(std::string text, bool wrap)
{
  auto control = std::shared_ptr<Control>(new Control(Kind::Text, text));
  control->mWrap = wrap;
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addButton(std::string label)
{
  return append(std::shared_ptr<Control>(new Control(Kind::Button, label)));
}

//==============================================================================
std::shared_ptr<Control> Control::addCheckbox(std::string label, bool value)
{
  auto control = std::shared_ptr<Control>(new Control(Kind::Checkbox, label));
  control->mMin = 0.0;
  control->mMax = 1.0;
  control->mValue.x() = value ? 1.0 : 0.0;
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addSlider(
    std::string label,
    double value,
    double min,
    double max,
    int decimals,
    bool integer,
    bool notifyOnRelease)
{
  requireFinite(value, "slider value");
  if (decimals < 0 || decimals > kMaxDecimals)
    throw std::invalid_argument("decimals must be between 0 and 15");
  auto control = std::shared_ptr<Control>(new Control(Kind::Slider, label));
  control->mDecimals = decimals;
  control->mInteger = integer;
  control->mNotifyOnRelease = notifyOnRelease;
  control->setRange(min, max);
  control->setValue(Eigen::Vector4d(value, 0.0, 0.0, 0.0));
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addInput(
    std::string label,
    double value,
    double step,
    int decimals,
    bool integer,
    double min,
    double max)
{
  requireFinite(value, "input value");
  requireFinite(step, "input step");
  if (decimals < 0 || decimals > kMaxDecimals)
    throw std::invalid_argument("decimals must be between 0 and 15");
  auto control = std::shared_ptr<Control>(new Control(Kind::Input, label));
  control->mStep = std::abs(step);
  control->mDecimals = decimals;
  control->mInteger = integer;
  control->setRange(min, max);
  control->setValue(Eigen::Vector4d(value, 0.0, 0.0, 0.0));
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addVec3(
    std::string label,
    const Eigen::Vector3d& value,
    double speed,
    double min,
    double max)
{
  requireFinite(speed, "drag speed");
  auto control = std::shared_ptr<Control>(new Control(Kind::Vec3, label));
  control->mSpeed = std::abs(speed);
  control->setRange(min, max);
  control->setValue(Eigen::Vector4d(value.x(), value.y(), value.z(), 0.0));
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addChoice(
    std::string label, std::vector<std::string> options, int index, bool combo)
{
  if (options.empty())
    throw std::invalid_argument("a choice needs at least one option");
  if (index < 0 || static_cast<std::size_t>(index) >= options.size())
    throw std::invalid_argument("choice index is out of range");
  auto control = std::shared_ptr<Control>(new Control(Kind::Choice, label));
  control->mOptions = std::move(options);
  control->mCombo = combo;
  control->mMin = 0.0;
  control->mMax = static_cast<double>(control->mOptions.size() - 1);
  control->mValue.x() = index;
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addColor(
    std::string label, const Eigen::Vector4d& rgba, bool alpha)
{
  auto control = std::shared_ptr<Control>(new Control(Kind::Color, label));
  control->mAlpha = alpha;
  control->setValue(rgba);
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addSwatch(const Eigen::Vector4d& rgba)
{
  auto control = std::shared_ptr<Control>(new Control(Kind::Swatch, ""));
  control->setValue(rgba);
  return append(std::move(control));
}

//==============================================================================
std::shared_ptr<Control> Control::addPlot(
    std::string label, std::size_t capacity, double min, double max)
{
  if (capacity == 0)
    throw std::invalid_argument("plot capacity must be positive");
  requireFloat(min, "plot bounds");
  requireFloat(max, "plot bounds");
  // ImGui scales samples by the bounds' float span.
  const float span = static_cast<float>(max) - static_cast<float>(min);
  if (!(span > 0.0f && std::isfinite(span)))
    throw std::invalid_argument(
        "plot bounds must satisfy min < max, with a span that fits in a float");
  auto control = std::shared_ptr<Control>(new Control(Kind::Plot, label));
  control->mMin = min;
  control->mMax = max;
  control->mSamples.assign(capacity, 0.0f);
  return append(std::move(control));
}

//==============================================================================
void Control::addSeparator()
{
  append(std::shared_ptr<Control>(new Control(Kind::Separator, "")));
}

//==============================================================================
const std::vector<std::shared_ptr<Control>>& Control::getChildren() const
{
  return mChildren;
}

//==============================================================================
std::shared_ptr<Control> Control::append(std::shared_ptr<Control> child)
{
  requireGroup();
  mChildren.push_back(child);
  return child;
}

//==============================================================================
void Control::requireGroup() const
{
  if (mKind != Kind::Group)
    throw std::logic_error("only groups have children");
}

//==============================================================================
Eigen::Vector4d Control::sanitize(const Eigen::Vector4d& value) const
{
  Eigen::Vector4d result = value;
  switch (mKind) {
    case Kind::Checkbox:
      result.x() = result.x() != 0.0 ? 1.0 : 0.0;
      break;
    case Kind::Choice:
      result.x() = std::clamp(std::round(result.x()), mMin, mMax);
      break;
    case Kind::Slider:
    case Kind::Input:
      result.x() = std::clamp(result.x(), mMin, mMax);
      if (mInteger)
        result.x() = std::clamp(std::round(result.x()), mMin, mMax);
      break;
    case Kind::Vec3:
      for (int i = 0; i < 3; ++i)
        result[i] = std::clamp(result[i], mMin, mMax);
      break;
    case Kind::Color:
    case Kind::Swatch:
      result = result.cwiseMax(0.0).cwiseMin(1.0);
      if (mKind == Kind::Color && !mAlpha)
        result.w() = 1.0;
      break;
    default:
      break;
  }
  return result;
}

//==============================================================================
void Control::edit(const Eigen::Vector4d& value, bool notify)
{
  // ImGui's Ctrl+click text entry bypasses widget ranges; never let a typed
  // NaN or infinity reach a scene.
  if (!value.allFinite())
    return;
  const Eigen::Vector4d sanitized = sanitize(value);
  if (sanitized == mValue)
    return;
  mValue = sanitized;
  if (notify)
    mPending = true;
  else
    mUnreleasedEdit = true;
}

//==============================================================================
void Control::release()
{
  if (std::exchange(mUnreleasedEdit, false))
    mPending = true;
}

//==============================================================================
void Control::markChanged()
{
  mPending = true;
}

//==============================================================================
std::size_t Control::dispatch(
    const std::function<void(Control&, const std::string&)>& onError)
{
  std::size_t failures = 0;
  if (mPending) {
    mPending = false;
    if (mOnChange) {
      // A callback may replace or clear itself; keep it alive without
      // copying it, so its own state persists across calls.
      const auto callback = mOnChange;
      try {
        (*callback)();
      } catch (const std::exception& error) {
        ++failures;
        mOnChange = nullptr;
        mEnabled = false;
        if (onError)
          onError(*this, error.what());
      } catch (...) {
        ++failures;
        mOnChange = nullptr;
        mEnabled = false;
        if (onError)
          onError(*this, "unknown error");
      }
    }
  }
  // Iterate a snapshot: a callback may append controls to a group.
  const auto children = mChildren;
  for (const auto& child : children)
    failures += child->dispatch(onError);
  return failures;
}

//==============================================================================
void Control::clearCallbacks()
{
  // A destroyed callback's captures may add controls, with callbacks of their
  // own, anywhere in the tree: repeat until a pass drops none.
  while (clearCallbacksPass()) {
  }
}

//==============================================================================
bool Control::clearCallbacksPass()
{
  bool dropped = std::exchange(mOnChange, nullptr) != nullptr;
  mPending = false;
  mUnreleasedEdit = false;
  // Index the live list, which a destroyed callback may grow.
  for (std::size_t i = 0; i < mChildren.size(); ++i) {
    const auto child = mChildren[i];
    dropped = child->clearCallbacksPass() || dropped;
  }
  return dropped;
}

//==============================================================================
void Control::render()
{
  if (!mVisible)
    return;

  if (mKind == Kind::Group) {
    if (mRoot) {
      ImGui::BeginDisabled(!mEnabled);
      renderChildren();
      ImGui::EndDisabled();
      return;
    }
    ImGui::PushID(this);
    if (mSameLine)
      ImGui::SameLine();
    ImGui::BeginDisabled(!mEnabled);
    const bool open = ImGui::CollapsingHeader(
        (mLabel + kStableId).c_str(),
        mOpen ? ImGuiTreeNodeFlags_DefaultOpen : 0);
    if (!mTooltip.empty())
      ImGui::SetItemTooltip("%s", mTooltip.c_str());
    if (open)
      renderChildren();
    ImGui::EndDisabled();
    ImGui::PopID();
    return;
  }

  ImGui::PushID(this);
  if (mSameLine)
    ImGui::SameLine();
  ImGui::BeginDisabled(!mEnabled);
  renderSelf();
  if (!mTooltip.empty())
    ImGui::SetItemTooltip("%s", mTooltip.c_str());
  ImGui::EndDisabled();
  ImGui::PopID();
}

//==============================================================================
void Control::renderChildren()
{
  for (const auto& child : mChildren)
    child->render();
}

//==============================================================================
void Control::renderSelf()
{
  // Value widgets put their label on its own line and take the full width,
  // so scaled panels never clip it (see docs/onboarding/testing.md).
  const auto labelAbove = [this] {
    if (!mLabel.empty())
      ImGui::TextUnformatted(mLabel.c_str());
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
  };

  switch (mKind) {
    case Kind::Text: {
      const bool tinted = mColor.w() > 0.0;
      if (tinted) {
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            ImVec4(
                static_cast<float>(mColor.x()),
                static_cast<float>(mColor.y()),
                static_cast<float>(mColor.z()),
                static_cast<float>(mColor.w())));
      }
      if (mWrap)
        ImGui::TextWrapped("%s", mLabel.c_str());
      else
        ImGui::TextUnformatted(mLabel.c_str());
      if (tinted)
        ImGui::PopStyleColor();
      break;
    }
    case Kind::Button:
      if (ImGui::Button((mLabel + kStableId).c_str()))
        markChanged();
      break;
    case Kind::Checkbox: {
      bool checked = mValue.x() != 0.0;
      if (ImGui::Checkbox((mLabel + kStableId).c_str(), &checked))
        edit(Eigen::Vector4d(checked ? 1.0 : 0.0, 0.0, 0.0, 0.0));
      break;
    }
    case Kind::Slider: {
      labelAbove();
      // Integers are edited as doubles shown without decimals: ImGui's
      // integer text parser wraps an out-of-range entry before edit() could
      // clamp it. A "%.0f" slider also rounds its drags to whole numbers.
      const std::string format = mInteger ? "%.0f" : formatFor(mDecimals);
      double value = mValue.x();
      if (ImGui::SliderScalar(
              kValueId,
              ImGuiDataType_Double,
              &value,
              &mMin,
              &mMax,
              format.c_str(),
              ImGuiSliderFlags_AlwaysClamp))
        edit(Eigen::Vector4d(value, 0.0, 0.0, 0.0), !mNotifyOnRelease);
      // ImGui reports a finished edit even when edit() rejected all of it.
      if (mNotifyOnRelease && ImGui::IsItemDeactivatedAfterEdit())
        release();
      break;
    }
    case Kind::Input: {
      labelAbove();
      const std::string format = mInteger ? "%.0f" : formatFor(mDecimals);
      double value = mValue.x();
      // An integer input always steps, by at least one.
      const double integerStep = std::max(1.0, std::round(mStep));
      const double* step
          = mInteger ? &integerStep : (mStep > 0.0 ? &mStep : nullptr);
      if (ImGui::InputScalar(
              kValueId,
              ImGuiDataType_Double,
              &value,
              step,
              nullptr,
              format.c_str()))
        edit(Eigen::Vector4d(value, 0.0, 0.0, 0.0));
      break;
    }
    case Kind::Vec3: {
      labelAbove();
      double values[3] = {mValue.x(), mValue.y(), mValue.z()};
      const double* min = std::isfinite(mMin) ? &mMin : nullptr;
      const double* max = std::isfinite(mMax) ? &mMax : nullptr;
      if (ImGui::DragScalarN(
              kValueId,
              ImGuiDataType_Double,
              values,
              3,
              static_cast<float>(mSpeed),
              min,
              max,
              "%.3f"))
        edit(Eigen::Vector4d(values[0], values[1], values[2], 0.0));
      break;
    }
    case Kind::Choice: {
      const int current = static_cast<int>(mValue.x());
      // Options may share a label; their IDs come from their indices.
      if (mCombo) {
        labelAbove();
        if (ImGui::BeginCombo(kValueId, mOptions[current].c_str())) {
          for (std::size_t i = 0; i < mOptions.size(); ++i) {
            const bool selected = static_cast<int>(i) == current;
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(mOptions[i].c_str(), selected))
              edit(Eigen::Vector4d(static_cast<double>(i), 0.0, 0.0, 0.0));
            if (selected)
              ImGui::SetItemDefaultFocus();
            ImGui::PopID();
          }
          ImGui::EndCombo();
        }
      } else {
        if (!mLabel.empty())
          ImGui::TextUnformatted(mLabel.c_str());
        // Wrap an option that would pass the panel's right edge, so scaled
        // panels never clip it.
        const ImGuiStyle& style = ImGui::GetStyle();
        const float right
            = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
        for (std::size_t i = 0; i < mOptions.size(); ++i) {
          if (i > 0) {
            const float width
                = ImGui::GetFrameHeight() + style.ItemInnerSpacing.x
                  + ImGui::CalcTextSize(mOptions[i].c_str(), nullptr, true).x;
            if (ImGui::GetItemRectMax().x + style.ItemSpacing.x + width
                <= right)
              ImGui::SameLine();
          }
          ImGui::PushID(static_cast<int>(i));
          if (ImGui::RadioButton(
                  mOptions[i].c_str(), static_cast<int>(i) == current))
            edit(Eigen::Vector4d(static_cast<double>(i), 0.0, 0.0, 0.0));
          ImGui::PopID();
        }
      }
      break;
    }
    case Kind::Color: {
      labelAbove();
      float rgba[4]
          = {static_cast<float>(mValue.x()),
             static_cast<float>(mValue.y()),
             static_cast<float>(mValue.z()),
             static_cast<float>(mValue.w())};
      const bool changed = mAlpha ? ImGui::ColorEdit4(kValueId, rgba)
                                  : ImGui::ColorEdit3(kValueId, rgba);
      if (changed)
        edit(Eigen::Vector4d(rgba[0], rgba[1], rgba[2], rgba[3]));
      break;
    }
    case Kind::Swatch: {
      const float size = ImGui::GetFontSize();
      ImGui::ColorButton(
          "##swatch",
          ImVec4(
              static_cast<float>(mValue.x()),
              static_cast<float>(mValue.y()),
              static_cast<float>(mValue.z()),
              static_cast<float>(mValue.w())),
          ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker,
          ImVec2(size, size));
      break;
    }
    case Kind::Plot: {
      const std::size_t capacity = mSamples.size();
      const std::size_t offset
          = mSampleCount < capacity ? 0 : mSampleHead % capacity;
      labelAbove();
      ImGui::PlotLines(
          kValueId,
          mSamples.data(),
          static_cast<int>(mSampleCount),
          static_cast<int>(offset),
          nullptr,
          static_cast<float>(mMin),
          static_cast<float>(mMax),
          ImVec2(0.0f, 5.0f * ImGui::GetFontSize()));
      break;
    }
    case Kind::Separator:
      ImGui::Separator();
      break;
    case Kind::Group:
      break;
  }
}

} // namespace dart::gui::app
