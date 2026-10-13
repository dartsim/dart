// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include "detail/eigen.hpp"
#include "viewer/session.hpp"

#include <nanobind/stl/vector.h>

#include <limits>

namespace dartnb::viewer {

namespace app = dart::gui::app;
using Kind = app::Control::Kind;

//==============================================================================
int PyControl::traverse(visitproc visit, void* arg) const
{
  if (const int result = visitSlot(onChange, visit, arg))
    return result;
  for (const auto& child : children)
    Py_VISIT(child.ptr());
  return 0;
}

//==============================================================================
void PyControl::clear()
{
  if (native)
    native->setOnChange(nullptr);
  onChange.reset();
  children.clear();
}

//==============================================================================
void PyControl::rewire()
{
  if (onChange)
    native->setOnChange(hostCallback(onChange));
  for (const auto& child : children)
    nb::cast<PyControl&>(child).rewire();
}

namespace {

//==============================================================================
nb::object adopt(PyControl& parent, std::shared_ptr<app::Control> child)
{
  nb::object wrapper = nb::cast(PyControl{std::move(child), nullptr, {}});
  parent.children.push_back(wrapper);
  return wrapper;
}

//==============================================================================
nb::object controlValue(const PyControl& control)
{
  const Eigen::Vector4d& value = control.native->getValue();
  switch (control.native->getKind()) {
    case Kind::Checkbox:
      return nb::bool_(value.x() != 0.0);
    case Kind::Choice:
      return nb::int_(static_cast<long long>(value.x()));
    case Kind::Slider:
    case Kind::Input:
      if (control.native->isInteger())
        return nb::int_(static_cast<long long>(value.x()));
      return nb::float_(value.x());
    case Kind::Vec3:
      return nb::cast(Eigen::Vector3d(value.head<3>()));
    case Kind::Color:
    case Kind::Swatch:
      return nb::cast(Eigen::Vector4d(value));
    default:
      throw nb::type_error("this kind of control has no value");
  }
}

//==============================================================================
Eigen::Vector4d nativeValue(const PyControl& control, nb::handle value)
{
  Eigen::Vector4d native = Eigen::Vector4d::Zero();
  switch (control.native->getKind()) {
    case Kind::Checkbox:
      native.x() = nb::cast<bool>(value) ? 1.0 : 0.0;
      break;
    case Kind::Choice: {
      const auto index = nb::cast<long long>(value);
      if (index < 0
          || index
                 >= static_cast<long long>(control.native->getOptions().size()))
        throw nb::value_error("choice index is out of range");
      native.x() = static_cast<double>(index);
      break;
    }
    case Kind::Slider:
    case Kind::Input:
      native.x() = nb::cast<double>(value);
      break;
    case Kind::Vec3:
      native.head<3>() = nb::cast<Eigen::Vector3d>(value);
      break;
    case Kind::Color:
    case Kind::Swatch:
      native = nb::cast<Eigen::Vector4d>(value);
      break;
    default:
      throw nb::type_error("this kind of control has no value");
  }
  return native;
}

//==============================================================================
void setControlValue(PyControl& control, nb::handle value)
{
  control.native->setValue(nativeValue(control, value));
}

//==============================================================================
void editControl(PyControl& control, nb::handle value)
{
  switch (control.native->getKind()) {
    case Kind::Button:
      if (!value.is_none())
        throw nb::type_error("a button takes no value");
      control.native->markChanged();
      return;
    case Kind::Swatch:
      throw nb::type_error("this kind of control never changes");
    default:
      control.native->edit(nativeValue(control, value));
  }
}

//==============================================================================
void setOnChange(PyControl& control, nb::handle fn)
{
  switch (control.native->getKind()) {
    case Kind::Group:
    case Kind::Text:
    case Kind::Swatch:
    case Kind::Plot:
    case Kind::Separator:
      if (!fn.is_none())
        throw nb::type_error("this kind of control never changes");
      break;
    default:
      break;
  }
  auto slot = makeSlot(fn, "on_change");
  control.native->setOnChange(hostCallback(slot));
  control.onChange = std::move(slot);
}

} // namespace

//==============================================================================
void bindPanel(nb::module_& m)
{
  constexpr double infinity = std::numeric_limits<double>::infinity();

  nb::class_<PyControl> control(
      m,
      "Control",
      nb::type_slots(gcSlots<PyControl>()),
      nb::is_final(),
      nb::is_weak_referenceable(),
      "One element of a scene panel, built with the add_* methods of a "
      "group, starting from Panel(). The host draws the panel and validates "
      "every value; on_change runs between frames after a user edit or "
      "click, never after an assignment from Python.");

  nb::enum_<Kind>(control, "Kind")
      .value("GROUP", Kind::Group)
      .value("TEXT", Kind::Text)
      .value("BUTTON", Kind::Button)
      .value("CHECKBOX", Kind::Checkbox)
      .value("SLIDER", Kind::Slider)
      .value("INPUT", Kind::Input)
      .value("VEC3", Kind::Vec3)
      .value("CHOICE", Kind::Choice)
      .value("COLOR", Kind::Color)
      .value("SWATCH", Kind::Swatch)
      .value("PLOT", Kind::Plot)
      .value("SEPARATOR", Kind::Separator);

  control
      .def_prop_ro(
          "kind", [](const PyControl& self) { return self.native->getKind(); })
      .def_prop_rw(
          "label",
          [](const PyControl& self) { return self.native->getLabel(); },
          [](PyControl& self, std::string label) {
            self.native->setLabel(std::move(label));
          },
          "The label; for a text control, the text itself.")
      .def_prop_rw(
          "tooltip",
          [](const PyControl& self) { return self.native->getTooltip(); },
          [](PyControl& self, std::string tooltip) {
            self.native->setTooltip(std::move(tooltip));
          })
      .def_prop_rw(
          "enabled",
          [](const PyControl& self) { return self.native->isEnabled(); },
          [](PyControl& self, bool enabled) {
            self.native->setEnabled(enabled);
          })
      .def_prop_rw(
          "visible",
          [](const PyControl& self) { return self.native->isVisible(); },
          [](PyControl& self, bool visible) {
            self.native->setVisible(visible);
          })
      .def_prop_rw(
          "same_line",
          [](const PyControl& self) { return self.native->isSameLine(); },
          [](PyControl& self, bool sameLine) {
            self.native->setSameLine(sameLine);
          },
          "Places the control on the line of the previous one.")
      .def_prop_rw(
          "color",
          [](const PyControl& self) -> Eigen::Vector4d {
            return self.native->getColor();
          },
          [](PyControl& self, const Eigen::Vector4d& rgba) {
            self.native->setColor(rgba);
          },
          "Text tint in RGBA, clamped to [0, 1]; an alpha of 0 keeps the "
          "theme color.")
      .def_prop_rw(
          "value",
          &controlValue,
          &setControlValue,
          "bool for a checkbox, the option index for a choice, a float (an "
          "int when integer) for a slider or input, xyz for a vec3, and RGBA "
          "for a color or swatch. Assignments raise ValueError for "
          "non-finite values and are clamped to the range.")
      .def(
          "edit",
          &editControl,
          nb::arg("value").none() = nb::none(),
          "Applies value as a user edit does, or clicks a button: the value "
          "is clamped, a non-finite one is ignored, and a change queues "
          "on_change for the host's next dispatch between frames. Scripted "
          "scene tests use it to drive panels without a display.")
      .def_prop_rw(
          "on_change",
          [](const PyControl& self) { return slotFunction(self.onChange); },
          &setOnChange,
          nb::arg("callback").none(),
          "Called with no arguments between frames after a user edit or "
          "click. A callback that raises is disabled with its control.")
      .def_prop_ro(
          "min", [](const PyControl& self) { return self.native->getMin(); })
      .def_prop_ro(
          "max", [](const PyControl& self) { return self.native->getMax(); })
      .def(
          "set_range",
          [](PyControl& self, double min, double max) {
            self.native->setRange(min, max);
          },
          nb::arg("min"),
          nb::arg("max"),
          "Sets the range of a slider, input or vec3 and clamps the value.")
      .def_prop_ro(
          "options",
          [](const PyControl& self) { return self.native->getOptions(); })
      .def_prop_ro(
          "samples",
          [](const PyControl& self) { return self.native->getSamples(); },
          "The samples of a plot, oldest first.")
      .def(
          "push",
          [](PyControl& self, double sample) { self.native->push(sample); },
          nb::arg("sample"),
          "Appends a sample to a plot, dropping the oldest when full.")
      .def(
          "clear",
          [](PyControl& self) { self.native->clear(); },
          "Removes every sample of a plot.")
      .def(
          "add_group",
          [](PyControl& self, std::string label, bool open) {
            return adopt(self, self.native->addGroup(std::move(label), open));
          },
          nb::arg("label"),
          nb::kw_only(),
          nb::arg("open") = true)
      .def(
          "add_text",
          [](PyControl& self, std::string text, bool wrap) {
            return adopt(self, self.native->addText(std::move(text), wrap));
          },
          nb::arg("text"),
          nb::kw_only(),
          nb::arg("wrap") = false)
      .def(
          "add_button",
          [](PyControl& self, std::string label) {
            return adopt(self, self.native->addButton(std::move(label)));
          },
          nb::arg("label"))
      .def(
          "add_checkbox",
          [](PyControl& self, std::string label, bool value) {
            return adopt(
                self, self.native->addCheckbox(std::move(label), value));
          },
          nb::arg("label"),
          nb::arg("value") = false)
      .def(
          "add_slider",
          [](PyControl& self,
             std::string label,
             double value,
             double min,
             double max,
             int decimals,
             bool integer,
             bool notifyOnRelease) {
            return adopt(
                self,
                self.native->addSlider(
                    std::move(label),
                    value,
                    min,
                    max,
                    decimals,
                    integer,
                    notifyOnRelease));
          },
          nb::arg("label"),
          nb::arg("value"),
          nb::kw_only(),
          nb::arg("min"),
          nb::arg("max"),
          nb::arg("decimals") = 3,
          nb::arg("integer") = false,
          nb::arg("notify_on_release") = false,
          "notify_on_release calls on_change once when the drag ends.")
      .def(
          "add_input",
          [](PyControl& self,
             std::string label,
             double value,
             double step,
             int decimals,
             bool integer,
             double min,
             double max,
             double stepFast) {
            return adopt(
                self,
                self.native->addInput(
                    std::move(label),
                    value,
                    step,
                    decimals,
                    integer,
                    min,
                    max,
                    stepFast));
          },
          nb::arg("label"),
          nb::arg("value"),
          nb::kw_only(),
          nb::arg("step") = 0.0,
          nb::arg("decimals") = 3,
          nb::arg("integer") = false,
          nb::arg("min") = -infinity,
          nb::arg("max") = infinity,
          nb::arg("step_fast") = 0.0)
      .def(
          "add_vec3",
          [](PyControl& self,
             std::string label,
             const Eigen::Vector3d& value,
             double speed,
             double min,
             double max) {
            return adopt(
                self,
                self.native->addVec3(std::move(label), value, speed, min, max));
          },
          nb::arg("label"),
          nb::arg("value"),
          nb::kw_only(),
          nb::arg("speed") = 0.01,
          nb::arg("min") = -infinity,
          nb::arg("max") = infinity)
      .def(
          "add_choice",
          [](PyControl& self,
             std::string label,
             std::vector<std::string> options,
             int index,
             bool combo) {
            return adopt(
                self,
                self.native->addChoice(
                    std::move(label), std::move(options), index, combo));
          },
          nb::arg("label"),
          nb::arg("options"),
          nb::kw_only(),
          nb::arg("index") = 0,
          nb::arg("combo") = false,
          "Radio buttons, or a combo box when combo is true.")
      .def(
          "add_color",
          [](PyControl& self,
             std::string label,
             const Eigen::Vector4d& rgba,
             bool alpha) {
            return adopt(
                self, self.native->addColor(std::move(label), rgba, alpha));
          },
          nb::arg("label"),
          nb::arg("rgba"),
          nb::kw_only(),
          nb::arg("alpha") = true)
      .def(
          "add_swatch",
          [](PyControl& self, const Eigen::Vector4d& rgba) {
            return adopt(self, self.native->addSwatch(rgba));
          },
          nb::arg("rgba"))
      .def(
          "add_plot",
          [](PyControl& self,
             std::string label,
             std::size_t capacity,
             double min,
             double max) {
            return adopt(
                self,
                self.native->addPlot(std::move(label), capacity, min, max));
          },
          nb::arg("label"),
          nb::kw_only(),
          nb::arg("capacity"),
          nb::arg("min"),
          nb::arg("max"))
      .def("add_separator", [](PyControl& self) {
        self.native->addSeparator();
      });

  m.def(
      "Panel",
      [] {
        return PyControl{app::Control::makePanel(), nullptr, {}};
      },
      "The root group of a scene panel, drawn without a header.");
}

} // namespace dartnb::viewer
