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

#ifndef DART_GUI_INPUT_EVENT_HPP_
#define DART_GUI_INPUT_EVENT_HPP_

#include <variant>

namespace dart {
namespace gui {

enum class Key
{
  Space,
  Escape,
  Enter,
  Tab,
  Left,
  Right,
  Up,
  Down,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z,
  Num0,
  Num1,
  Num2,
  Num3,
  Num4,
  Num5,
  Num6,
  Num7,
  Num8,
  Num9,
};

enum class MouseButton
{
  Left,
  Right,
  Middle
};

struct ModifierKeys
{
  bool ctrl = false;
  bool shift = false;
  bool alt = false;
  bool super = false;
};

struct KeyEvent
{
  Key key;
  bool pressed;
  ModifierKeys modifiers;
};

struct MouseMoveEvent
{
  double x, y;
  double dx, dy;
  ModifierKeys modifiers;
};

struct MouseButtonEvent
{
  MouseButton button;
  bool pressed;
  double x, y;
  ModifierKeys modifiers;
};

struct ScrollEvent
{
  double dx, dy;
};

using InputEvent
    = std::variant<KeyEvent, MouseMoveEvent, MouseButtonEvent, ScrollEvent>;

} // namespace gui
} // namespace dart

#endif // DART_GUI_INPUT_EVENT_HPP_
