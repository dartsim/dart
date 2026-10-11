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

#include <dart/gui/GUIEventHandler.hpp>

#include <GLFW/glfw3.h>

#include <optional>
#include <unordered_map>

namespace dart::gui::detail {

/// Maps a GLFW key to the viewer's key symbol. Printable keys fall back to
/// their US-layout token, lowercase unless Shift is held.
inline int viewerEventKey(int key, int mods)
{
  if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z)
    return ((mods & GLFW_MOD_SHIFT) ? 'A' : 'a') + key - GLFW_KEY_A;
  if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F12)
    return GUIEventAdapter::KEY_F1 + key - GLFW_KEY_F1;
  switch (key) {
    case GLFW_KEY_LEFT:
      return GUIEventAdapter::KEY_Left;
    case GLFW_KEY_RIGHT:
      return GUIEventAdapter::KEY_Right;
    case GLFW_KEY_UP:
      return GUIEventAdapter::KEY_Up;
    case GLFW_KEY_DOWN:
      return GUIEventAdapter::KEY_Down;
    case GLFW_KEY_ESCAPE:
      return GUIEventAdapter::KEY_Escape;
    case GLFW_KEY_HOME:
      return GUIEventAdapter::KEY_Home;
    case GLFW_KEY_END:
      return GUIEventAdapter::KEY_End;
    case GLFW_KEY_PAGE_UP:
      return GUIEventAdapter::KEY_Page_Up;
    case GLFW_KEY_PAGE_DOWN:
      return GUIEventAdapter::KEY_Page_Down;
    case GLFW_KEY_INSERT:
      return GUIEventAdapter::KEY_Insert;
    case GLFW_KEY_DELETE:
      return GUIEventAdapter::KEY_Delete;
    case GLFW_KEY_BACKSPACE:
      return GUIEventAdapter::KEY_BackSpace;
    case GLFW_KEY_TAB:
      return GUIEventAdapter::KEY_Tab;
    case GLFW_KEY_ENTER:
      return GUIEventAdapter::KEY_Return;
    case GLFW_KEY_LEFT_SHIFT:
      return GUIEventAdapter::KEY_Shift_L;
    case GLFW_KEY_RIGHT_SHIFT:
      return GUIEventAdapter::KEY_Shift_R;
    case GLFW_KEY_LEFT_CONTROL:
      return GUIEventAdapter::KEY_Control_L;
    case GLFW_KEY_RIGHT_CONTROL:
      return GUIEventAdapter::KEY_Control_R;
    case GLFW_KEY_LEFT_ALT:
      return GUIEventAdapter::KEY_Alt_L;
    case GLFW_KEY_RIGHT_ALT:
      return GUIEventAdapter::KEY_Alt_R;
    default:
      return key;
  }
}

/// Pairs each printable GLFW key press with the layout- and Shift-aware
/// character GLFW reports right after it, as OSG's key symbols were.
/// Held keys repeat as presses; a release reports the symbol of its press.
class KeySymbolTranslator
{
public:
  struct Event
  {
    bool down = true;
    int symbol = 0;
    int mods = 0;
  };

  /// Handles a key callback. Printable presses wait for character().
  std::optional<Event> key(int key, int action, int mods)
  {
    if (action == GLFW_RELEASE) {
      int symbol = viewerEventKey(key, mods);
      if (const auto it = mPressed.find(key); it != mPressed.end()) {
        symbol = it->second;
        mPressed.erase(it);
      }
      return Event{false, symbol, mods};
    }
    // Ctrl+Alt is AltGr on Windows, which still types a character.
    const int ctrlAlt = mods & (GLFW_MOD_CONTROL | GLFW_MOD_ALT);
    const bool typesText
        = (mods & GLFW_MOD_SUPER) == 0
          && (ctrlAlt == 0 || ctrlAlt == (GLFW_MOD_CONTROL | GLFW_MOD_ALT));
    const bool printable
        = (key >= GLFW_KEY_SPACE && key <= GLFW_KEY_GRAVE_ACCENT)
          || key == GLFW_KEY_WORLD_1 || key == GLFW_KEY_WORLD_2
          || (key >= GLFW_KEY_KP_0 && key <= GLFW_KEY_KP_ADD)
          || key == GLFW_KEY_KP_EQUAL;
    if (printable && typesText) {
      mPending = Pending{key, mods};
      return std::nullopt;
    }
    return press(key, viewerEventKey(key, mods), mods);
  }

  /// Completes the pending printable press with its typed character.
  std::optional<Event> character(unsigned int codepoint)
  {
    if (!mPending)
      return std::nullopt;
    const Pending pending = *mPending;
    mPending.reset();
    return press(pending.key, static_cast<int>(codepoint), pending.mods);
  }

  /// Delivers a printable press whose character never arrived (e.g. a dead
  /// key) with the fallback symbol.
  std::optional<Event> flush()
  {
    if (!mPending)
      return std::nullopt;
    const Pending pending = *mPending;
    mPending.reset();
    return press(
        pending.key, viewerEventKey(pending.key, pending.mods), pending.mods);
  }

private:
  /// Every delivered press records its symbol so the release reports it.
  Event press(int key, int symbol, int mods)
  {
    mPressed[key] = symbol;
    return Event{true, symbol, mods};
  }

  struct Pending
  {
    int key;
    int mods;
  };
  std::optional<Pending> mPending;
  std::unordered_map<int, int> mPressed;
};

} // namespace dart::gui::detail
