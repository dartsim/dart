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

#include <dartsim_engine/scene_object.hpp>

#include <functional>
#include <utility>
#include <vector>

namespace dartsim {

/// Categories of engine state changes broadcast to observers.
///
/// A typed observer surface so the GUI (and other consumers such as logging or
/// automation) react to engine changes instead of polling. The immediate-mode
/// editor still reads state directly each frame; the bus exists for
/// change-driven consumers and tests.
enum class EventType
{
  SceneChanged,      ///< Objects added/removed/edited (topology or properties).
  SelectionChanged,  ///< The selection set or primary selection changed.
  ModeChanged,       ///< Edit/Simulation mode or running state changed.
  SimulationChanged, ///< Runtime world state changed without an edit command.
  RecordingChanged,  ///< Recording started/stopped or a frame was captured.
  ProjectCreated,    ///< A new project replaced the current scene.
  ProjectSaved,      ///< The current project was saved.
  ProjectLoaded,     ///< A project was loaded, replacing the scene.
  ProjectStateChanged, ///< Project path or dirty/clean state changed.
};

/// A single broadcast event. `object` is set when the event concerns one node.
struct Event
{
  EventType type = EventType::SceneChanged;
  ObjectId object = kNoObject;
};

/// Multi-listener typed event dispatcher.
class EventBus
{
public:
  using Listener = std::function<void(const Event&)>;

  /// Subscribe a listener; returns a token usable with unsubscribe().
  int subscribe(Listener listener)
  {
    const int token = m_nextToken++;
    m_listeners.emplace_back(token, std::move(listener));
    return token;
  }

  void unsubscribe(int token)
  {
    for (auto it = m_listeners.begin(); it != m_listeners.end(); ++it) {
      if (it->first == token) {
        m_listeners.erase(it);
        return;
      }
    }
  }

  void emit(const Event& event) const
  {
    // Dispatch over a snapshot so a listener that subscribes or unsubscribes
    // (including unsubscribing itself) during the callback cannot invalidate
    // the iteration over m_listeners.
    const auto snapshot = m_listeners;
    for (const auto& [token, listener] : snapshot) {
      (void)token;
      if (listener) {
        listener(event);
      }
    }
  }

  void emit(EventType type, ObjectId object = kNoObject) const
  {
    emit(Event{type, object});
  }

  [[nodiscard]] std::size_t listenerCount() const
  {
    return m_listeners.size();
  }

private:
  std::vector<std::pair<int, Listener>> m_listeners;
  int m_nextToken = 1;
};

} // namespace dartsim
