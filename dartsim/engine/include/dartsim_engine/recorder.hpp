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

#include <iosfwd>
#include <string>
#include <vector>

#include <cstddef>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dartsim {

/// One captured frame: sim time, step index, and a DART 7 World binary
/// snapshot (World::saveBinary output).
struct RecordedFrame
{
  double time = 0.0;
  std::size_t step = 0;
  std::string blob;
};

/// An ordered sequence of world snapshots that can be replayed and stored.
class Recording
{
public:
  double timeStep = 0.001;

  [[nodiscard]] const std::vector<RecordedFrame>& frames() const
  {
    return m_frames;
  }
  [[nodiscard]] std::vector<RecordedFrame>& frames()
  {
    return m_frames;
  }
  [[nodiscard]] std::size_t frameCount() const
  {
    return m_frames.size();
  }
  [[nodiscard]] bool empty() const
  {
    return m_frames.empty();
  }
  void clear()
  {
    m_frames.clear();
  }

  /// Serialize to a self-describing binary stream.
  void save(std::ostream& output) const;
  /// Load from a stream written by save(); returns false on a malformed stream.
  bool load(std::istream& input);

private:
  std::vector<RecordedFrame> m_frames;
};

/// Captures world snapshots into a Recording while active.
class Recorder
{
public:
  void start(double timeStep);
  void stop();
  [[nodiscard]] bool isRecording() const
  {
    return m_recording;
  }

  /// Append a snapshot of `world` if recording is active.
  void capture(const dart::simulation::World& world);

  [[nodiscard]] const Recording& recording() const
  {
    return m_data;
  }
  void clear();

private:
  Recording m_data;
  bool m_recording = false;
};

/// Restores recorded frames into a world for scrub-replay (no re-simulation).
class Player
{
public:
  void setRecording(Recording recording);
  [[nodiscard]] const Recording& recording() const
  {
    return m_data;
  }
  [[nodiscard]] std::size_t frameCount() const
  {
    return m_data.frameCount();
  }
  [[nodiscard]] std::size_t currentIndex() const
  {
    return m_index;
  }
  [[nodiscard]] bool empty() const
  {
    return m_data.empty();
  }

  /// Restore frame `index` into `world`. Returns false if out of range.
  bool seek(dart::simulation::World& world, std::size_t index);

  /// Drop the loaded recording and rewind (e.g. when loading a new project).
  void clear()
  {
    m_data = {};
    m_index = 0;
  }

private:
  Recording m_data;
  std::size_t m_index = 0;
};

} // namespace dartsim
