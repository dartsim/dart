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

#include <dart/simulation/world.hpp>

#include <dartsim_engine/recorder.hpp>

#include <istream>
#include <ostream>
#include <sstream>

#include <cstdint>

namespace dartsim {

namespace {

constexpr char kMagic[6] = {'D', 'S', 'R', 'E', 'C', '1'};

template <typename T>
void writePod(std::ostream& out, const T& value)
{
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template <typename T>
bool readPod(std::istream& in, T& value)
{
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  return static_cast<bool>(in);
}

} // namespace

void Recording::save(std::ostream& output) const
{
  output.write(kMagic, sizeof(kMagic));
  writePod(output, timeStep);
  writePod<std::uint64_t>(output, m_frames.size());
  for (const RecordedFrame& frame : m_frames) {
    writePod(output, frame.time);
    writePod<std::uint64_t>(output, frame.step);
    writePod<std::uint64_t>(output, frame.blob.size());
    output.write(
        frame.blob.data(), static_cast<std::streamsize>(frame.blob.size()));
  }
}

bool Recording::load(std::istream& input)
{
  char magic[sizeof(kMagic)] = {};
  input.read(magic, sizeof(magic));
  if (!input) {
    return false;
  }
  for (std::size_t i = 0; i < sizeof(kMagic); ++i) {
    if (magic[i] != kMagic[i]) {
      return false;
    }
  }

  Recording loaded;
  if (!readPod(input, loaded.timeStep)) {
    return false;
  }
  std::uint64_t count = 0;
  if (!readPod(input, count)) {
    return false;
  }
  loaded.m_frames.reserve(static_cast<std::size_t>(count));
  for (std::uint64_t i = 0; i < count; ++i) {
    RecordedFrame frame;
    std::uint64_t step = 0;
    std::uint64_t length = 0;
    if (!readPod(input, frame.time) || !readPod(input, step)
        || !readPod(input, length)) {
      return false;
    }
    frame.step = static_cast<std::size_t>(step);
    frame.blob.resize(static_cast<std::size_t>(length));
    if (length > 0) {
      input.read(frame.blob.data(), static_cast<std::streamsize>(length));
      if (!input) {
        return false;
      }
    }
    loaded.m_frames.push_back(std::move(frame));
  }

  *this = std::move(loaded);
  return true;
}

void Recorder::start(double timeStep)
{
  m_data.clear();
  m_data.timeStep = timeStep;
  m_recording = true;
}

void Recorder::stop()
{
  m_recording = false;
}

void Recorder::capture(const dart::simulation::World& world)
{
  if (!m_recording) {
    return;
  }
  std::ostringstream stream(std::ios::binary);
  world.saveBinary(stream);
  RecordedFrame frame;
  frame.time = world.getTime();
  frame.step = world.getFrame();
  frame.blob = stream.str();
  m_data.frames().push_back(std::move(frame));
}

void Recorder::clear()
{
  m_data.clear();
  m_recording = false;
}

void Player::setRecording(Recording recording)
{
  m_data = std::move(recording);
  m_index = 0;
}

bool Player::seek(dart::simulation::World& world, std::size_t index)
{
  if (index >= m_data.frameCount()) {
    return false;
  }
  std::istringstream stream(m_data.frames()[index].blob, std::ios::binary);
  world.loadBinary(stream);
  m_index = index;
  return true;
}

} // namespace dartsim
