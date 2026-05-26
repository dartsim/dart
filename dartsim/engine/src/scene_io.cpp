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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dartsim_engine/scene_io.hpp>
#include <dartsim_engine/scene_model.hpp>
#include <dartsim_engine/scene_object.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iterator>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dartsim::scene_io {

namespace {

/// Highest scene-file format version this build can read and writes on save.
constexpr int kSceneFormatVersion = 1;

void writeRow(std::ostream& out, const char* key, const double* data, int n)
{
  out << key;
  for (int i = 0; i < n; ++i) {
    out << ' ' << data[i];
  }
  out << '\n';
}

void writeMatrix4(std::ostream& out, const Eigen::Matrix4d& m)
{
  out << "transform";
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out << ' ' << m(r, c);
    }
  }
  out << '\n';
}

void writeMatrix3(std::ostream& out, const Eigen::Matrix3d& m)
{
  out << "inertia";
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out << ' ' << m(r, c);
    }
  }
  out << '\n';
}

bool readMatrix4(std::istream& in, Eigen::Isometry3d& transform)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (!(in >> m(r, c))) {
        return false;
      }
    }
  }
  transform.matrix() = m;
  return true;
}

bool readMatrix3(std::istream& in, Eigen::Matrix3d& m)
{
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (!(in >> m(r, c))) {
        return false;
      }
    }
  }
  return true;
}

template <int N>
bool readVector(std::istream& in, Eigen::Matrix<double, N, 1>& v)
{
  for (int i = 0; i < N; ++i) {
    if (!(in >> v[i])) {
      return false;
    }
  }
  return true;
}

} // namespace

std::string save(const SceneModel& model)
{
  std::ostringstream out;
  out << std::setprecision(17);
  out << "dartsim-scene " << kSceneFormatVersion << "\n";
  out << "timestep " << model.timeStep << '\n';

  for (const ObjectId id : model.allIds()) {
    const SceneObject* object = model.find(id);
    if (object == nullptr) {
      continue;
    }
    out << "object\n";
    out << "id " << object->id << '\n';
    out << "type " << static_cast<int>(object->type) << '\n';
    out << "parent " << object->parent << '\n';
    out << "visible " << (object->visible ? 1 : 0) << '\n';
    out << "multibody " << object->multiBody << '\n';
    out << "parentlink " << object->parentLink << '\n';
    out << "jointkind " << static_cast<int>(object->jointType) << '\n';
    out << "shapetype " << static_cast<int>(object->shape.type) << '\n';
    writeMatrix4(out, object->transform.matrix());
    writeRow(out, "linvel", object->linearVelocity.data(), 3);
    writeRow(out, "angvel", object->angularVelocity.data(), 3);
    out << "mass " << object->mass << '\n';
    writeMatrix3(out, object->inertia);
    writeRow(out, "dim", object->shape.dimensions.data(), 3);
    writeRow(out, "color", object->shape.color.data(), 4);
    writeRow(out, "jointaxis", object->jointAxis.data(), 3);
    out << "jointpos " << object->jointPosition << '\n';
    out << "name " << object->name << '\n';
    out << "end\n";
  }
  return out.str();
}

bool load(std::string_view text, SceneModel& out)
{
  std::istringstream in{std::string(text)};
  std::string line;

  if (!std::getline(in, line)) {
    return false;
  }
  {
    std::istringstream header(line);
    std::string tag;
    int version = 0;
    if (!(header >> tag >> version) || tag != "dartsim-scene") {
      return false;
    }
    // Reject unsupported versions instead of parsing newer files with v1 rules.
    if (version < 1 || version > kSceneFormatVersion) {
      return false;
    }
  }

  SceneModel result;
  std::vector<SceneObject> objects;
  SceneObject current;
  bool inObject = false;

  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string key;
    ls >> key;
    if (key.empty()) {
      continue;
    }

    if (key == "timestep") {
      if (!(ls >> result.timeStep)) {
        return false;
      }
    } else if (key == "object") {
      if (inObject) {
        return false;
      }
      current = SceneObject{};
      inObject = true;
    } else if (key == "end") {
      if (inObject) {
        objects.push_back(current);
        inObject = false;
      }
    } else if (inObject) {
      if (key == "id") {
        if (!(ls >> current.id)) {
          return false;
        }
      } else if (key == "type") {
        int value = 0;
        if (!(ls >> value)) {
          return false;
        }
        current.type = static_cast<ObjectType>(value);
      } else if (key == "parent") {
        if (!(ls >> current.parent)) {
          return false;
        }
      } else if (key == "visible") {
        int value = 1;
        if (!(ls >> value)) {
          return false;
        }
        current.visible = value != 0;
      } else if (key == "multibody") {
        if (!(ls >> current.multiBody)) {
          return false;
        }
      } else if (key == "parentlink") {
        if (!(ls >> current.parentLink)) {
          return false;
        }
      } else if (key == "jointkind") {
        int value = 0;
        if (!(ls >> value)) {
          return false;
        }
        current.jointType = static_cast<JointKind>(value);
      } else if (key == "shapetype") {
        int value = 0;
        if (!(ls >> value)) {
          return false;
        }
        current.shape.type = static_cast<ShapeType>(value);
      } else if (key == "transform") {
        if (!readMatrix4(ls, current.transform)) {
          return false;
        }
      } else if (key == "linvel") {
        if (!readVector<3>(ls, current.linearVelocity)) {
          return false;
        }
      } else if (key == "angvel") {
        if (!readVector<3>(ls, current.angularVelocity)) {
          return false;
        }
      } else if (key == "mass") {
        if (!(ls >> current.mass)) {
          return false;
        }
      } else if (key == "inertia") {
        if (!readMatrix3(ls, current.inertia)) {
          return false;
        }
      } else if (key == "dim") {
        if (!readVector<3>(ls, current.shape.dimensions)) {
          return false;
        }
      } else if (key == "color") {
        if (!readVector<4>(ls, current.shape.color)) {
          return false;
        }
      } else if (key == "jointaxis") {
        if (!readVector<3>(ls, current.jointAxis)) {
          return false;
        }
      } else if (key == "jointpos") {
        if (!(ls >> current.jointPosition)) {
          return false;
        }
      } else if (key == "name") {
        std::string rest;
        std::getline(ls, rest);
        if (!rest.empty() && rest.front() == ' ') {
          rest.erase(rest.begin());
        }
        current.name = rest;
      }
    }
  }
  if (inObject) {
    return false;
  }

  std::unordered_set<ObjectId> objectIds;
  for (const SceneObject& object : objects) {
    if (!objectIds.insert(object.id).second) {
      return false;
    }
  }

  // Insert parent-before-child so tree links resolve, preserving ids.
  std::unordered_set<ObjectId> inserted;
  bool progressed = true;
  while (inserted.size() < objects.size() && progressed) {
    progressed = false;
    for (const SceneObject& object : objects) {
      if (inserted.count(object.id) != 0) {
        continue;
      }
      if (object.parent == kNoObject || inserted.count(object.parent) != 0) {
        result.add(object);
        inserted.insert(object.id);
        progressed = true;
      }
    }
  }
  // Attach any malformed leftovers (cycle/missing parent) to the world root.
  for (const SceneObject& object : objects) {
    if (inserted.count(object.id) == 0) {
      SceneObject copy = object;
      copy.parent = kNoObject;
      result.add(copy);
    }
  }

  out = std::move(result);
  return true;
}

bool saveToFile(const std::string& path, const SceneModel& model)
{
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }
  file << save(model);
  return static_cast<bool>(file);
}

bool loadFromFile(const std::string& path, SceneModel& out)
{
  std::error_code error;
  if (!std::filesystem::is_regular_file(std::filesystem::path(path), error)) {
    return false;
  }

  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }
  const std::string text(
      (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return load(text, out);
}

} // namespace dartsim::scene_io
