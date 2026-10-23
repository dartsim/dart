/* Copyright (c) 2026, The DART development contributors. All rights reserved.
 */

#include "detail/dart_nb.hpp"

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionGroup.hpp>

#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>

#include <dart/common/Macros.hpp>

#include <assimp/scene.h>

#include <memory>
#include <stdexcept>

#include <cstdint>

namespace {

template <class T>
T* checked(nb::handle value)
{
  if (!nb::isinstance<T>(value) || !nb::inst_ready(value))
    throw nb::type_error("Expected an initialized dartpy native owner");
  return nb::inst_ptr<T>(value);
}

void seedMesh(nb::handle value)
{
  auto* shape = checked<dart::dynamics::MeshShape>(value);
  auto scene = std::make_shared<aiScene>();
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh* [1] {
    new aiMesh
  };
  auto* mesh = scene->mMeshes[0];
  mesh->mNumVertices = 4;
  mesh->mVertices
      = new aiVector3D[4]{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  mesh->mNumFaces = 4;
  mesh->mFaces = new aiFace[4];
  const unsigned int faces[4][3]{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
  for (int i = 0; i < 4; ++i) {
    mesh->mFaces[i].mNumIndices = 3;
    mesh->mFaces[i].mIndices
        = new unsigned int[3]{faces[i][0], faces[i][1], faces[i][2]};
  }
  scene->mRootNode = new aiNode;
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1]{0};
  DART_SUPPRESS_DEPRECATED_BEGIN
  shape->setMesh(
      std::move(scene), shape->getMeshUri2(), shape->getResourceRetriever());
  DART_SUPPRESS_DEPRECATED_END
}

nb::tuple snapshot(nb::handle worldValue, nb::handle shapeValue)
{
  auto* world = checked<dart::simulation::World>(worldValue);
  auto* shape = checked<dart::dynamics::MeshShape>(shapeValue);
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto* scene = shape->getMesh();
  DART_SUPPRESS_DEPRECATED_END
  nb::list vertices;
  if (scene) {
    for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
      const auto* mesh = scene->mMeshes[i];
      for (unsigned int j = 0; j < mesh->mNumVertices; ++j) {
        const auto& vertex = mesh->mVertices[j];
        vertices.append(nb::make_tuple(vertex.x, vertex.y, vertex.z));
      }
    }
  }
  const auto group = world->getConstraintSolver()->getCollisionGroup();
  std::size_t ownedFrames = 0;
  for (std::size_t i = 0; i < group->getNumShapeFrames(); ++i) {
    if (group->getShapeFrame(i)->getShape().get() == shape)
      ++ownedFrames;
  }
  return nb::make_tuple(
      world->isCompiled(),
      world->isInSimulationMode(),
      world->getTopologyRevision(),
      world->getParameterRevision(),
      shape->getVersion(),
      reinterpret_cast<std::uintptr_t>(world),
      reinterpret_cast<std::uintptr_t>(scene),
      reinterpret_cast<std::uintptr_t>(shape->getResourceRetriever().get()),
      group->getNumShapeFrames(),
      ownedFrames,
      nb::tuple(vertices));
}

void clearShape(nb::handle value)
{
  checked<dart::dynamics::MeshShape>(value);
  auto* slot = PyType_GetSlot(
      reinterpret_cast<PyTypeObject*>(Py_TYPE(value.ptr())), Py_tp_clear);
  if (!slot)
    throw std::runtime_error("MeshShape has no registered GC clear slot");
  if (reinterpret_cast<inquiry>(slot)(value.ptr()) != 0)
    throw nb::python_error();
}

} // namespace

NB_MODULE(_dartpy_gc_test, module)
{
  module.def("seed_mesh", &seedMesh);
  module.def("compile_world", [](nb::handle value) {
    checked<dart::simulation::World>(value)->compile();
  });
  module.def("snapshot", &snapshot);
  module.def("clear_shape", &clearShape);
}
