#include <dart/dart.hpp>
#include <dart/simulation/World.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace dart {
namespace python {

void dart_simulation_World(pybind11::module& m)
{
  ::pybind11::class_<dart::simulation::World, std::shared_ptr<dart::simulation::World>>(m, "World")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const std::string &>(), ::pybind11::arg("_name"))
      .def("clone", +[](const dart::simulation::World *self) -> std::shared_ptr<dart::simulation::World> { return self->clone(); })
      .def("setName", +[](dart::simulation::World *self, const std::string & _newName) -> const std::string & { return self->setName(_newName); }, ::pybind11::return_value_policy::reference_internal, ::pybind11::arg("_newName"))
      .def("getName", +[](const dart::simulation::World *self) -> const std::string & { return self->getName(); }, ::pybind11::return_value_policy::reference_internal)
      .def("setGravity", +[](dart::simulation::World *self, const Eigen::Vector3d & _gravity) -> void { return self->setGravity(_gravity); }, ::pybind11::arg("_gravity"))
      .def("getGravity", +[](const dart::simulation::World *self) -> const Eigen::Vector3d & { return self->getGravity(); }, ::pybind11::return_value_policy::reference_internal)
      .def("setTimeStep", +[](dart::simulation::World *self, double _timeStep) -> void { return self->setTimeStep(_timeStep); }, ::pybind11::arg("_timeStep"))
      .def("getTimeStep", +[](const dart::simulation::World *self) -> double { return self->getTimeStep(); })
      .def("getSkeleton", +[](const dart::simulation::World *self, std::size_t _index) -> dart::dynamics::SkeletonPtr { return self->getSkeleton(_index); }, ::pybind11::arg("_index"))
      .def("getSkeleton", +[](const dart::simulation::World *self, const std::string & _name) -> dart::dynamics::SkeletonPtr { return self->getSkeleton(_name); }, ::pybind11::arg("_name"))
      .def("getNumSkeletons", +[](const dart::simulation::World *self) -> std::size_t { return self->getNumSkeletons(); })
      .def("addSkeleton", +[](dart::simulation::World *self, const dart::dynamics::SkeletonPtr & _skeleton) -> std::string { return self->addSkeleton(_skeleton); }, ::pybind11::arg("_skeleton"))
      .def("removeSkeleton", +[](dart::simulation::World *self, const dart::dynamics::SkeletonPtr & _skeleton) -> void { return self->removeSkeleton(_skeleton); }, ::pybind11::arg("_skeleton"))
      .def("removeAllSkeletons", +[](dart::simulation::World *self) -> std::set<dart::dynamics::SkeletonPtr> { return self->removeAllSkeletons(); })
      .def("hasSkeleton", +[](const dart::simulation::World *self, const dart::dynamics::ConstSkeletonPtr & skeleton) -> bool { return self->hasSkeleton(skeleton); }, ::pybind11::arg("skeleton"))
      .def("getIndex", +[](const dart::simulation::World *self, int _index) -> int { return self->getIndex(_index); }, ::pybind11::arg("_index"))
      .def("getSimpleFrame", +[](const dart::simulation::World *self, std::size_t _index) -> dart::dynamics::SimpleFramePtr { return self->getSimpleFrame(_index); }, ::pybind11::arg("_index"))
      .def("getSimpleFrame", +[](const dart::simulation::World *self, const std::string & _name) -> dart::dynamics::SimpleFramePtr { return self->getSimpleFrame(_name); }, ::pybind11::arg("_name"))
      .def("getNumSimpleFrames", +[](const dart::simulation::World *self) -> std::size_t { return self->getNumSimpleFrames(); })
      .def("addSimpleFrame", +[](dart::simulation::World *self, const dart::dynamics::SimpleFramePtr & _frame) -> std::string { return self->addSimpleFrame(_frame); }, ::pybind11::arg("_frame"))
      .def("removeSimpleFrame", +[](dart::simulation::World *self, const dart::dynamics::SimpleFramePtr & _frame) -> void { return self->removeSimpleFrame(_frame); }, ::pybind11::arg("_frame"))
      .def("removeAllSimpleFrames", +[](dart::simulation::World *self) -> std::set<dart::dynamics::SimpleFramePtr> { return self->removeAllSimpleFrames(); })
      .def("checkCollision", +[](dart::simulation::World *self, bool checkAllCollisions) -> bool { return self->checkCollision(checkAllCollisions); }, ::pybind11::arg("checkAllCollisions"))
      .def("checkCollision", +[](dart::simulation::World *self) -> bool { return self->checkCollision(); })
      .def("checkCollision", +[](dart::simulation::World *self, const dart::collision::CollisionOption & option) -> bool { return self->checkCollision(option); }, ::pybind11::arg("option"))
      .def("checkCollision", +[](dart::simulation::World *self, const dart::collision::CollisionOption & option, dart::collision::CollisionResult * result) -> bool { return self->checkCollision(option, result); }, ::pybind11::arg("option"), ::pybind11::arg("result"))
      .def("reset", +[](dart::simulation::World *self) -> void { return self->reset(); })
      .def("step", +[](dart::simulation::World *self) -> void { return self->step(); })
      .def("step", +[](dart::simulation::World *self, bool _resetCommand) -> void { return self->step(_resetCommand); }, ::pybind11::arg("_resetCommand"))
      .def("setTime", +[](dart::simulation::World *self, double _time) -> void { return self->setTime(_time); }, ::pybind11::arg("_time"))
      .def("getTime", +[](const dart::simulation::World *self) -> double { return self->getTime(); })
      .def("getSimFrames", +[](const dart::simulation::World *self) -> int { return self->getSimFrames(); })
      .def("bake", +[](dart::simulation::World *self) -> void { return self->bake(); })
      .def_static("create", +[]() -> std::shared_ptr<dart::simulation::World> { return dart::simulation::World::create(); })
      .def_static("create", +[](const std::string & name) -> std::shared_ptr<dart::simulation::World> { return dart::simulation::World::create(name); }, ::pybind11::arg("name"))
      .def_readonly("onNameChanged", &dart::simulation::World::onNameChanged)
  ;
}

} // namespace python
} // namespace dart
