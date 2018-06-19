#ifndef DART_GUI_GLFW_WORLDSCENE_HPP_
#define DART_GUI_GLFW_WORLDSCENE_HPP_

#include "dart/gui/glfw/Scene.hpp"
#include "dart/simulation/World.hpp"

namespace dart {
namespace gui {
namespace glfw {

// TODO(JS): docstring
class WorldScene : public Scene
{
public:
  WorldScene(simulation::WorldPtr world = nullptr);

  void setWorld(simulation::WorldPtr world);

  simulation::WorldPtr getWorld();

  simulation::ConstWorldPtr getWorld() const;

protected:
  simulation::WorldPtr mWorld;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_WORLDSCENE_HPP_
