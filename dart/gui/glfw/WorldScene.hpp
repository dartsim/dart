#ifndef DART_GUI_GLFW_WORLDSCENE_HPP_
#define DART_GUI_GLFW_WORLDSCENE_HPP_

#include <unordered_map>
#include "dart/gui/glfw/Scene.hpp"
#include "dart/simulation/World.hpp"

namespace dart {
namespace gui {
namespace glfw {

class ShapeFrameEntity;

// TODO(JS): docstring
class WorldScene : public Scene
{
public:
  WorldScene(simulation::WorldPtr world = nullptr,
             const std::string& name = "Default Scene", GLFWwindow* window = nullptr);

  void setWorld(simulation::WorldPtr world);

  simulation::WorldPtr getWorld();

  simulation::ConstWorldPtr getWorld() const;

  void update() override;


protected:
  void updateWorld();

  void updateSkeletons();

  void refreshBaseFrameNode(dart::dynamics::Frame* frame);

  void refreshShapeFrameNode(dart::dynamics::Frame* frame);

  simulation::WorldPtr mWorld;

  using NodeMap = std::unordered_map<dynamics::Frame*, std::shared_ptr<ShapeFrameEntity>>;
  // TODO(JS): Use ShapeFrameEntity (or ShapeFrameNode) instead of Entity

  /// Map from Frame pointers to FrameNode pointers
  NodeMap mFrameToNode;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_WORLDSCENE_HPP_
