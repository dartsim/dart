#include "dart/gui/glfw/WorldScene.hpp"

#include <deque>
#include <tuple>
#include "dart/common/StlHelpers.hpp"
#include "dart/gui/glfw/Entity.hpp"
#include "dart/math/Constants.hpp"
#include "dart/gui/glfw/Box.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/gui/glfw/ShapeFrameEntity.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/gui/glfw/BoxShapeFrameEntity.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
WorldScene::WorldScene(simulation::WorldPtr world, const std::string& name, GLFWwindow* window)
  : Scene(name, window)
{
  setWorld(world);
}

//==============================================================================
void WorldScene::setWorld(simulation::WorldPtr world)
{
  if (world == mWorld)
    return;

  mWorld = std::move(world);
}

//==============================================================================
simulation::WorldPtr WorldScene::getWorld()
{
  return mWorld;
}

//==============================================================================
simulation::ConstWorldPtr WorldScene::getWorld() const
{
  return mWorld;
}

//==============================================================================
void WorldScene::update()
{
  updateWorld();
}

//==============================================================================
void WorldScene::updateWorld()
{
  updateSkeletons();
}

//==============================================================================
void WorldScene::updateSkeletons()
{
  if (!mWorld)
    return;

  for (auto i = 0u; i < mWorld->getNumSkeletons(); ++i)
  {
    auto skeleton = mWorld->getSkeleton(i);

    for (auto i = 0u; i < skeleton->getNumTrees(); ++i)
      refreshBaseFrameNode(skeleton->getRootBodyNode(i));
  }
}

//==============================================================================
void WorldScene::refreshBaseFrameNode(dart::dynamics::Frame* frame)
{
  std::deque<dynamics::Frame*> frames;

  frames.push_back(frame);
  while (!frames.empty())
  {
    auto* nextFrame = frames.front();
    frames.pop_front();
    if (nextFrame->isShapeFrame())
      refreshShapeFrameNode(nextFrame);

    const auto& childFrames = nextFrame->getChildFrames();

    for (auto* child : childFrames)
      frames.push_back(child);
  }
}

void WorldScene::refreshShapeFrameNode(dynamics::Frame* frame)
{
  auto insertion = mFrameToNode.insert(std::make_pair(frame, nullptr));
  const auto& it = insertion.first;

  const auto frameAlreadyInserted = !insertion.second;
  if (frameAlreadyInserted)
  {
    auto node = it->second;

    // node shouldn't be a nullptr
    assert(node);

    node->update(true);
    return;
  }

  if (!frame->isShapeFrame())
  {
    dtwarn << "[WorldScene::refreshShapeFrameNode] Frame named ["
           << frame->getName() << "] (" << frame << ") claims to be a "
           << "ShapeFrame, but failed to be converted. Please report this as a "
           << "bug!\n";
    return;
  }

  auto shapeFrame = frame->asShapeFrame();
  auto shape = shapeFrame->getShape();
  const auto& shapeType = shape->getType();

  std::shared_ptr<ShapeFrameEntity> node;

  // TODO(JS): Improve
  if(dynamics::BoxShape::getStaticType() == shapeType)
  {
    auto bs = std::dynamic_pointer_cast<dynamics::BoxShape>(shape);
    if (bs)
      node = std::make_shared<BoxShapeFrameEntity>(shapeFrame, bs);
    it->second = node;
    mEntities.insert(node);
  }

//  auto node = std::make_shared<ShapeFrameEntity>(shapeFrame);

//  it->second = node;
//  mEntities.insert(node);
}

} // namespace glfw
} // namespace gui
} // namespace dart
