#include "dart/gui/glfw/ShapeFrameEntity.hpp"

namespace dart {
namespace gui {
namespace glfw {

ShapeFrameEntity::ShapeFrameEntity(dynamics::ShapeFrame* shapeFrame)
    : Entity(), mShapeFrame(shapeFrame), mUtilized(false)
{
  assert(shapeFrame);

  update();
}

dynamics::ShapeFrame*ShapeFrameEntity::getShapeFrame()
{
  return mShapeFrame;
}

const dynamics::ShapeFrame*ShapeFrameEntity::getShapeFrame() const
{
  return mShapeFrame;
}

void ShapeFrameEntity::update(bool shortCircuitIfUtilized)
{
  if(shortCircuitIfUtilized && mUtilized)
    return;

  mUtilized = true;

  setTransform(mShapeFrame->getWorldTransform().cast<float>());
  // TODO(JS): Maybe the data varicance information should be in ShapeFrame and
  // checked here.

  auto shape = mShapeFrame->getShape();
  if(shape && mShapeFrame->getVisualAspect())
  {
    refreshShapeNode(shape);
  }
  // No need to render if shape or VisualAspect is removed.
//  else if(mRenderShapeNode)
//  {
//    removeChild(mRenderShapeNode->getNode());
//    mRenderShapeNode = nullptr;
//  }
}

void ShapeFrameEntity::refreshShapeNode(const std::shared_ptr<dynamics::Shape>& shape)
{

}

void ShapeFrameEntity::createShapeNode(const std::shared_ptr<dynamics::Shape>& shape)
{

}

} // namespace glfw
} // namespace gui
} // namespace dart
