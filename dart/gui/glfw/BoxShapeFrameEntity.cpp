#include "dart/gui/glfw/BoxShapeFrameEntity.hpp"

namespace dart {
namespace gui {
namespace glfw {

BoxShapeFrameEntity::BoxShapeFrameEntity(dynamics::ShapeFrame* shapeFrame, dynamics::BoxShapePtr boxShape)
    : ShapeFrameEntity(shapeFrame), mBoxShape(std::move(boxShape))
{
  update();
}

void BoxShapeFrameEntity::update(bool shortCircuitIfUtilized)
{
  ShapeFrameEntity::update(shortCircuitIfUtilized);
}

} // namespace glfw
} // namespace gui
} // namespace dart
