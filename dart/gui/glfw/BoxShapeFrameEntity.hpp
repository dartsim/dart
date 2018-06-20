#ifndef DART_GUI_GLFW_BOXSHAPEFRAMEENTITY_HPP_
#define DART_GUI_GLFW_BOXSHAPEFRAMEENTITY_HPP_

#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/gui/glfw/ShapeFrameEntity.hpp"

namespace dart {
namespace gui {
namespace glfw {

class BoxShapeFrameEntity : public ShapeFrameEntity
{
public:
  BoxShapeFrameEntity(dynamics::ShapeFrame* shapeFrame, dynamics::BoxShapePtr boxShape);
  // NOTE: We take a raw pointer of ShapeFrame instead as in std::shared_ptr
  // because ShapeFrame will live longer than Entity as long as WorldNode lives
  // longer than the World holding the ShapeFrame, which is gauranteed by the
  // fact that WorldNode holds the World in std::shared_ptr.

  void update(bool shortCircuitIfUtilized = false) override;

protected:
dynamics::BoxShapePtr mBoxShape;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_BOXSHAPEFRAMEENTITY_HPP_
