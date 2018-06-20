#ifndef DART_GUI_GLFW_SHAPEFRAMEENTITY_HPP_
#define DART_GUI_GLFW_SHAPEFRAMEENTITY_HPP_

#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/gui/glfw/Entity.hpp"

namespace dart {
namespace gui {
namespace glfw {

class ShapeFrameEntity : public Entity
{
public:
  ShapeFrameEntity(dynamics::ShapeFrame* shapeFrame);
  // NOTE: We take a raw pointer of ShapeFrame instead as in std::shared_ptr
  // because ShapeFrame will live longer than Entity as long as WorldNode lives
  // longer than the World holding the ShapeFrame, which is gauranteed by the
  // fact that WorldNode holds the World in std::shared_ptr.

  /// Pointer to the ShapeFrame associated with this ShapeFrameNode
  dynamics::ShapeFrame* getShapeFrame();

  /// Pointer to the ShapeFrame associated with this ShapeFrameNode
  const dynamics::ShapeFrame* getShapeFrame() const;

  void update(bool shortCircuitIfUtilized = false) override;

protected:
  void refreshShapeNode(const std::shared_ptr<dart::dynamics::Shape>& shape);

  void createShapeNode(const std::shared_ptr<dart::dynamics::Shape>& shape);

  /// Pointer to the ShapeFrame that this ShapeFrameNode is associated with
  dart::dynamics::ShapeFrame* mShapeFrame;

  /// True iff this ShapeFrameNode has been utilized on the latest update.
  /// If it has not, that is an indication that it is no longer being
  /// used and should be deleted.
  bool mUtilized;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_SHAPEFRAMEENTITY_HPP_
