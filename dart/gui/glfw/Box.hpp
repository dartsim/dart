#ifndef DART_GUI_GLFW_BOX_HPP_
#define DART_GUI_GLFW_BOX_HPP_

#include <unordered_map>

#include <Eigen/Dense>

#include "dart/common/SmartPointer.hpp"
#include "dart/gui/VertexArrayObject.hpp"
#include "dart/gui/VertexBufferObject.hpp"
#include "dart/gui/glfw/Entity.hpp"

namespace dart {
namespace gui {
namespace glfw {

class Box : public Entity
{
public:
  Box() = default;
};

} // namespace glfw
} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_BOX_HPP_
