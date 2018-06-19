#ifndef DART_GUI_RENDERSTATE_HPP_
#define DART_GUI_RENDERSTATE_HPP_

#include "dart/gui/LoadOpengl.hpp"
#include "dart/gui/glfw/Entity.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/gui/Program.hpp"

namespace dart {
namespace gui {

struct RenderState
{
  Program* mProgram;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLFW_RENDERSTATE_HPP_
