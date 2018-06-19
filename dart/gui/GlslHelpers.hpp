#ifndef DART_GUI_GLSLHELPERS_HPP_
#define DART_GUI_GLSLHELPERS_HPP_

#include <string>
#include "dart/gui/glfw/LoadGlfw.hpp"  // TODO(JS): Fix loading OpenGL

namespace dart {
namespace gui {

void setIntUniform(
    GLuint mProgramObjectId,
    const std::string& variableName,
    int value,
    bool errorIfMissing = false);

} // namespace gui
} // namespace dart

#endif // DART_GUI_GLSLHELPERS_HPP_
