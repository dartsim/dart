#include "dart/gui/GlslHelpers.hpp"

#include <cassert>
#include "dart/common/Console.hpp"

namespace dart {
namespace gui {

//==============================================================================
void setIntUniform(
    GLuint mProgramObjectId,
    const std::string& variableName,
    int value,
    bool errorIfMissing)
{
  assert(mProgramObjectId != 0);
  const auto location
      = glGetUniformLocation(mProgramObjectId, variableName.c_str());
  if (location == -1 && errorIfMissing)
  {
    dterr << "Error in vertex shader or in fragment shader: "
          << "No Uniform variable : " << variableName;
    throw std::logic_error("Error in Shader");
  }

  if (location != -1)
    glUniform1i(location, value);
}

} // namespace gui
} // namespace dart
