#ifndef DART_GUI_SHADER_IMPL_HPP_
#define DART_GUI_SHADER_IMPL_HPP_

#include "dart/gui/Shader.hpp"

#include <cassert>
#include <vector>
#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"

namespace dart {
namespace gui {

//==============================================================================
template <GLuint ShaderType>
Shader<ShaderType>::Shader(const char* shaderString)
{
  compile(shaderString);
}

//==============================================================================
template <GLuint ShaderType>
Shader<ShaderType>::Shader(const std::string& shaderString)
  : Shader(shaderString.c_str())
{
  // Do nothing
}

//==============================================================================
template <GLuint ShaderType>
Shader<ShaderType>::Shader(
    const common::Uri& shaderUri, common::ResourceRetriever* retriever)
{
  std::unique_ptr<common::ResourceRetriever> localRetriever;
  if (!retriever)
  {
    localRetriever.reset(new common::LocalResourceRetriever());
    retriever = localRetriever.get();
  }

  auto shaderStr = retriever->readAll(shaderUri);
  compile(shaderStr.c_str());
}

//==============================================================================
template <GLuint ShaderType>
Shader<ShaderType>::~Shader()
{
  glDeleteShader(mShaderId);
}

//==============================================================================
template <GLuint ShaderType>
GLuint Shader<ShaderType>::getId() const
{
  return mShaderId;
}

//==============================================================================
template <GLuint ShaderType>
void Shader<ShaderType>::compile(const char* shaderString)
{
  // Create the OpenGL vertex shader and compile it
  mShaderId = glCreateShader(ShaderType);

  assert(mShaderId != GL_INVALID_ENUM);
  // ShaderType is not an accepted value

  assert(mShaderId != 0);
  // If glCreateShader returns 0, make sure an OpenGL context is activated
  // by calling glfwMakeContextCurrent() function beforehand.

  glShaderSource(mShaderId, 1, &shaderString, NULL);
  glCompileShader(mShaderId);

  // Get the compilation information
  int compiled;
  glGetShaderiv(mShaderId, GL_COMPILE_STATUS, &compiled);

  // If the compilation failed
  if (compiled == 0)
  {
    // Get the log of the compilation
    int lengthLog;
    glGetShaderiv(mShaderId, GL_INFO_LOG_LENGTH, &lengthLog);
    std::vector<char> str(static_cast<std::size_t>(lengthLog));
    glGetShaderInfoLog(mShaderId, lengthLog, NULL, str.data());

    // Display the log of the compilation
    dterr << "Vertex Shader Error: " << std::string(str.begin(), str.end()) << "\n";
    assert(false);

    // TODO(JS): Needs better error handling

    return;
  }
}

} // namespace gui
} // namespace dart

#endif // DART_GUI_SHADER_IMPL_HPP_
