#include "dart/gui/Shader.hpp"

#include <fstream>
#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"

namespace dart {
namespace gui {

//==============================================================================
Shader::Shader(GLuint type, const common::Uri& shaderUri, common::ResourceRetriever* retriever)
{
  std::unique_ptr<common::ResourceRetriever> localRetriever;
  if (!retriever)
  {
    localRetriever.reset(new common::LocalResourceRetriever());
    retriever = localRetriever.get();
  }

  auto shaderStr = retriever->readAll(shaderUri);
  const char* c_str = shaderStr.c_str();

  // Create the OpenGL vertex shader and compile it
  mShaderId = glCreateShader(type);
  assert(mShaderId != 0);
  glShaderSource(mShaderId, 1, &c_str, NULL);
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
    char* str = new char[lengthLog];
    glGetShaderInfoLog(mShaderId, lengthLog, NULL, str);

    // Display the log of the compilation
    dterr << "Vertex Shader Error (in " << shaderUri.toString()
          << ") : " << str;
    delete[] str;
    assert(false);
    return;
  }
}

//==============================================================================
Shader::~Shader()
{
  glDeleteShader(mShaderId);
}

//==============================================================================
GLuint Shader::getId() const
{
  return mShaderId;
}

} // namespace gui
} // namespace dart
