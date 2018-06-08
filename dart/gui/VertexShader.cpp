#include "dart/gui/VertexShader.hpp"

#include <fstream>
#include "dart/common/Console.hpp"

namespace dart {
namespace gui {

//==============================================================================
VertexShader::VertexShader(const std::string& vertexShaderFilepath)
  : mVertexShaderFilepath(vertexShaderFilepath)
{
  assert(!vertexShaderFilepath.empty());

  // Set the shader filenames
  mVertexShaderFilepath = vertexShaderFilepath;

  std::ifstream fileVertexShader;
  fileVertexShader.open(vertexShaderFilepath.c_str(), std::ios::binary);

  if (!fileVertexShader.is_open())
  {
    dterr << "Error : Impossible to open the vertex shader file "
          << vertexShaderFilepath;
    assert(false);
    return;
  }

  // Get the size of the file
  fileVertexShader.seekg(0, std::ios::end);
  uint fileSize = (uint)(fileVertexShader.tellg());
  assert(fileSize != 0);

  // Read the file
  fileVertexShader.seekg(std::ios::beg);
  char* bufferVertexShader = new char[fileSize + 1];
  fileVertexShader.read(bufferVertexShader, fileSize);
  fileVertexShader.close();
  bufferVertexShader[fileSize] = '\0';

  // Create the OpenGL vertex shader and compile it
  mVertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  assert(mVertexShaderId != 0);
  glShaderSource(mVertexShaderId, 1, (const char**)(&bufferVertexShader), NULL);
  glCompileShader(mVertexShaderId);
  delete[] bufferVertexShader;

  // Get the compilation information
  int compiled;
  glGetShaderiv(mVertexShaderId, GL_COMPILE_STATUS, &compiled);

  // If the compilation failed
  if (compiled == 0)
  {
    // Get the log of the compilation
    int lengthLog;
    glGetShaderiv(mVertexShaderId, GL_INFO_LOG_LENGTH, &lengthLog);
    char* str = new char[lengthLog];
    glGetShaderInfoLog(mVertexShaderId, lengthLog, NULL, str);

    // Display the log of the compilation
    dterr << "Vertex Shader Error (in " << vertexShaderFilepath
          << ") : " << str;
    delete[] str;
    assert(false);
    return;
  }
}

//==============================================================================
VertexShader::~VertexShader()
{
  glDeleteShader(mVertexShaderId);
}

//==============================================================================
GLuint VertexShader::getId() const
{
  return mVertexShaderId;
}

} // namespace gui
} // namespace dart
