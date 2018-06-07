#include "dart/gui/glfw/Shader.hpp"

#include <fstream>
#include "dart/common/Console.hpp"

namespace dart {

//==============================================================================
Shader::Shader(
    const std::string& vertexShaderFilename,
    const std::string& fragmentShaderFilename)
  : mProgramObjectID{0u},
    mVertexShaderFilename(vertexShaderFilename),
    mFragmentShaderFilename(fragmentShaderFilename)
{
  // Set the shader filenames
  mVertexShaderFilename = vertexShaderFilename;
  mFragmentShaderFilename = fragmentShaderFilename;

  // Check that the needed OpenGL extensions are available
  /*bool isExtensionOK = checkOpenGLExtensions();
  if (!isExtensionOK) {
     cerr << "Error : Impossible to use GLSL vertex or fragment shaders on this
  platform" << endl;
     assert(false);
     return false;
  }*/

  assert(!vertexShaderFilename.empty() && !fragmentShaderFilename.empty());

  // ------------------- Load the vertex shader ------------------- //
  GLuint vertexShaderID;
  std::ifstream fileVertexShader;
  fileVertexShader.open(vertexShaderFilename.c_str(), std::ios::binary);

  if (!fileVertexShader.is_open())
  {
    dterr << "Error : Impossible to open the vertex shader file "
          << vertexShaderFilename;
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
  vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
  assert(vertexShaderID != 0);
  glShaderSource(vertexShaderID, 1, (const char**)(&bufferVertexShader), NULL);
  glCompileShader(vertexShaderID);
  delete[] bufferVertexShader;

  // Get the compilation information
  int compiled;
  glGetShaderiv(vertexShaderID, GL_COMPILE_STATUS, &compiled);

  // If the compilation failed
  if (compiled == 0)
  {
    // Get the log of the compilation
    int lengthLog;
    glGetShaderiv(vertexShaderID, GL_INFO_LOG_LENGTH, &lengthLog);
    char* str = new char[lengthLog];
    glGetShaderInfoLog(vertexShaderID, lengthLog, NULL, str);

    // Display the log of the compilation
    dterr << "Vertex Shader Error (in " << vertexShaderFilename
          << ") : " << str;
    delete[] str;
    assert(false);
    return;
  }

  // ------------------- Load the fragment shader ------------------- //
  GLuint fragmentShaderID;
  std::ifstream fileFragmentShader;
  fileFragmentShader.open(fragmentShaderFilename.c_str(), std::ios::binary);

  if (!fileFragmentShader.is_open())
  {
    dterr << "Error : Impossible to open the fragment shader file "
          << fragmentShaderFilename;
    assert(false);
    return;
  }

  // Get the size of the file
  fileFragmentShader.seekg(0, std::ios::end);
  fileSize = (uint)(fileFragmentShader.tellg());
  assert(fileSize != 0);

  // Read the file
  fileFragmentShader.seekg(std::ios::beg);
  char* bufferFragmentShader = new char[fileSize + 1];
  fileFragmentShader.read(bufferFragmentShader, fileSize);
  fileFragmentShader.close();
  bufferFragmentShader[fileSize] = '\0';

  // Create the OpenGL fragment shader and compile it
  fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
  assert(fragmentShaderID != 0);
  glShaderSource(
      fragmentShaderID, 1, (const char**)(&bufferFragmentShader), NULL);
  glCompileShader(fragmentShaderID);
  delete[] bufferFragmentShader;

  // Get the compilation information
  glGetShaderiv(fragmentShaderID, GL_COMPILE_STATUS, &compiled);

  // If the compilation failed
  if (compiled == 0)
  {

    // Get the log of the compilation
    int lengthLog;
    glGetShaderiv(fragmentShaderID, GL_INFO_LOG_LENGTH, &lengthLog);
    char* str = new char[lengthLog];
    glGetShaderInfoLog(fragmentShaderID, lengthLog, NULL, str);

    // Display the log of the compilation
    dterr << "Fragment Shader Error (in " << fragmentShaderFilename
          << ") : " << str;
    delete[] str;
    assert(false);
    return;
  }

  // Create the shader program and attach the shaders
  mProgramObjectID = glCreateProgram();
  assert(mProgramObjectID);
  glAttachShader(mProgramObjectID, vertexShaderID);
  glAttachShader(mProgramObjectID, fragmentShaderID);
  glDeleteShader(vertexShaderID);
  glDeleteShader(fragmentShaderID);

  // Try to link the program
  glLinkProgram(mProgramObjectID);
  int linked;
  glGetProgramiv(mProgramObjectID, GL_LINK_STATUS, &linked);
  if (!linked)
  {
    int logLength;
    glGetProgramiv(mProgramObjectID, GL_INFO_LOG_LENGTH, &logLength);
    char* strLog = new char[logLength];
    glGetProgramInfoLog(mProgramObjectID, logLength, NULL, strLog);
    dterr << "Linker Error in vertex shader " << vertexShaderFilename
          << " or in fragment shader " << fragmentShaderFilename << " : "
          << strLog;
    delete[] strLog;
    assert(false);
  }
}

//==============================================================================
void Shader::bind()
{
  assert(mProgramObjectID != 0);
  glUseProgram(mProgramObjectID);
}

//==============================================================================
void Shader::unbind()
{
  assert(mProgramObjectID != 0);
  glUseProgram(0);
}

//==============================================================================
GLint Shader::getUniformLocation(
    const std::string& variableName, bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);
  GLint location = glGetUniformLocation(mProgramObjectID, variableName.c_str());
  if (location == -1 && errorIfMissing)
  {
    dterr << "Error in vertex shader " << mVertexShaderFilename
          << " or in fragment shader" << mFragmentShaderFilename
          << " : No Uniform variable : " << variableName;
    throw std::logic_error("Error in Shader");
  }

  return location;
}

//==============================================================================
GLint Shader::getAttribLocation(
    const std::string& variableName, bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);
  GLint location = glGetAttribLocation(mProgramObjectID, variableName.c_str());
  if (location == -1 && errorIfMissing)
  {
    dterr << "Error in vertex shader " << mVertexShaderFilename
          << " or in fragment shader" << mFragmentShaderFilename
          << " : No Uniform variable : " << variableName;
    throw std::logic_error("Error in Shader");
  }

  return location;
}

//==============================================================================
void Shader::setFloatUniform(
    const std::string& variableName, float value, bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);
  GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
  {
    glUniform1f(location, value);
  }
}

//==============================================================================
void Shader::setIntUniform(
    const std::string& variableName, int value, bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);
  GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
  {
    glUniform1i(location, value);
  }
}

//==============================================================================
void Shader::setVector2Uniform(
    const std::string& variableName,
    const Eigen::Vector2f& v,
    bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);
  GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform2fv(location, 1, v.data());
}

//==============================================================================
void Shader::setVector3Uniform(
    const std::string& variableName,
    const Eigen::Vector3f& v,
    bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);

  GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform3fv(location, 1, v.data());
}

//==============================================================================
void Shader::setVector4Uniform(
    const std::string& variableName,
    const Eigen::Vector4f& v,
    bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);

  GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform4fv(location, 1, v.data());
}

//==============================================================================
void Shader::setMatrix3x3Uniform(
    const std::string& variableName,
    const float* matrix,
    bool transpose,
    bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);
  GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniformMatrix3fv(location, 1, transpose, matrix);
}

//==============================================================================
void Shader::setMatrix3x3Uniform(
    const std::string& variableName,
    const Eigen::Matrix3f& matrix,
    bool errorIfMissing) const
{
  setMatrix3x3Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
}

//==============================================================================
void Shader::setMatrix4x4Uniform(
    const std::string& variableName,
    const float* matrix,
    bool transpose,
    bool errorIfMissing) const
{
  assert(mProgramObjectID != 0);
  GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniformMatrix4fv(location, 1, transpose, matrix);
}

//==============================================================================
void Shader::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Matrix4f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
void Shader::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Affine3f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
void Shader::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Isometry3f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
bool Shader::checkOpenGLExtensions()
{
  // Check that GLSL vertex and fragment shaders are available on the platform
  // return (GLEW_VERSION_2_0 || (GLEW_ARB_vertex_shader &&
  // GLEW_ARB_fragment_shader));
  return true;
}

} // namespace dart
