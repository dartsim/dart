#include "dart/gui/Program.hpp"

#include <fstream>
#include <vector>
#include "dart/common/Console.hpp"

namespace dart {
namespace gui {

//==============================================================================
Program::Program(
    const VertexShader& vertexShader, const FragmentShader& fragmentShader)
  : mProgramObjectId{0u}
{
  // Create the shader program and attach the shaders
  mProgramObjectId = glCreateProgram();
  assert(mProgramObjectId);
  glAttachShader(mProgramObjectId, vertexShader.getId());
  glAttachShader(mProgramObjectId, fragmentShader.getId());

  // Try to link the program
  glLinkProgram(mProgramObjectId);
  int linked;
  glGetProgramiv(mProgramObjectId, GL_LINK_STATUS, &linked);
  if (!linked)
  {
    int logLength;
    glGetProgramiv(mProgramObjectId, GL_INFO_LOG_LENGTH, &logLength);
    std::vector<char> strLog(logLength);
    glGetProgramInfoLog(mProgramObjectId, logLength, NULL, strLog.data());
    //    dterr << "Linker Error in vertex shader " << vertexShaderFilepath
    //          << " or in fragment shader " << fragmentShaderFilepath << " : "
    //          << strLog;
    dterr << "Linker Error in vertex shader or in fragment shader: "
          << strLog.data();
    assert(false);
  }
}

//==============================================================================
void Program::bind()
{
  assert(mProgramObjectId != 0);
  glUseProgram(mProgramObjectId);
}

//==============================================================================
void Program::unbind()
{
  assert(mProgramObjectId != 0);
  glUseProgram(0);
}

//==============================================================================
GLint Program::getUniformLocation(
    const std::string& variableName, bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);
  const GLint location
      = glGetUniformLocation(mProgramObjectId, variableName.c_str());
  if (location == -1 && errorIfMissing)
  {
    //    dterr << "Error in vertex shader " << mVertexShaderFilepath
    //          << " or in fragment shader" << mFragmentShaderFilepath
    //          << " : No Uniform variable : " << variableName;
    dterr << "Error in vertex shader or in fragment shader: "
          << "No Uniform variable : " << variableName;
    throw std::logic_error("Error in Shader");
  }

  return location;
}

//==============================================================================
GLint Program::getAttribLocation(
    const std::string& variableName, bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);
  const GLint location
      = glGetAttribLocation(mProgramObjectId, variableName.c_str());
  if (location == -1 && errorIfMissing)
  {
    //    dterr << "Error in vertex shader " << mVertexShaderFilepath
    //          << " or in fragment shader" << mFragmentShaderFilepath
    //          << " : No Uniform variable : " << variableName;
    dterr << "Error in vertex shader or in fragment shader: "
          << "No Uniform variable : " << variableName;
    throw std::logic_error("Error in Shader");
  }

  return location;
}

//==============================================================================
void Program::setFloatUniform(
    const std::string& variableName, float value, bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
  {
    glUniform1f(location, value);
  }
}

//==============================================================================
void Program::setIntUniform(
    const std::string& variableName, int value, bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
  {
    glUniform1i(location, value);
  }
}

//==============================================================================
void Program::setVector2Uniform(
    const std::string& variableName,
    const Eigen::Vector2f& v,
    bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform2fv(location, 1, v.data());
}

//==============================================================================
void Program::setVector3Uniform(
    const std::string& variableName,
    const Eigen::Vector3f& v,
    bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);

  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform3fv(location, 1, v.data());
}

//==============================================================================
void Program::setVector4Uniform(
    const std::string& variableName,
    const Eigen::Vector4f& v,
    bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);

  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniform4fv(location, 1, v.data());
}

//==============================================================================
void Program::setMatrix3x3Uniform(
    const std::string& variableName,
    const float* matrix,
    bool transpose,
    bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniformMatrix3fv(location, 1, transpose, matrix);
}

//==============================================================================
void Program::setMatrix3x3Uniform(
    const std::string& variableName,
    const Eigen::Matrix3f& matrix,
    bool errorIfMissing) const
{
  setMatrix3x3Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const float* matrix,
    bool transpose,
    bool errorIfMissing) const
{
  assert(mProgramObjectId != 0);
  const GLint location = getUniformLocation(variableName, errorIfMissing);
  if (location != -1)
    glUniformMatrix4fv(location, 1, transpose, matrix);
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Matrix4f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Affine3f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

//==============================================================================
void Program::setMatrix4x4Uniform(
    const std::string& variableName,
    const Eigen::Isometry3f& matrix,
    bool errorIfMissing) const
{
  setMatrix4x4Uniform(variableName, matrix.data(), false, errorIfMissing);
  // TODO: add test if matrix.data() order is right
  // TODO: add transposed version
}

} // namespace gui
} // namespace dart
