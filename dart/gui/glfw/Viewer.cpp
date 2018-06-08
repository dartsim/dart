/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/glfw/Viewer.hpp"

#include <thread>

#include <GLFW/glfw3.h>

#include "dart/common/Console.hpp"
#include "dart/gui/glfw/LoadGlfw.hpp"

namespace dart {
namespace gui {
namespace glfw {

//==============================================================================
bool Viewer::mMainloopActive = false;
std::unordered_map<GLFWwindow*, Viewer*> Viewer::mViewerMap;
const float Viewer::SCROLL_SENSITIVITY = 0.08f;

//==============================================================================
Viewer::Viewer(const std::string& title, int width, int height, bool show)
  : mIsVisible(true), mClearColor(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0))
{
  if (mViewerMap.empty())
    startupGlfw();

  glfwWindowHint(GLFW_VISIBLE, show);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

  mGlfwWindow
      = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
  if (!mGlfwWindow)
  {
    dterr << "[Window::initWindow] Failed to create a GLFW window";
    exit(EXIT_FAILURE);
  }

  auto ok = mViewerMap.insert(std::make_pair(mGlfwWindow, this)).second;
  if (!ok)
  {
    dterr << "[Window::initWindow] Attempting to create the same GLFW window";
    exit(EXIT_FAILURE);
  }

  setCallbacks();
}

//==============================================================================
Viewer::~Viewer()
{
  assert(mGlfwWindow);

  glfwDestroyWindow(mGlfwWindow);

  mViewerMap.erase(mGlfwWindow);
  if (mViewerMap.empty())
    shutdownGlfw();
}

//==============================================================================
Viewer* Viewer::findViewer(GLFWwindow* window)
{
  auto result = mViewerMap.find(window);

  if (result == mViewerMap.end())
  {
    dterr << "[Window::current] Failed to find an Window associated GLFWwindow."
             " Exiting.\n";
    exit(EXIT_FAILURE);
  }

  return result->second;
}

//==============================================================================
void Viewer::setScenePosition(
    const Eigen::Vector3f& position, float sceneRadius)
{
  // Set the position and radius of the scene
  mCenterScene = position;
  mCamera.setSceneRadius(sceneRadius);

  // Reset the camera position and zoom in order to view all the scene
  resetCameraToViewAll();
}

//==============================================================================
void Viewer::resetCameraToViewAll()
{
  // Move the camera to the origin of the scene
  mCamera.translateWorld(-mCamera.getTranslation());

  // Move the camera to the center of the scene
  mCamera.translateWorld(mCenterScene);

  // Set the zoom of the camera so that the scene center is
  // in negative view direction of the camera
  mCamera.setZoom(1.0);
}

//==============================================================================
bool Viewer::mapMouseCoordinatesToSphere(
    double xMouse, double yMouse, Eigen::Vector3f& spherePoint) const
{
  if ((xMouse >= 0) && (xMouse <= mWindowWidth) && (yMouse >= 0)
      && (yMouse <= mWindowHeight))
  {
    float x = float(xMouse - 0.5f * mWindowWidth) / float(mWindowWidth);
    float y = float(0.5f * mWindowHeight - yMouse) / float(mWindowHeight);
    float sinx = std::sin(math::constantsf::pi() * x * 0.5f);
    float siny = std::sin(math::constantsf::pi() * y * 0.5f);
    float sinx2siny2 = sinx * sinx + siny * siny;

    // Compute the point on the sphere
    spherePoint[0] = sinx;
    spherePoint[1] = siny;
    spherePoint[2] = (sinx2siny2 < 1.0) ? std::sqrt(1.0f - sinx2siny2) : 0.0f;

    return true;
  }

  return false;
}

//==============================================================================
void Viewer::zoom(float zoomDiff)
{
  // Zoom the camera
  mCamera.setZoom(zoomDiff);
}

//==============================================================================
void Viewer::translate(int xMouse, int yMouse)
{
  const auto dx = static_cast<float>(xMouse - mLastMouseX);
  const auto dy = static_cast<float>(yMouse - mLastMouseY);

  // Translate the camera
  mCamera.translateCamera(
      -dx / float(mCamera.getWidth()),
      -dy / float(mCamera.getHeight()),
      mCenterScene);
}

//==============================================================================
void Viewer::rotate(int xMouse, int yMouse)
{
  if (mIsLastPointOnSphereValid)
  {
    Eigen::Vector3f newPoint3D;
    bool isNewPointOK = mapMouseCoordinatesToSphere(xMouse, yMouse, newPoint3D);

    if (isNewPointOK)
    {
      Eigen::Vector3f axis = mLastPointOnSphere.cross(newPoint3D);
      float cosAngle = mLastPointOnSphere.dot(newPoint3D);

      float epsilon = std::numeric_limits<float>::epsilon();
      if (std::fabs(cosAngle) < 1.0f && axis.norm() > epsilon)
      {
        axis.normalize();
        float angle = 2.0f * acos(cosAngle);

        // Rotate the camera around the center of the scene
        mCamera.rotateAroundLocalPoint(axis, -angle, mCenterScene);
      }
    }
  }
}

//==============================================================================
void Viewer::runAllViewers(std::size_t refresh)
{
  if (mMainloopActive)
    throw std::runtime_error(
        "[Viewer::runMainLoop] Attempting to run the main "
        "loop twice. Ignoring this action.\n");

  mMainloopActive = true;

  std::thread refreshThread;
  if (refresh > 0)
  {
    refreshThread = std::thread([refresh]() {
      std::chrono::milliseconds time(refresh);
      while (mMainloopActive)
      {
        std::this_thread::sleep_for(time);
        glfwPostEmptyEvent();
      }
    });
  }

  try
  {
    while (mMainloopActive)
    {
      int numScreens = 0;

      for (auto windowPair : mViewerMap)
      {
        auto& window = windowPair.second;

        if (!window->isVisible())
        {
          continue;
        }
        else if (glfwWindowShouldClose(window->mGlfwWindow))
        {
          window->hide();
          continue;
        }

        window->render();
        numScreens++;
      }

      if (numScreens == 0)
      {
        // Give up if there was nothing to render
        mMainloopActive = false;
        break;
      }

      // Wait for mouse/keyboard or empty refresh events
      glfwWaitEvents();
    }

    // Process events once more
    glfwPollEvents();
  }
  catch (const std::exception& e)
  {
    dterr << "[Viewer::runAllViewers] Caught exception in main loop: "
          << e.what();
    abort();
  }

  if (refresh > 0)
    refreshThread.join();
}

//==============================================================================
void Viewer::setTitle(const std::string& title)
{
  glfwSetWindowTitle(mGlfwWindow, title.c_str());
}

//==============================================================================
void Viewer::setSize(const Eigen::Vector2i& size)
{
  setSize(size[0], size[1]);
}

//==============================================================================
void Viewer::setSize(int width, int height)
{
  glfwSetWindowSize(mGlfwWindow, width, height);
}

//==============================================================================
Eigen::Vector2i Viewer::getSize() const
{
  Eigen::Vector2i size;

  glfwGetWindowSize(mGlfwWindow, &size[0], &size[1]);

  return size;
}

//==============================================================================
void Viewer::setVisible(bool visible)
{
  if (mIsVisible == visible)
    return;

  mIsVisible = visible;

  if (mIsVisible)
    glfwShowWindow(mGlfwWindow);
  else
    glfwHideWindow(mGlfwWindow);
}

//==============================================================================
void Viewer::show()
{
  setVisible(true);
}

//==============================================================================
void Viewer::hide()
{
  setVisible(false);
}

//==============================================================================
void Viewer::setScene(std::unique_ptr<Scene>&& scene)
{
  if (mScene == scene)
    return;

  if (nullptr != mScene)
    mScene->notifyMainWindowChanged(nullptr);

  mScene = std::move(scene);
  mScene->notifyMainWindowChanged(mGlfwWindow);

  windowSizeCallback(0, 0);
}

//==============================================================================
Scene* Viewer::getScene() const
{
  return mScene.get();
}

//==============================================================================
Scene* Viewer::releaseScene()
{
  if (nullptr != mScene)
    mScene->notifyMainWindowChanged(nullptr);

  return mScene.release();
}

//==============================================================================
void Viewer::transferSceneTo(Viewer* other)
{
  if (nullptr == other)
    releaseScene();

  if (this == other)
    return;

  other->setScene(std::move(mScene));
  mScene.reset();
}

//==============================================================================
bool Viewer::isVisible() const
{
  return mIsVisible;
}

//==============================================================================
void Viewer::windowSizeCallback(int width, int height)
{
  // Get the framebuffer dimension
  //  int width, height;
  //  glfwGetFramebufferSize(mGlfwWindow, &width, &height);

  // Resize the camera viewport
  mCamera.setDimensions(width, height);

  // Update the window size of the scene
  int windowWidth, windowHeight;
  glfwGetWindowSize(mGlfwWindow, &windowWidth, &windowHeight);

  mWindowWidth = width;
  mWindowHeight = height;
}

//==============================================================================
void Viewer::cursorPosCallback(double xMouse, double yMouse)
{
  const int leftButtonState
      = glfwGetMouseButton(mGlfwWindow, GLFW_MOUSE_BUTTON_LEFT);
  const int rightButtonState
      = glfwGetMouseButton(mGlfwWindow, GLFW_MOUSE_BUTTON_RIGHT);
  const int middleButtonState
      = glfwGetMouseButton(mGlfwWindow, GLFW_MOUSE_BUTTON_MIDDLE);
  const int altKeyState = glfwGetKey(mGlfwWindow, GLFW_KEY_LEFT_ALT);

  // Zoom
  if (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS)
  {
    const float dy = static_cast<float>(yMouse - mLastMouseY);
    const float h = static_cast<float>(mWindowHeight);

    // Zoom the camera
    zoom(-dy / h);
  }
  // Translation
  else if (
      middleButtonState == GLFW_PRESS || rightButtonState == GLFW_PRESS
      || (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS))
  {
    translate(xMouse, yMouse);
  }
  // Rotation
  else if (leftButtonState == GLFW_PRESS)
  {
    rotate(xMouse, yMouse);
  }

  // Remember the mouse position
  mLastMouseX = xMouse;
  mLastMouseY = yMouse;
  mIsLastPointOnSphereValid
      = mapMouseCoordinatesToSphere(xMouse, yMouse, mLastPointOnSphere);
}

//==============================================================================
void Viewer::mouseButtonCallback(int /*button*/, int action, int /*mods*/)
{
  // Get the mouse cursor position
  double x, y;
  glfwGetCursorPos(mGlfwWindow, &x, &y);

  // If the mouse button is pressed
  if (action == GLFW_PRESS)
  {
    mLastMouseX = x;
    mLastMouseY = y;
    mIsLastPointOnSphereValid
        = mapMouseCoordinatesToSphere(x, y, mLastPointOnSphere);
  }
  else
  {
    // If the mouse button is released
    mIsLastPointOnSphereValid = false;
  }
}

//==============================================================================
void Viewer::keyboardCallback(
    int key, int /*scancode*/, int action, int /*mods*/)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    glfwSetWindowShouldClose(mGlfwWindow, true);
}

//==============================================================================
void Viewer::charCallback(unsigned int /*codepoint*/)
{
  // Do nothing
}

//==============================================================================
void Viewer::dropCallback(int /*count*/, const char** /*filenames*/)
{
  // Do nothing
}

//==============================================================================
void Viewer::scrollCallback(double /*xoffset*/, double yoffset)
{
  zoom(yoffset * SCROLL_SENSITIVITY);
}

//==============================================================================
void Viewer::render()
{
  glfwMakeContextCurrent(mGlfwWindow);

  // Rendering
  glClearColor(
      mClearColor.x(), mClearColor.y(), mClearColor.z(), mClearColor.w());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  renderScene();
  // renderGui();

  glfwSwapBuffers(mGlfwWindow);
}

//==============================================================================
void Viewer::renderScene()
{
  if (!mScene)
  {
    // TODO: render empty scene
    return;
  }

  const Eigen::Vector4f& diffCol = mLight0.getDiffuseColor();

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  const Eigen::Isometry3f worldToLightCameraMatrix
      = mLight0.getTransform().inverse();

  glCullFace(GL_BACK);

  // Get the world-space to camera-space matrix
  const Eigen::Isometry3f worldToCameraMatrix
      = mCamera.getTransform().inverse();

  mPhongProgram->bind();

  // Set the variables of the shader
  mPhongProgram->setMatrix4x4Uniform(
      std::string("projectionMatrix"), mCamera.getProjectionMatrix());
  mPhongProgram->setMatrix4x4Uniform(
      std::string("worldToLight0CameraMatrix"), worldToLightCameraMatrix);
  mPhongProgram->setVector3Uniform(
      std::string("light0PosCameraSpace"),
      worldToCameraMatrix * mLight0.getTranslation());
  mPhongProgram->setVector3Uniform(
      std::string("lightAmbientColor"), Eigen::Vector3f(0.4f, 0.4f, 0.4f));
  mPhongProgram->setVector3Uniform(
      std::string("light0DiffuseColor"), diffCol.head<3>());

  int display_w, display_h;
  glfwGetFramebufferSize(mGlfwWindow, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  mViewportX = 0;
  mViewportY = 0;
  mViewportWidth = display_w;
  mViewportHeight = display_h;
  // TODO: ugly

  // Set the viewport to render the scene
  glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);

  // Enabling color write (previously disabled for light POV z-buffer rendering)
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

  // Clear previous frame values
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Render the objects of the scene
  renderSinglePass(*mPhongProgram, worldToCameraMatrix);

  mPhongProgram->unbind();
}

//==============================================================================
void Viewer::renderSinglePass(
    Program& program, const Eigen::Isometry3f& worldToCameraMatrix)
{
  program.bind();

  if (mScene)
    mScene->renderSinglePass(program, worldToCameraMatrix);

  program.unbind();
}

//==============================================================================
void Viewer::startupGlfw()
{
  assert(mViewerMap.empty());

  dtmsg << "starting GLFW " << glfwGetVersionString();

  // Setup window
  glfwSetErrorCallback([](int error, const char* description) {
    dterr << "GLFW Error occured, Error ID: " << error
          << ", Description: " << description;
  });

  if (!glfwInit())
    throw std::runtime_error("Could not initialize GLFW.");

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#if __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  gl3wInit();
}

//==============================================================================
void Viewer::shutdownGlfw()
{
  assert(mViewerMap.empty());

  glfwTerminate();
}

//==============================================================================
void Viewer::setCallbacks()
{
  assert(mGlfwWindow);

  glfwSetWindowSizeCallback(
      mGlfwWindow, [](GLFWwindow* window, int width, int height) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->windowSizeCallback(width, height);
      });

  glfwSetCursorPosCallback(
      mGlfwWindow, [](GLFWwindow* window, double xpos, double ypos) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->cursorPosCallback(xpos, ypos);
      });

  glfwSetMouseButtonCallback(
      mGlfwWindow, [](GLFWwindow* window, int button, int action, int mods) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->mouseButtonCallback(button, action, mods);
      });

  glfwSetKeyCallback(
      mGlfwWindow,
      [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->keyboardCallback(key, scancode, action, mods);
      });

  glfwSetCharCallback(
      mGlfwWindow, [](GLFWwindow* window, unsigned int codepoint) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->charCallback(codepoint);
      });

  glfwSetDropCallback(
      mGlfwWindow, [](GLFWwindow* window, int count, const char** filenames) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->dropCallback(count, filenames);
      });

  glfwSetScrollCallback(
      mGlfwWindow, [](GLFWwindow* window, double xoffset, double yoffset) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->scrollCallback(xoffset, yoffset);
      });

  glfwSetFramebufferSizeCallback(
      mGlfwWindow, [](GLFWwindow* window, int width, int height) {
        auto* viewer = findViewer(window);
        if (viewer)
          viewer->windowSizeCallback(width, height);
      });
}

//==============================================================================
void Viewer::initLights()
{
  static float ambient[] = {0.2, 0.2, 0.2, 1.0};
  static float diffuse[] = {0.6, 0.6, 0.6, 1.0};
  static float front_mat_shininess[] = {60.0};
  static float front_mat_specular[] = {0.2, 0.2, 0.2, 1.0};
  static float front_mat_diffuse[] = {0.5, 0.28, 0.38, 1.0};
  static float lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
  static float lmodel_twoside[] = {GL_FALSE};

  GLfloat position[] = {1.0, 0.0, 0.0, 0.0};
  GLfloat position1[] = {-1.0, 0.0, 0.0, 0.0};

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, position);

  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, position1);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, front_mat_specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, front_mat_diffuse);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glDisable(GL_CULL_FACE);
  glEnable(GL_NORMALIZE);
}

} // namespace glfw
} // namespace gui
} // namespace dart
