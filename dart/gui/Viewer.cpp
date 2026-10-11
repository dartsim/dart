/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/gui/DefaultEventHandler.hpp>
#include <dart/gui/DragAndDrop.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/InteractiveFrame.hpp>
#include <dart/gui/RenderingUnavailableError.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/ViewerAttachment.hpp>
#include <dart/gui/WorldNode.hpp>
#include <dart/gui/detail/FilamentRenderer.hpp>
#include <dart/gui/detail/interaction.hpp>
#include <dart/gui/detail/key_input.hpp>
#include <dart/gui/detail/native_window.hpp>
#include <dart/gui/detail/overlay_adapter.hpp>
#include <dart/gui/detail/render_context.hpp>
#include <dart/gui/detail/renderable.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <dart/common/sub_ptr.hpp>

#include <Eigen/Geometry>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <png.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <cmath>
#include <cstdlib>

namespace dart {
namespace gui {
namespace {
constexpr double viewerNearPlane = 0.01;
constexpr double viewerFarPlane = 1000.0;
unsigned int viewerGlfwUsers = 0;
// glfwPollEvents dispatches every viewer's window callbacks, and GLFW forbids
// destroying or creating windows from those callbacks. Windows released while
// it runs are destroyed after it returns.
bool viewerEventDispatch = false;
std::vector<GLFWwindow*> deferredViewerWindows;
void initViewerWindowSystem()
{
  dart::gui::detail::checkFilamentBackendAvailability(false);
#if defined(_WIN32) || defined(__APPLE__)
  constexpr const char* requirement = "";
#else
  // Filament's Linux GL platform is GLX, so Wayland desktops use XWayland.
  glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
  constexpr const char* requirement
      = " (install libx11-6, libgl1 and libgl1-mesa-dri; use an X11 or "
        "XWayland display)";
#endif
  if (glfwInit() != GLFW_TRUE)
    throw RenderingUnavailableError(
        std::string("Cannot initialize the native window system")
        + requirement);
}
void destroyViewerWindow(GLFWwindow* window)
{
  dart::gui::detail::releaseFilamentNativeWindow(window);
  glfwDestroyWindow(window);
  if (--viewerGlfwUsers == 0)
    glfwTerminate();
}
struct ViewerWorldOwner
{
  Viewer* viewer;
  std::weak_ptr<WorldNode> node;
  bool active;
};
std::map<const simulation::World*, std::vector<ViewerWorldOwner>>
    viewerWorldOwners;
void removeViewerWorldOwners(Viewer* viewer, const WorldNode* node = nullptr)
{
  for (auto it = viewerWorldOwners.begin(); it != viewerWorldOwners.end();) {
    auto& owners = it->second;
    owners.erase(
        std::remove_if(
            owners.begin(),
            owners.end(),
            [&](const auto& owner) {
              const auto existing = owner.node.lock();
              return !existing
                     || (owner.viewer == viewer
                         && (!node || existing.get() == node))
                     || existing->getWorld().get() != it->first;
            }),
        owners.end());
    if (owners.empty())
      it = viewerWorldOwners.erase(it);
    else
      ++it;
  }
}
bool viewerOwnsWorldSteps(
    Viewer* viewer, const std::shared_ptr<WorldNode>& node, bool active = true)
{
  const auto world = node->getWorld();
  if (!world)
    return false;
  auto& owners = viewerWorldOwners[world.get()];
  owners.erase(
      std::remove_if(
          owners.begin(),
          owners.end(),
          [&](const auto& owner) {
            const auto existing = owner.node.lock();
            return !existing || existing->getWorld() != world;
          }),
      owners.end());
  const auto found
      = std::find_if(owners.begin(), owners.end(), [&](const auto& owner) {
          return owner.viewer == viewer && owner.node.lock() == node;
        });
  if (found == owners.end())
    owners.push_back({viewer, node, active});
  else
    found->active = active;
  const auto stepOwner
      = std::find_if(owners.begin(), owners.end(), [](const auto& owner) {
          const auto existing = owner.node.lock();
          return owner.active && !owner.viewer->done() && existing
                 && existing->isSimulating();
        });
  return stepOwner != owners.end() && stepOwner->viewer == viewer
         && stepOwner->node.lock() == node;
}
ImGuiKey viewerImGuiKey(int key)
{
  if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z)
    return static_cast<ImGuiKey>(ImGuiKey_A + key - GLFW_KEY_A);
  if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9)
    return static_cast<ImGuiKey>(ImGuiKey_0 + key - GLFW_KEY_0);
  if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F12)
    return static_cast<ImGuiKey>(ImGuiKey_F1 + key - GLFW_KEY_F1);
  switch (key) {
    case GLFW_KEY_TAB:
      return ImGuiKey_Tab;
    case GLFW_KEY_LEFT:
      return ImGuiKey_LeftArrow;
    case GLFW_KEY_RIGHT:
      return ImGuiKey_RightArrow;
    case GLFW_KEY_UP:
      return ImGuiKey_UpArrow;
    case GLFW_KEY_DOWN:
      return ImGuiKey_DownArrow;
    case GLFW_KEY_PAGE_UP:
      return ImGuiKey_PageUp;
    case GLFW_KEY_PAGE_DOWN:
      return ImGuiKey_PageDown;
    case GLFW_KEY_HOME:
      return ImGuiKey_Home;
    case GLFW_KEY_END:
      return ImGuiKey_End;
    case GLFW_KEY_INSERT:
      return ImGuiKey_Insert;
    case GLFW_KEY_DELETE:
      return ImGuiKey_Delete;
    case GLFW_KEY_BACKSPACE:
      return ImGuiKey_Backspace;
    case GLFW_KEY_SPACE:
      return ImGuiKey_Space;
    case GLFW_KEY_ENTER:
      return ImGuiKey_Enter;
    case GLFW_KEY_ESCAPE:
      return ImGuiKey_Escape;
    case GLFW_KEY_APOSTROPHE:
      return ImGuiKey_Apostrophe;
    case GLFW_KEY_COMMA:
      return ImGuiKey_Comma;
    case GLFW_KEY_MINUS:
      return ImGuiKey_Minus;
    case GLFW_KEY_PERIOD:
      return ImGuiKey_Period;
    case GLFW_KEY_SLASH:
      return ImGuiKey_Slash;
    case GLFW_KEY_SEMICOLON:
      return ImGuiKey_Semicolon;
    case GLFW_KEY_EQUAL:
      return ImGuiKey_Equal;
    case GLFW_KEY_LEFT_BRACKET:
      return ImGuiKey_LeftBracket;
    case GLFW_KEY_BACKSLASH:
      return ImGuiKey_Backslash;
    case GLFW_KEY_RIGHT_BRACKET:
      return ImGuiKey_RightBracket;
    case GLFW_KEY_GRAVE_ACCENT:
      return ImGuiKey_GraveAccent;
    case GLFW_KEY_LEFT_SHIFT:
      return ImGuiKey_LeftShift;
    case GLFW_KEY_RIGHT_SHIFT:
      return ImGuiKey_RightShift;
    case GLFW_KEY_LEFT_CONTROL:
      return ImGuiKey_LeftCtrl;
    case GLFW_KEY_RIGHT_CONTROL:
      return ImGuiKey_RightCtrl;
    case GLFW_KEY_LEFT_ALT:
      return ImGuiKey_LeftAlt;
    case GLFW_KEY_RIGHT_ALT:
      return ImGuiKey_RightAlt;
    case GLFW_KEY_LEFT_SUPER:
      return ImGuiKey_LeftSuper;
    case GLFW_KEY_RIGHT_SUPER:
      return ImGuiKey_RightSuper;
    default:
      return ImGuiKey_None;
  }
}
void writeViewerPng(
    const std::string& filename,
    int width,
    int height,
    const std::vector<std::uint8_t>& pixels)
{
  if (pixels.size() != static_cast<std::size_t>(width) * height * 4)
    throw std::runtime_error("Renderer returned an invalid RGBA image");
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  image.width = width;
  image.height = height;
  image.format = PNG_FORMAT_RGBA;
  if (!png_image_write_to_file(
          &image, filename.c_str(), 0, pixels.data(), 0, nullptr)) {
    const std::string error = image.message;
    png_image_free(&image);
    throw std::runtime_error(
        "Cannot write capture '" + filename + "': " + error);
  }
  png_image_free(&image);
}
struct ViewerImGuiScope
{
  ImGuiContext* previous;
  explicit ViewerImGuiScope(ImGuiContext* current)
    : previous(ImGui::GetCurrentContext())
  {
    ImGui::SetCurrentContext(current);
  }
  ~ViewerImGuiScope()
  {
    ImGui::SetCurrentContext(previous);
  }
};
struct ViewerFrameScope
{
  bool& framing;
  explicit ViewerFrameScope(bool& flag) : framing(flag)
  {
    framing = true;
  }
  ~ViewerFrameScope()
  {
    framing = false;
  }
};
} // namespace

struct Viewer::Impl
{
  Viewer* owner;
  struct Node
  {
    std::shared_ptr<WorldNode> node;
    bool active;
  };
  std::vector<Node> nodes;
  std::vector<std::shared_ptr<ViewerAttachment>> attachments;
  std::vector<std::shared_ptr<ImGuiWidget>> widgets;
  std::vector<std::shared_ptr<GUIEventHandler>> handlers;
  std::unique_ptr<DefaultEventHandler> defaultHandler;
  std::vector<std::shared_ptr<DragAndDrop>> dragAndDrops;
  std::unique_ptr<detail::OverlayRenderer> overlay;
  DebugScene debugScene;
  ImGuiStyle baseStyle;
  bool haveBaseStyle = false;
  std::string instructions;
  std::exception_ptr inputError;
  bool wireframe = false;
  std::map<const dynamics::BodyNode*, dynamics::WeakBodyNodePtr>
      wireframeBodies;
  RenderableExtractor extractor;
  detail::KeySymbolTranslator keys;
  std::vector<std::function<void(Impl&, ImGuiIO&)>> pendingInput;
  std::unique_ptr<detail::FilamentRenderer> renderer;
  GLFWwindow* window = nullptr;
  ImGuiContext* gui = nullptr;
  Eigen::Vector4d background;
  Eigen::Vector3d eye{3.0, 3.0, 2.0}, center{0.0, 0.0, 0.45}, up{0.0, 0.0, 1.0};
  Eigen::Vector3d homeEye = eye, homeCenter = center, homeUp = up;
  Eigen::Vector3d lightingUp = Eigen::Vector3d::UnitZ();
  CameraMode cameraMode = CameraMode::RGBA;
  std::string title = "DART";
  std::string capture;
  std::string recordingDirectory, recordingPrefix;
  std::size_t recordingIndex = 0, recordingDigits = 6;
  bool recording = false, finished = false, maySimulate = true,
       simulating = false;
  bool headlights = true, headless = false, framing = false, closing = false,
       defaultHandlerEnabled = true;
  int width = 1600, height = 1000;
  // OSG's default view used about 29 degrees; keep existing camera homes
  // framed.
  double fovy = 30.0, requestedGuiScale = 0.0, detectedGuiScale = 1.0,
         appliedGuiScale = 1.0;
  double cursorX = 0.0, cursorY = 0.0;
  bool cursorKnown = false;
  std::chrono::steady_clock::time_point previousFrame
      = std::chrono::steady_clock::now();

  explicit Impl(Viewer* viewer, const Eigen::Vector4d& color)
    : owner(viewer), background(color)
  {
  }
  ~Impl()
  {
    releaseGraphics();
  }
  void releaseGraphics()
  {
    overlay.reset();
    haveBaseStyle = false;
    appliedGuiScale = detectedGuiScale = 1.0;
    cursorKnown = false;
    inputError = {};
    keys = {};
    renderer.reset();
    if (gui) {
      ImGui::DestroyContext(gui);
      gui = nullptr;
    }
    pendingInput.clear();
    if (window) {
      if (viewerEventDispatch) {
        glfwSetWindowUserPointer(window, nullptr);
        glfwHideWindow(window);
        deferredViewerWindows.push_back(window);
      } else {
        destroyViewerWindow(window);
      }
      window = nullptr;
    }
  }
  template <class Callback>
  void withGui(Callback callback)
  {
    if (inputError)
      return;
    ViewerImGuiScope scope(gui);
    try {
      callback(ImGui::GetIO());
    } catch (...) {
      inputError = std::current_exception();
    }
  }
  /// Runs window input inside this viewer's own frame. Events GLFW delivers
  /// during another viewer's poll wait, so handlers never run (or destroy
  /// their viewer) inside a foreign event dispatch.
  template <class Callback>
  static void input(GLFWwindow* window, Callback callback)
  {
    auto* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
    if (!self)
      return;
    // ponytail: a viewer that is never framed drops input past this cap;
    // coalesce motion events if a real app needs more.
    constexpr std::size_t maxPendingInput = 1024;
    if (self->framing && !self->inputError)
      self->withGui([&](auto& io) { callback(*self, io); });
    else if (self->pendingInput.size() < maxPendingInput)
      self->pendingInput.emplace_back(std::move(callback));
  }
  static int buttonMask(GLFWwindow* window)
  {
    int mask = 0;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
      mask |= GUIEventAdapter::LEFT_MOUSE_BUTTON;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
      mask |= GUIEventAdapter::RIGHT_MOUSE_BUTTON;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS)
      mask |= GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
    return mask;
  }
  int buttonMask() const
  {
    return buttonMask(window);
  }
  void deliverKey(const std::optional<detail::KeySymbolTranslator::Event>& key)
  {
    if (key)
      owner->handleEvent(GUIEventAdapter(
          key->down ? GUIEventAdapter::KEYDOWN : GUIEventAdapter::KEYUP,
          key->symbol,
          cursorX,
          cursorY,
          buttonMask(),
          modifiers(key->mods)));
  }
  static int modifiers(int mods)
  {
    return ((mods & GLFW_MOD_SHIFT) ? GUIEventAdapter::MODKEY_SHIFT : 0)
           | ((mods & GLFW_MOD_CONTROL) ? GUIEventAdapter::MODKEY_CTRL : 0)
           | ((mods & GLFW_MOD_ALT) ? GUIEventAdapter::MODKEY_ALT : 0)
           | ((mods & GLFW_MOD_SUPER) ? GUIEventAdapter::MODKEY_SUPER : 0);
  }
  int modifiers() const
  {
    return heldModifiers(window);
  }
  /// X11 key events report the modifier state from before the event, so
  /// ImGui reads the state GLFW tracks after it, as its own backend does.
  static void addImGuiModifiers(ImGuiIO& io, int held)
  {
    io.AddKeyEvent(ImGuiMod_Ctrl, (held & GUIEventAdapter::MODKEY_CTRL) != 0);
    io.AddKeyEvent(ImGuiMod_Shift, (held & GUIEventAdapter::MODKEY_SHIFT) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (held & GUIEventAdapter::MODKEY_ALT) != 0);
    io.AddKeyEvent(ImGuiMod_Super, (held & GUIEventAdapter::MODKEY_SUPER) != 0);
  }
  static int heldModifiers(GLFWwindow* window)
  {
    int mask = 0;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
        || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS)
      mask |= GUIEventAdapter::MODKEY_SHIFT;
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
        || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS)
      mask |= GUIEventAdapter::MODKEY_CTRL;
    if (glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS
        || glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS)
      mask |= GUIEventAdapter::MODKEY_ALT;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SUPER) == GLFW_PRESS
        || glfwGetKey(window, GLFW_KEY_RIGHT_SUPER) == GLFW_PRESS)
      mask |= GUIEventAdapter::MODKEY_SUPER;
    return mask;
  }
  /// Scales the eye-center distance without crossing the clip planes, so the
  /// camera never collapses onto its center.
  void zoom(double factor)
  {
    const Eigen::Vector3d offset = eye - center;
    const double distance = offset.norm();
    const double target = std::clamp(
        distance * factor,
        std::min(distance, viewerNearPlane),
        std::max(distance, viewerFarPlane));
    eye = center + offset * (target / distance);
  }
  bool isDragging() const
  {
    return std::any_of(
        dragAndDrops.begin(), dragAndDrops.end(), [](const auto& drag) {
          return drag->isMoving();
        });
  }
  void readGuiScaleOverride()
  {
    if (requestedGuiScale <= 0)
      if (const char* env = std::getenv("DART_GUI_SCALE"); env && *env) {
        char* end = nullptr;
        const double scale = std::strtod(env, &end);
        if (!end || *end || !std::isfinite(scale) || scale <= 0)
          throw std::invalid_argument(
              "DART_GUI_SCALE must be a finite positive number");
        requestedGuiScale = scale;
      }
  }
  void createGraphics(bool offscreen, int x, int y, int newWidth, int newHeight)
  {
    if (newWidth <= 0 || newHeight <= 0)
      throw std::invalid_argument("Viewport dimensions must be positive");
    if (closing)
      throw std::logic_error(
          "Cannot create graphics while the Viewer is closing");
    if (framing)
      throw std::logic_error(
          "Cannot recreate viewer graphics during a frame callback");
    if (viewerEventDispatch)
      throw std::logic_error(
          "Cannot create viewer graphics from a window event callback");
    readGuiScaleOverride();
    releaseGraphics();
    headless = offscreen;
    width = newWidth;
    height = newHeight;
    if (!headless) {
      if (viewerGlfwUsers == 0)
        initViewerWindowSystem();
      glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
      glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_FALSE);
      window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
      if (!window) {
        if (viewerGlfwUsers == 0)
          glfwTerminate();
        throw RenderingUnavailableError("Cannot create the DART window");
      }
      ++viewerGlfwUsers;
      glfwSetWindowPos(window, x, y);
      glfwSetWindowUserPointer(window, this);
      glfwSetCharCallback(window, [](GLFWwindow* w, unsigned int character) {
        input(w, [=](Impl& self, ImGuiIO& io) {
          io.AddInputCharacter(character);
          self.deliverKey(self.keys.character(character));
        });
      });
      glfwSetKeyCallback(
          window, [](GLFWwindow* w, int key, int, int action, int mods) {
            const int held = heldModifiers(w);
            input(w, [=](Impl& self, ImGuiIO& io) {
              addImGuiModifiers(io, held);
              const auto mapped = viewerImGuiKey(key);
              if (mapped != ImGuiKey_None)
                io.AddKeyEvent(mapped, action != GLFW_RELEASE);
              self.deliverKey(self.keys.flush());
              if (action != GLFW_RELEASE && io.WantCaptureKeyboard)
                return;
              self.deliverKey(self.keys.key(key, action, mods));
            });
          });
      glfwSetCursorPosCallback(window, [](GLFWwindow* w, double px, double py) {
        // Queued events keep the button and modifier state of their arrival.
        const int buttons = buttonMask(w), held = heldModifiers(w);
        input(w, [=](Impl& self, ImGuiIO& io) {
          io.AddMousePosEvent(static_cast<float>(px), static_cast<float>(py));
          const bool captured = io.WantCaptureMouse;
          const bool handled
              = !captured
                && self.owner->handleEvent(GUIEventAdapter(
                    buttons ? GUIEventAdapter::DRAG : GUIEventAdapter::MOVE,
                    0,
                    px,
                    py,
                    buttons,
                    held));
          if (self.cursorKnown && !captured && !handled && !self.isDragging()
              && !(held & GUIEventAdapter::MODKEY_CTRL)) {
            const double dx = px - self.cursorX, dy = py - self.cursorY;
            const auto direction = (self.center - self.eye).normalized().eval();
            const auto right = direction.cross(self.up).normalized().eval();
            if (buttons & GUIEventAdapter::LEFT_MOUSE_BUTTON) {
              Eigen::Vector3d offset = Eigen::AngleAxisd(-dx * 0.006, self.up)
                                       * (self.eye - self.center);
              const Eigen::Vector3d pitched
                  = Eigen::AngleAxisd(-dy * 0.006, right) * offset;
              if (std::abs(pitched.normalized().dot(self.up)) < 0.995)
                offset = pitched;
              self.eye = self.center + offset;
            } else if (buttons & GUIEventAdapter::MIDDLE_MOUSE_BUTTON) {
              int windowWidth = 0, windowHeight = 0;
              glfwGetWindowSize(self.window, &windowWidth, &windowHeight);
              const double scale = (self.eye - self.center).norm() * 2.0
                                   * std::tan(self.fovy * 0.008726646259971648)
                                   / std::max(1, windowHeight);
              const Eigen::Vector3d motion
                  = (-dx * right + dy * right.cross(direction)) * scale;
              self.eye += motion;
              self.center += motion;
            } else if (buttons & GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
              self.zoom(std::exp(dy * 0.006));
            }
          }
          self.cursorX = px;
          self.cursorY = py;
          self.cursorKnown = true;
        });
      });
      glfwSetMouseButtonCallback(
          window, [](GLFWwindow* w, int button, int action, int mods) {
            const int buttons = buttonMask(w), held = heldModifiers(w);
            input(w, [=](Impl& self, ImGuiIO& io) {
              addImGuiModifiers(io, held);
              const int mapped = button == GLFW_MOUSE_BUTTON_RIGHT    ? 1
                                 : button == GLFW_MOUSE_BUTTON_MIDDLE ? 2
                                                                      : button;
              if (mapped >= 0 && mapped < 5)
                io.AddMouseButtonEvent(mapped, action != GLFW_RELEASE);
              if (!io.WantCaptureMouse || action == GLFW_RELEASE)
                self.owner->handleEvent(GUIEventAdapter(
                    action == GLFW_RELEASE ? GUIEventAdapter::RELEASE
                                           : GUIEventAdapter::PUSH,
                    0,
                    self.cursorX,
                    self.cursorY,
                    buttons,
                    modifiers(mods)));
            });
          });
      glfwSetScrollCallback(window, [](GLFWwindow* w, double sx, double sy) {
        const int buttons = buttonMask(w), held = heldModifiers(w);
        input(w, [=](Impl& self, ImGuiIO& io) {
          io.AddMouseWheelEvent(static_cast<float>(sx), static_cast<float>(sy));
          if (!io.WantCaptureMouse)
            self.owner->handleEvent(GUIEventAdapter(
                GUIEventAdapter::SCROLL,
                0,
                self.cursorX,
                self.cursorY,
                buttons,
                held,
                sx,
                sy));
        });
      });
      glfwSetWindowFocusCallback(window, [](GLFWwindow* w, int focused) {
        input(w, [=](Impl& self, ImGuiIO& io) {
          io.AddFocusEvent(focused != 0);
          if (!focused)
            self.owner->handleEvent(GUIEventAdapter(
                GUIEventAdapter::RELEASE, 0, self.cursorX, self.cursorY));
        });
      });
      glfwSetWindowCloseCallback(window, [](GLFWwindow* w) {
        input(w, [=](Impl& self, ImGuiIO&) {
          self.owner->handleEvent(
              GUIEventAdapter(GUIEventAdapter::CLOSE_WINDOW));
        });
      });
      glfwGetFramebufferSize(window, &width, &height);
      // Scenes installed before the first frame read getGuiScale().
      float scaleX = 1.0f, scaleY = 1.0f;
      glfwGetWindowContentScale(window, &scaleX, &scaleY);
      detectedGuiScale = scaleY > 0 ? scaleY : 1.0;
    }
    try {
      ViewerImGuiScope restore(nullptr);
      gui = ImGui::CreateContext();
      ImGui::SetCurrentContext(gui);
      ImGui::StyleColorsDark();
      auto& io = ImGui::GetIO();
      if (headless)
        io.IniFilename = nullptr;
      io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
#if IMGUI_VERSION_NUM >= 19200
      io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
#endif
#ifdef IMGUI_HAS_DOCK
      io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif
      renderer = std::make_unique<detail::FilamentRenderer>(
          width,
          height,
          headless,
          window ? detail::getFilamentNativeWindow(window) : nullptr);
      renderer->setBackgroundColor(background);
      renderer->renderImGui(nullptr);
      overlay = std::make_unique<detail::OverlayRenderer>();
    } catch (...) {
      releaseGraphics();
      throw;
    }
  }
  void createDefaultWindow()
  {
    readGuiScaleOverride();
    if (viewerGlfwUsers == 0)
      initViewerWindowSystem();
    int x = 0, y = 0, workWidth = 1600, workHeight = 1000;
    float scaleX = 1.0f, scaleY = 1.0f;
    if (auto* monitor = glfwGetPrimaryMonitor()) {
      glfwGetMonitorWorkarea(monitor, &x, &y, &workWidth, &workHeight);
      // X11's desktop work area can have no intersection with this monitor.
      if (workWidth <= 0 || workHeight <= 0) {
        glfwGetMonitorPos(monitor, &x, &y);
        workWidth = 1600;
        workHeight = 1000;
        if (const auto* mode = glfwGetVideoMode(monitor);
            mode && mode->width > 0 && mode->height > 0) {
          workWidth = mode->width;
          workHeight = mode->height;
        }
      }
      glfwGetMonitorContentScale(monitor, &scaleX, &scaleY);
    }
#ifdef __APPLE__
    constexpr bool logicalCoordinates = true;
#else
    constexpr bool logicalCoordinates = false;
#endif
    const auto fitted = detail::fitDefaultWindow(
        {x, y, workWidth, workHeight},
        scaleY,
        requestedGuiScale,
        logicalCoordinates);
    createGraphics(false, fitted.x, fitted.y, fitted.width, fitted.height);
  }
  void updateGui()
  {
    int logicalWidth = width, logicalHeight = height;
    float scaleX = 1.0f, scaleY = 1.0f;
    if (window) {
      glfwGetWindowSize(window, &logicalWidth, &logicalHeight);
      glfwGetWindowContentScale(window, &scaleX, &scaleY);
      detectedGuiScale = scaleY > 0 ? scaleY : 1.0;
    }
    auto& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(
        static_cast<float>(std::max(1, logicalWidth)),
        static_cast<float>(std::max(1, logicalHeight)));
    io.DisplayFramebufferScale = ImVec2(
        static_cast<float>(width) / std::max(1, logicalWidth),
        static_cast<float>(height) / std::max(1, logicalHeight));
    const double scale
        = (requestedGuiScale > 0 ? requestedGuiScale : detectedGuiScale)
          / io.DisplayFramebufferScale.y;
    if (!haveBaseStyle) {
      baseStyle = ImGui::GetStyle();
      haveBaseStyle = true;
    }
    if (scale != appliedGuiScale) {
      ImGui::GetStyle() = baseStyle;
      ImGui::GetStyle().ScaleAllSizes(static_cast<float>(scale));
#if IMGUI_VERSION_NUM >= 19200
      ImGui::GetStyle().FontScaleMain = static_cast<float>(scale);
#else
      io.FontGlobalScale = static_cast<float>(scale);
#endif
      appliedGuiScale = scale;
    }
    const auto now = std::chrono::steady_clock::now();
    io.DeltaTime = static_cast<float>(std::clamp(
        std::chrono::duration<double>(now - previousFrame).count(), 1e-6, 1.0));
    previousFrame = now;
  }
};

std::string toString(CameraMode mode)
{
  return mode == CameraMode::RGBA ? "RGBA" : "DEPTH";
}
Viewer::Viewer(const Eigen::Vector4d& color)
  : mImpl(std::make_unique<Impl>(this, color))
{
  if (!color.allFinite())
    throw std::invalid_argument("Clear color must be finite");
  mImpl->defaultHandler = std::make_unique<DefaultEventHandler>(this);
}
Viewer::~Viewer()
{
  mImpl->closing = true;
  mImpl->finished = true;
  while (!mImpl->dragAndDrops.empty())
    disableDragAndDrop(mImpl->dragAndDrops.back().get());
  // Directly constructed observers also need detachment while members live.
  sendDestructionNotification();
  mImpl->defaultHandler.reset();
  removeViewerWorldOwners(this);
  for (const auto& item : mImpl->nodes)
    if (item.node->mViewer == this)
      item.node->mViewer = nullptr;
  auto attachments = std::move(mImpl->attachments);
  for (const auto& attachment : attachments)
    attachment->mViewer = nullptr;
  for (const auto& attachment : attachments) {
    try {
      attachment->customAttach(nullptr);
    } catch (const std::exception& error) {
      std::cerr << "Viewer attachment teardown: " << error.what() << '\n';
    } catch (...) {
      std::cerr << "Viewer attachment teardown failed\n";
    }
  }
}
void Viewer::addWorldNode(std::shared_ptr<WorldNode> node, bool active)
{
  if (mImpl->closing)
    throw std::logic_error(
        "Cannot add a world node while the Viewer is closing");
  if (!node)
    throw std::invalid_argument("WorldNode must not be null");
  if (std::any_of(
          mImpl->nodes.begin(), mImpl->nodes.end(), [&](const auto& entry) {
            return entry.node == node;
          }))
    return;
  Viewer* previousViewer = node->mViewer;
  const bool previouslySimulating = node->isSimulating();
  mImpl->nodes.push_back({node, active});
  node->mViewer = this;
  if (active)
    node->simulate(mImpl->simulating);
  try {
    node->setupViewer();
  } catch (...) {
    const bool stillAttached = node->mViewer == this;
    removeWorldNode(node);
    if (stillAttached) {
      node->mViewer = previousViewer;
      node->simulate(previouslySimulating);
    }
    throw;
  }
  viewerOwnsWorldSteps(this, node, active);
}
void Viewer::removeWorldNode(const std::shared_ptr<WorldNode>& node)
{
  if (!node)
    return;
  removeViewerWorldOwners(this, node.get());
  mImpl->nodes.erase(
      std::remove_if(
          mImpl->nodes.begin(),
          mImpl->nodes.end(),
          [&](const auto& entry) { return entry.node == node; }),
      mImpl->nodes.end());
  if (node && node->mViewer == this)
    node->mViewer = nullptr;
}
void Viewer::removeWorldNode(const std::shared_ptr<simulation::World>& world)
{
  removeWorldNode(getWorldNode(world));
}
std::shared_ptr<WorldNode> Viewer::getWorldNode(
    const std::shared_ptr<simulation::World>& world) const
{
  for (const auto& entry : mImpl->nodes)
    if (entry.node->getWorld() == world)
      return entry.node;
  return nullptr;
}
std::size_t Viewer::getNumWorldNodes() const
{
  return mImpl->nodes.size();
}
void Viewer::setWorldNodeActive(
    const std::shared_ptr<WorldNode>& node, bool active)
{
  for (auto& entry : mImpl->nodes)
    if (entry.node == node) {
      entry.active = active;
      if (active)
        node->simulate(mImpl->simulating);
      viewerOwnsWorldSteps(this, node, active);
    }
}
void Viewer::setWorldNodeActive(
    const std::shared_ptr<simulation::World>& world, bool active)
{
  setWorldNodeActive(getWorldNode(world), active);
}
bool Viewer::ownsWorldSteps(WorldNode* node)
{
  if (mImpl->closing || mImpl->finished || !mImpl->maySimulate)
    return false;
  const auto entry = std::find_if(
      mImpl->nodes.begin(), mImpl->nodes.end(), [node](const auto& item) {
        return item.node.get() == node;
      });
  return entry != mImpl->nodes.end()
         && viewerOwnsWorldSteps(this, entry->node, entry->active);
}
void Viewer::addAttachment(std::shared_ptr<ViewerAttachment> attachment)
{
  if (mImpl->closing)
    throw std::logic_error(
        "Cannot add an attachment while the Viewer is closing");
  if (!attachment)
    throw std::invalid_argument("Attachment must not be null");
  if (attachment->mViewer && attachment->mViewer != this)
    throw std::invalid_argument("Attachment already belongs to another Viewer");
  if (std::find(
          mImpl->attachments.begin(), mImpl->attachments.end(), attachment)
      != mImpl->attachments.end())
    return;
  mImpl->attachments.push_back(attachment);
  attachment->mViewer = this;
  try {
    attachment->customAttach(this);
  } catch (...) {
    auto& list = mImpl->attachments;
    list.erase(std::remove(list.begin(), list.end(), attachment), list.end());
    if (attachment->mViewer == this) {
      attachment->mViewer = nullptr;
      try {
        attachment->customAttach(nullptr);
      } catch (...) {
      }
    }
    throw;
  }
}
void Viewer::removeAttachment(
    const std::shared_ptr<ViewerAttachment>& attachment)
{
  auto& list = mImpl->attachments;
  const auto it = std::find(list.begin(), list.end(), attachment);
  if (it != list.end()) {
    const auto retained = *it;
    list.erase(it);
    retained->mViewer = nullptr;
    retained->customAttach(nullptr);
  }
}
const std::vector<std::shared_ptr<ViewerAttachment>>& Viewer::getAttachments()
    const
{
  return mImpl->attachments;
}
void Viewer::addWidget(std::shared_ptr<ImGuiWidget> widget)
{
  if (!widget)
    throw std::invalid_argument("Widget must not be null");
  if (std::find(mImpl->widgets.begin(), mImpl->widgets.end(), widget)
      == mImpl->widgets.end())
    mImpl->widgets.push_back(std::move(widget));
}
void Viewer::removeWidget(const std::shared_ptr<ImGuiWidget>& widget)
{
  auto& list = mImpl->widgets;
  list.erase(std::remove(list.begin(), list.end(), widget), list.end());
}
void Viewer::addEventHandler(std::shared_ptr<GUIEventHandler> handler)
{
  if (!handler)
    throw std::invalid_argument("Event handler must not be null");
  if (std::find(mImpl->handlers.begin(), mImpl->handlers.end(), handler)
      == mImpl->handlers.end())
    mImpl->handlers.push_back(std::move(handler));
}
void Viewer::removeEventHandler(const std::shared_ptr<GUIEventHandler>& handler)
{
  auto& handlers = mImpl->handlers;
  handlers.erase(
      std::remove(handlers.begin(), handlers.end(), handler), handlers.end());
}
bool Viewer::handleEvent(const GUIEventAdapter& event)
{
  if (mImpl->closing)
    return false;
  if (!std::isfinite(event.getX()) || !std::isfinite(event.getY())
      || !std::isfinite(event.getScrollingDeltaX())
      || !std::isfinite(event.getScrollingDeltaY()))
    throw std::invalid_argument("Input coordinates must be finite");
  GUIActionAdapter action(*this);
  const auto handlers = mImpl->handlers;
  for (const auto& handler : handlers) {
    if (std::find(mImpl->handlers.begin(), mImpl->handlers.end(), handler)
            != mImpl->handlers.end()
        && handler->handle(event, action))
      return true;
  }
  if (event.getEventType() == GUIEventAdapter::CLOSE_WINDOW
      || event.getEventType() == GUIEventAdapter::QUIT_APPLICATION) {
    setDone();
    return true;
  }
  if (mImpl->defaultHandlerEnabled
      && mImpl->defaultHandler->handle(event, action))
    return true;
  if (event.getEventType() == GUIEventAdapter::SCROLL) {
    mImpl->zoom(
        std::exp(std::clamp(-event.getScrollingDeltaY() * 0.12, -20.0, 20.0)));
    return true;
  }
  if (event.getEventType() == GUIEventAdapter::KEYDOWN) {
    if (event.getKey() == GUIEventAdapter::KEY_Escape) {
      setDone();
      return true;
    }
    if (event.getKey() == GUIEventAdapter::KEY_Home) {
      mImpl->eye = mImpl->homeEye;
      mImpl->center = mImpl->homeCenter;
      mImpl->up = mImpl->homeUp;
      return true;
    }
  }
  return mImpl->isDragging();
}
DefaultEventHandler* Viewer::getDefaultEventHandler() const
{
  return mImpl->closing ? nullptr : mImpl->defaultHandler.get();
}
void Viewer::switchDefaultEventHandler(bool on)
{
  mImpl->defaultHandlerEnabled = on;
}
void Viewer::addInstructionText(const std::string& text)
{
  mImpl->instructions.append(text);
}
const std::string& Viewer::getInstructions() const
{
  return mImpl->instructions;
}
DragAndDrop* Viewer::enableDragAndDrop(dynamics::Entity* entity)
{
  if (auto* frame = dynamic_cast<InteractiveFrame*>(entity))
    return enableDragAndDrop(frame);
  if (auto* body = dynamic_cast<dynamics::BodyNode*>(entity))
    return enableDragAndDrop(body);
  if (auto* frame = dynamic_cast<dynamics::SimpleFrame*>(entity))
    return enableDragAndDrop(frame);
  return nullptr;
}
SimpleFrameDnD* Viewer::enableDragAndDrop(dynamics::SimpleFrame* frame)
{
  if (mImpl->closing)
    throw std::logic_error(
        "Cannot add a drag handler while the Viewer is closing");
  if (!frame)
    throw std::invalid_argument("Drag frame must not be null");
  for (const auto& drag : mImpl->dragAndDrops) {
    auto* existing = drag.get();
    if (existing->getEntity() == frame
        && typeid(*existing) == typeid(SimpleFrameDnD))
      return static_cast<SimpleFrameDnD*>(existing);
  }
  auto drag = std::make_shared<SimpleFrameDnD>(this, frame);
  mImpl->dragAndDrops.push_back(drag);
  return drag.get();
}
SimpleFrameShapeDnD* Viewer::enableDragAndDrop(
    dynamics::SimpleFrame* frame, dynamics::Shape* shape)
{
  if (mImpl->closing)
    throw std::logic_error(
        "Cannot add a drag handler while the Viewer is closing");
  if (!frame || !shape)
    throw std::invalid_argument("Drag frame and shape must not be null");
  for (const auto& drag : mImpl->dragAndDrops)
    if (auto* existing = dynamic_cast<SimpleFrameShapeDnD*>(drag.get());
        existing && existing->getEntity() == frame
        && existing->getShape() == shape)
      return existing;
  auto drag = std::make_shared<SimpleFrameShapeDnD>(this, frame, shape);
  mImpl->dragAndDrops.push_back(drag);
  return drag.get();
}
InteractiveFrameDnD* Viewer::enableDragAndDrop(InteractiveFrame* frame)
{
  if (mImpl->closing)
    throw std::logic_error(
        "Cannot add a drag handler while the Viewer is closing");
  if (!frame)
    throw std::invalid_argument("Interactive frame must not be null");
  for (const auto& drag : mImpl->dragAndDrops)
    if (auto* existing = dynamic_cast<InteractiveFrameDnD*>(drag.get());
        existing && existing->getFrame() == frame)
      return existing;
  auto drag = std::make_shared<InteractiveFrameDnD>(this, frame);
  mImpl->dragAndDrops.push_back(drag);
  return drag.get();
}
BodyNodeDnD* Viewer::enableDragAndDrop(
    dynamics::BodyNode* body, bool external, bool wholeBody)
{
  if (mImpl->closing)
    throw std::logic_error(
        "Cannot add a drag handler while the Viewer is closing");
  if (!body)
    throw std::invalid_argument("Drag body must not be null");
  for (const auto& drag : mImpl->dragAndDrops)
    if (auto* existing = dynamic_cast<BodyNodeDnD*>(drag.get());
        existing && existing->getBodyNode() == body)
      return existing;
  auto drag = std::make_shared<BodyNodeDnD>(this, body, external, wholeBody);
  mImpl->dragAndDrops.push_back(drag);
  return drag.get();
}
bool Viewer::disableDragAndDrop(DragAndDrop* drag)
{
  auto& drags = mImpl->dragAndDrops;
  const auto found
      = std::find_if(drags.begin(), drags.end(), [drag](const auto& item) {
          return item.get() == drag;
        });
  if (found == drags.end())
    return false;
  const auto retained = *found;
  drags.erase(found);
  retained->detach();
  return true;
}
void Viewer::updateDragAndDrops()
{
  const auto drags = mImpl->dragAndDrops;
  for (const auto& drag : drags)
    if (drag->getEntity())
      drag->update();
}
void Viewer::computeIntersections(
    const GUIEventAdapter& event, std::vector<PickInfo>& picks)
{
  picks.clear();
  Eigen::Vector3d origin, farPoint;
  getRayUnderCursor(event.getX(), event.getY(), origin, farPoint);
  std::vector<std::shared_ptr<simulation::World>> worlds;
  for (const auto& entry : mImpl->nodes) {
    const auto world = entry.node->getWorld();
    if (world && std::find(worlds.begin(), worlds.end(), world) == worlds.end())
      worlds.push_back(world);
  }
  std::vector<std::pair<double, PickInfo>> hits;
  for (const auto& descriptor : mImpl->extractor.extract(worlds)) {
    const auto hit = intersectRenderableHit(
        descriptor,
        {origin, farPoint - origin},
        {mImpl->eye,
         mImpl->center,
         mImpl->up,
         mImpl->fovy,
         mImpl->width,
         mImpl->height});
    if (!hit || !descriptor.shapeFrame)
      continue;
    const double cameraDepth
        = (hit->point - mImpl->eye).dot(getCameraDirection());
    if (cameraDepth < viewerNearPlane || cameraDepth > viewerFarPlane)
      continue;
    auto* frame = const_cast<dynamics::ShapeFrame*>(descriptor.shapeFrame);
    PickInfo pick(frame, frame->getShape(), hit->point, hit->normal);
    pick.retainFrameOwner(worlds);
    hits.emplace_back(hit->distance, std::move(pick));
  }
  std::stable_sort(
      hits.begin(), hits.end(), [](const auto& first, const auto& second) {
        return first.first < second.first;
      });
  for (const auto& hit : hits)
    picks.push_back(hit.second);
}
void Viewer::getRayUnderCursor(
    double x,
    double y,
    Eigen::Vector3d& nearPoint,
    Eigen::Vector3d& farPoint) const
{
  if (!std::isfinite(x) || !std::isfinite(y))
    throw std::invalid_argument("Cursor coordinates must be finite");
  int width = mImpl->width, height = mImpl->height;
  if (mImpl->window)
    glfwGetWindowSize(mImpl->window, &width, &height);
  const Eigen::Vector3d forward = getCameraDirection();
  const Eigen::Vector3d right = forward.cross(mImpl->up).normalized();
  const Eigen::Vector3d up = right.cross(forward);
  const double tangent = std::tan(mImpl->fovy * 0.008726646259971648);
  const Eigen::Vector3d direction
      = (forward
         + right
               * ((2 * x / std::max(1, width) - 1) * tangent * width
                  / std::max(1, height))
         + up * ((1 - 2 * y / std::max(1, height)) * tangent))
            .normalized();
  const double cosine = direction.dot(forward);
  nearPoint = mImpl->eye + direction * (viewerNearPlane / cosine);
  farPoint = mImpl->eye + direction * (viewerFarPlane / cosine);
}
Eigen::Vector3d Viewer::getCameraDirection() const
{
  return (mImpl->center - mImpl->eye).normalized();
}
double Viewer::getWindowDepthRayParameter(double depth) const
{
  double cameraDepth
      = viewerNearPlane * viewerFarPlane
        / (viewerFarPlane * (1 - depth) + viewerNearPlane * depth);
  if (mImpl->renderer) {
    if (const auto projection = mImpl->renderer->getLastDepthProjection()) {
      const double ndcDepth = 2 * depth - 1;
      // Unproject clip z/w for perspective or orthographic depth.
      cameraDepth = ((*projection)[1] - ndcDepth * (*projection)[3])
                    / ((*projection)[0] - ndcDepth * (*projection)[2]);
    }
  }
  return (cameraDepth - viewerNearPlane) / (viewerFarPlane - viewerNearPlane);
}
void Viewer::setWireframe(bool enabled)
{
  mImpl->wireframe = enabled;
}
bool Viewer::isWireframe() const
{
  return mImpl->wireframe;
}
void Viewer::setBodyWireframe(dynamics::BodyNode* body, bool enabled)
{
  if (!body)
    throw std::invalid_argument("Wireframe body must not be null");
  if (enabled)
    mImpl->wireframeBodies[body] = body;
  else
    mImpl->wireframeBodies.erase(body);
}
bool Viewer::getBodyWireframe(const dynamics::BodyNode* body) const
{
  const auto found = mImpl->wireframeBodies.find(body);
  return found != mImpl->wireframeBodies.end()
         && found->second.lock() != nullptr;
}
void Viewer::simulate(bool on)
{
  if (on && !mImpl->maySimulate)
    return;
  mImpl->simulating = on;
  for (const auto& entry : mImpl->nodes)
    if (entry.active)
      entry.node->simulate(on);
}
bool Viewer::isSimulating() const
{
  return mImpl->simulating;
}
void Viewer::allowSimulation(bool allow)
{
  mImpl->maySimulate = allow;
  if (!allow)
    simulate(false);
}
bool Viewer::isAllowingSimulation() const
{
  return mImpl->maySimulate;
}
void Viewer::setCameraHome(
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up)
{
  if (!eye.allFinite() || !center.allFinite() || !up.allFinite()
      || (eye - center).norm() < 1e-12 || up.norm() < 1e-12
      || (center - eye).cross(up).norm() < 1e-12)
    throw std::invalid_argument(
        "Camera vectors must be finite, distinct and nonparallel");
  mImpl->eye = mImpl->homeEye = eye;
  mImpl->center = mImpl->homeCenter = center;
  mImpl->up = mImpl->homeUp = up.normalized();
}
void Viewer::setCameraHomePosition(
    const Eigen::Vector3d& eye,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& up)
{
  setCameraHome(eye, center, up);
}
void Viewer::home()
{
  mImpl->eye = mImpl->homeEye;
  mImpl->center = mImpl->homeCenter;
  mImpl->up = mImpl->homeUp;
}
void Viewer::setUpwardsDirection(const Eigen::Vector3d& up)
{
  const double norm = up.stableNorm();
  if (!up.allFinite() || !std::isfinite(norm))
    throw std::invalid_argument("Lighting up direction must be finite");
  if (norm > 0)
    mImpl->lightingUp = up / norm;
  else
    mImpl->lightingUp = Eigen::Vector3d::UnitZ();
}
void Viewer::setVerticalFieldOfView(double fov)
{
  if (!std::isfinite(fov) || fov <= 0 || fov >= 180)
    throw std::invalid_argument("FOV must be between 0 and 180 degrees");
  mImpl->fovy = fov;
}
double Viewer::getVerticalFieldOfView() const
{
  return mImpl->fovy;
}
void Viewer::setCameraMode(CameraMode mode)
{
  mImpl->cameraMode = mode;
}
CameraMode Viewer::getCameraMode() const
{
  return mImpl->cameraMode;
}
void Viewer::switchHeadlights(bool on)
{
  mImpl->headlights = on;
}
bool Viewer::checkHeadlights() const
{
  return mImpl->headlights;
}
void Viewer::setUpViewInWindow(int x, int y, int width, int height)
{
  mImpl->createGraphics(false, x, y, width, height);
}
void Viewer::setUpViewInWindow()
{
  mImpl->createDefaultWindow();
}
void Viewer::setUpViewForCapture(int width, int height)
{
  mImpl->createGraphics(true, 0, 0, width, height);
}
void Viewer::setGuiScale(double scale)
{
  if (!std::isfinite(scale) || scale <= 0)
    throw std::invalid_argument("GUI scale must be finite and positive");
  mImpl->requestedGuiScale = scale;
}
double Viewer::getGuiScale() const
{
  return mImpl->requestedGuiScale > 0 ? mImpl->requestedGuiScale
                                      : mImpl->detectedGuiScale;
}
void Viewer::setAutomaticGuiScale(bool automatic)
{
  mImpl->requestedGuiScale = automatic ? 0.0 : getGuiScale();
}
ImGuiContext* Viewer::getImGuiContext() const
{
  return mImpl->gui;
}
void Viewer::setBackgroundColor(const Eigen::Vector4d& color)
{
  if (!color.allFinite())
    throw std::invalid_argument("Background color must be finite");
  mImpl->background = color;
  if (mImpl->renderer)
    mImpl->renderer->setBackgroundColor(color);
}
void Viewer::setWindowTitle(const std::string& title)
{
  mImpl->title = title;
  if (mImpl->window)
    glfwSetWindowTitle(mImpl->window, title.c_str());
}
void Viewer::captureScreen(const std::string& filename)
{
  if (filename.empty())
    throw std::invalid_argument("Capture filename must not be empty");
  mImpl->capture = filename;
}
void Viewer::record(
    const std::string& directory,
    const std::string& prefix,
    bool restart,
    std::size_t digits)
{
  if (directory.empty() || digits == 0 || digits > 20)
    throw std::invalid_argument(
        "Recording requires a directory and 1..20 digits");
  mImpl->recordingDirectory = directory;
  mImpl->recordingPrefix = prefix;
  mImpl->recordingDigits = digits;
  if (restart)
    mImpl->recordingIndex = 0;
  mImpl->recording = true;
}
void Viewer::pauseRecording()
{
  mImpl->recording = false;
}
bool Viewer::isRecording() const
{
  return mImpl->recording;
}
void Viewer::updateViewer()
{
  updateDragAndDrops();
}
void Viewer::frame()
{
  frameImpl(nullptr);
}
bool Viewer::frameImpl(std::vector<std::uint8_t>* captureRGBA)
{
  if (mImpl->closing)
    return false;
  if (mImpl->framing)
    throw std::logic_error(
        "Viewer frame callbacks cannot recursively render the same viewer");
  if (viewerEventDispatch)
    throw std::logic_error(
        "Viewer frames cannot run from a window event callback");
  if (mImpl->finished)
    return false;
  if (!mImpl->renderer)
    mImpl->createDefaultWindow();
  ViewerFrameScope frameScope(mImpl->framing);
  bool minimized = false;
  if (mImpl->window) {
    // A handler error keeps the remaining input, and the OS queue, for the
    // next frame instead of dropping releases and focus changes.
    auto pending = std::exchange(mImpl->pendingInput, {});
    auto next = pending.begin();
    while (next != pending.end() && !mImpl->inputError)
      mImpl->withGui([&](auto& io) { (*next++)(*mImpl, io); });
    mImpl->pendingInput.insert(
        mImpl->pendingInput.begin(),
        std::make_move_iterator(next),
        std::make_move_iterator(pending.end()));
    if (!mImpl->inputError) {
      struct DispatchScope
      {
        DispatchScope()
        {
          viewerEventDispatch = true;
        }
        ~DispatchScope()
        {
          viewerEventDispatch = false;
          for (auto* window : std::exchange(deferredViewerWindows, {}))
            destroyViewerWindow(window);
        }
      } dispatch;
      glfwPollEvents();
    }
    mImpl->withGui([&](auto&) { mImpl->deliverKey(mImpl->keys.flush()); });
    if (mImpl->inputError) {
      const auto error = std::exchange(mImpl->inputError, {});
      std::rethrow_exception(error);
    }
    if (glfwWindowShouldClose(mImpl->window)) {
      mImpl->finished = true;
      return false;
    }
    int width, height;
    glfwGetFramebufferSize(mImpl->window, &width, &height);
    // Minimized windows (0x0 on Windows) keep simulating, as with OSG, but
    // render nothing.
    minimized = width <= 0 || height <= 0;
    if (!minimized && (width != mImpl->width || height != mImpl->height)) {
      mImpl->width = width;
      mImpl->height = height;
      detail::resizeFilamentNativeWindow(mImpl->window, width, height);
      mImpl->renderer->resize(width, height);
    }
  }
  ViewerImGuiScope scope(mImpl->gui);
  handleEvent(GUIEventAdapter(
      GUIEventAdapter::FRAME,
      0,
      mImpl->cursorX,
      mImpl->cursorY,
      mImpl->window ? mImpl->buttonMask() : 0,
      mImpl->window ? mImpl->modifiers()
                    : mImpl->defaultHandler->getModKeyMask()));
  updateViewer();
  const auto nodes = mImpl->nodes;
  std::vector<std::shared_ptr<simulation::World>> worlds;
  std::unordered_set<const simulation::World*> shadowedWorlds;
  for (const auto& entry : nodes) {
    if (std::none_of(
            mImpl->nodes.begin(), mImpl->nodes.end(), [&](const auto& current) {
              return current.node == entry.node;
            }))
      continue;
    entry.node->mViewer = this;
    entry.node->refreshForViewer(mImpl->maySimulate);
    if (std::none_of(
            mImpl->nodes.begin(), mImpl->nodes.end(), [&](const auto& current) {
              return current.node == entry.node;
            }))
      continue;
    if (const auto world = entry.node->getWorld()) {
      if (std::find(worlds.begin(), worlds.end(), world) == worlds.end())
        worlds.push_back(world);
      if (entry.node->isShadowed())
        shadowedWorlds.insert(world.get());
    }
  }
  const bool shadows = !shadowedWorlds.empty();
  const auto attachments = mImpl->attachments;
  for (const auto& attachment : attachments)
    if (attachment->getViewer() == this)
      attachment->refresh();
  if (minimized)
    return false;
  detail::collectDebugScene(mImpl->attachments, mImpl->debugScene);
  detail::prepareDebugScene(
      mImpl->debugScene,
      mImpl->eye,
      mImpl->center,
      mImpl->up,
      mImpl->fovy,
      mImpl->width,
      mImpl->height);
  mImpl->renderer->setDebugScene(mImpl->debugScene);
  mImpl->renderer->setShadowsEnabled(shadows);
  mImpl->renderer->setHeadlightsEnabled(mImpl->headlights);
  mImpl->renderer->setLightingUp(mImpl->lightingUp);
  mImpl->renderer->setDepthMode(mImpl->cameraMode == CameraMode::DEPTH);
  mImpl->renderer->setWireframe(mImpl->wireframe);
  mImpl->renderer->setCamera(
      mImpl->eye,
      mImpl->center,
      mImpl->up,
      mImpl->fovy,
      viewerNearPlane,
      viewerFarPlane);
  const auto& descriptors = mImpl->extractor.extract(worlds);
  mImpl->renderer->synchronize(descriptors);
  for (auto it = mImpl->wireframeBodies.begin();
       it != mImpl->wireframeBodies.end();) {
    if (!it->second.lock())
      it = mImpl->wireframeBodies.erase(it);
    else
      ++it;
  }
  for (const auto& descriptor : descriptors) {
    const auto* body = descriptor.shapeNode
                           ? descriptor.shapeNode->getBodyNodePtr().get()
                           : nullptr;
    mImpl->renderer->setWireframe(descriptor.id, getBodyWireframe(body));
    const bool shadowed = shadowedWorlds.count(descriptor.world) != 0;
    mImpl->renderer->setShadowsEnabled(
        descriptor.id,
        shadowed && descriptor.material.castsShadows,
        shadowed && descriptor.material.receivesShadows);
  }
  mImpl->updateGui();
  mImpl->overlay->prepareFonts(mImpl->debugScene);
  ImGui::NewFrame();
  const auto widgets = mImpl->widgets;
  try {
    for (const auto& widget : widgets)
      if (widget->isVisible())
        widget->render();
  } catch (...) {
    // A throwing widget may leave Begin/End unbalanced; recover the frame
    // instead of tripping ImGui's Debug error-recovery assertion.
    auto& io = ImGui::GetIO();
    const bool assertOnRecovery = io.ConfigErrorRecoveryEnableAssert;
    io.ConfigErrorRecoveryEnableAssert = false;
    ImGui::EndFrame();
    io.ConfigErrorRecoveryEnableAssert = assertOnRecovery;
    throw;
  }
  mImpl->overlay->render(
      mImpl->debugScene,
      mImpl->eye,
      mImpl->center,
      mImpl->up,
      mImpl->fovy,
      mImpl->width,
      mImpl->height);
  ImGui::Render();
  mImpl->renderer->renderImGui(ImGui::GetDrawData());
  const bool capture
      = captureRGBA || !mImpl->capture.empty() || mImpl->recording;
  if (!mImpl->renderer->frame(capture))
    return false;
  if (capture) {
    auto pixels = mImpl->renderer->consumeCapturedPixelsRGBA();
    if (!mImpl->capture.empty()) {
      // A failed write is reported once; later frames do not retry it.
      const auto file = std::exchange(mImpl->capture, std::string());
      writeViewerPng(file, mImpl->width, mImpl->height, pixels);
    }
    if (mImpl->recording) {
      std::ostringstream name;
      name << mImpl->recordingPrefix << std::setfill('0')
           << std::setw(static_cast<int>(mImpl->recordingDigits))
           << mImpl->recordingIndex++ << ".png";
      writeViewerPng(
          (std::filesystem::path(mImpl->recordingDirectory) / name.str())
              .string(),
          mImpl->width,
          mImpl->height,
          pixels);
    }
    if (captureRGBA)
      *captureRGBA = std::move(pixels);
  }
  return true;
}
void Viewer::run()
{
  while (!done())
    frame();
}
void Viewer::setDone(bool done)
{
  mImpl->finished = done;
}
bool Viewer::done() const
{
  return mImpl->finished;
}
std::string Viewer::getRenderBackend() const
{
  return mImpl->renderer ? mImpl->renderer->getBackendName() : std::string{};
}
std::string Viewer::getRenderDevice() const
{
  return mImpl->renderer ? mImpl->renderer->getDeviceName() : std::string{};
}
std::vector<std::uint8_t> detail::readViewerPixelsRGBA(Viewer& viewer)
{
  if (!viewer.mImpl->renderer)
    throw std::logic_error("Viewer graphics are not initialized");
  return viewer.mImpl->renderer->readPixelsRGBA();
}
std::vector<std::uint8_t> detail::captureViewerPixelsRGBA(Viewer& viewer)
{
  std::vector<std::uint8_t> pixels;
  if (!viewer.frameImpl(&pixels))
    throw std::runtime_error("No Filament frame was available for capture");
  return pixels;
}
} // namespace gui
} // namespace dart
#undef GLFW_INCLUDE_NONE
