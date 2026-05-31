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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; OR
 *   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 *   DAMAGE.
 */

#include "input.hpp"

#include "scenes.hpp"
#include "selection.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <array>
#include <limits>

#include <cctype>

namespace dart::gui::detail {

using dart::gui::addOrbitCameraScroll;
using dart::gui::computeCameraRelativeNudge;
using dart::gui::DirectionalNudgeInput;
using dart::gui::KeyboardActionContext;
using dart::gui::KeyboardActionTrigger;
using dart::gui::KeyboardKey;
using dart::gui::KeyboardShortcut;
using dart::gui::OrbitCamera;
using dart::gui::OrbitCameraController;
using dart::gui::OrbitCameraControllerInput;
using dart::gui::OrbitCameraControlOptions;
using dart::gui::OrbitCameraMouseMode;
using dart::gui::requestSingleStep;
using dart::gui::resetOrbitCameraTracking;
using dart::gui::togglePaused;
using dart::gui::updateOrbitCameraController;

namespace {

#if GLFW_VERSION_MAJOR > 3                                                     \
    || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 1)
  #define DART_GUI_DETAIL_HAS_GLFW_STANDARD_CURSORS 1
#else
  #define DART_GUI_DETAIL_HAS_GLFW_STANDARD_CURSORS 0
#endif

#if GLFW_VERSION_MAJOR > 3                                                     \
    || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 4)
  #define DART_GUI_DETAIL_HAS_GLFW_EXTENDED_CURSORS 1
#else
  #define DART_GUI_DETAIL_HAS_GLFW_EXTENDED_CURSORS 0
#endif

#if DART_GUI_DETAIL_HAS_GLFW_STANDARD_CURSORS
struct ImGuiMouseCursorState
{
  std::array<GLFWcursor*, ImGuiMouseCursor_COUNT> cursors{};
  GLFWcursor* lastCursor = nullptr;
  bool initialized = false;
  bool cursorHidden = false;
};

ImGuiMouseCursorState& imguiMouseCursorState()
{
  static ImGuiMouseCursorState state;
  return state;
}

GLFWcursor* createStandardCursor(int shape)
{
  GLFWerrorfun previousErrorCallback = glfwSetErrorCallback(nullptr);
  GLFWcursor* cursor = glfwCreateStandardCursor(shape);
  glfwSetErrorCallback(previousErrorCallback);
  return cursor;
}

void initializeImGuiMouseCursors(ImGuiMouseCursorState& state)
{
  if (state.initialized) {
    return;
  }

  state.cursors[ImGuiMouseCursor_Arrow]
      = createStandardCursor(GLFW_ARROW_CURSOR);
  state.cursors[ImGuiMouseCursor_TextInput]
      = createStandardCursor(GLFW_IBEAM_CURSOR);
  state.cursors[ImGuiMouseCursor_ResizeNS]
      = createStandardCursor(GLFW_VRESIZE_CURSOR);
  state.cursors[ImGuiMouseCursor_ResizeEW]
      = createStandardCursor(GLFW_HRESIZE_CURSOR);
  state.cursors[ImGuiMouseCursor_Hand] = createStandardCursor(GLFW_HAND_CURSOR);
  #if DART_GUI_DETAIL_HAS_GLFW_EXTENDED_CURSORS
  state.cursors[ImGuiMouseCursor_ResizeAll]
      = createStandardCursor(GLFW_RESIZE_ALL_CURSOR);
  state.cursors[ImGuiMouseCursor_ResizeNESW]
      = createStandardCursor(GLFW_RESIZE_NESW_CURSOR);
  state.cursors[ImGuiMouseCursor_ResizeNWSE]
      = createStandardCursor(GLFW_RESIZE_NWSE_CURSOR);
  state.cursors[ImGuiMouseCursor_NotAllowed]
      = createStandardCursor(GLFW_NOT_ALLOWED_CURSOR);
  #endif
  state.initialized = true;
}

GLFWcursor* cursorForImGuiMouseCursor(
    const ImGuiMouseCursorState& state, ImGuiMouseCursor imguiCursor)
{
  if (imguiCursor >= 0 && imguiCursor < ImGuiMouseCursor_COUNT) {
    if (GLFWcursor* cursor = state.cursors[imguiCursor]) {
      return cursor;
    }
  }

  return state.cursors[ImGuiMouseCursor_Arrow];
}
#endif

void updateImGuiKeyModifiers(GLFWwindow* window, ImGuiIO& io)
{
  if (window == nullptr) {
    return;
  }

  io.AddKeyEvent(
      ImGuiMod_Ctrl,
      glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
          || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);
  io.AddKeyEvent(
      ImGuiMod_Shift,
      glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
          || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
  io.AddKeyEvent(
      ImGuiMod_Alt,
      glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS
          || glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS);
  io.AddKeyEvent(
      ImGuiMod_Super,
      glfwGetKey(window, GLFW_KEY_LEFT_SUPER) == GLFW_PRESS
          || glfwGetKey(window, GLFW_KEY_RIGHT_SUPER) == GLFW_PRESS);
}

ImGuiKey imguiKeyForPrintableCharacter(char character)
{
  const unsigned char unsignedCharacter = static_cast<unsigned char>(character);
  const int lowerCharacter = std::tolower(unsignedCharacter);
  if (lowerCharacter >= 'a' && lowerCharacter <= 'z') {
    return static_cast<ImGuiKey>(
        ImGuiKey_A + (lowerCharacter - static_cast<int>('a')));
  }

  if (unsignedCharacter >= static_cast<unsigned char>('0')
      && unsignedCharacter <= static_cast<unsigned char>('9')) {
    return static_cast<ImGuiKey>(
        ImGuiKey_0 + (unsignedCharacter - static_cast<unsigned char>('0')));
  }

  switch (character) {
    case ' ':
      return ImGuiKey_Space;
    case '\'':
      return ImGuiKey_Apostrophe;
    case ',':
      return ImGuiKey_Comma;
    case '-':
      return ImGuiKey_Minus;
    case '.':
      return ImGuiKey_Period;
    case '/':
      return ImGuiKey_Slash;
    case ';':
      return ImGuiKey_Semicolon;
    case '=':
      return ImGuiKey_Equal;
    case '[':
      return ImGuiKey_LeftBracket;
    case '\\':
      return ImGuiKey_Backslash;
    case ']':
      return ImGuiKey_RightBracket;
    case '`':
      return ImGuiKey_GraveAccent;
    default:
      return ImGuiKey_None;
  }
}

bool isLayoutTranslatedGlfwKey(int key)
{
  return key == GLFW_KEY_SPACE
         || (key >= GLFW_KEY_APOSTROPHE && key <= GLFW_KEY_GRAVE_ACCENT)
         || key == GLFW_KEY_WORLD_1 || key == GLFW_KEY_WORLD_2;
}

ImGuiKey imguiKeyForTranslatedPrintableGlfwKey(int key, int scancode)
{
  if (!isLayoutTranslatedGlfwKey(key)) {
    return ImGuiKey_None;
  }

  // GLFW key tokens are US-position codes; key names follow the active layout.
  const char* keyName = glfwGetKeyName(key, scancode);
  if (keyName == nullptr || keyName[0] == '\0' || keyName[1] != '\0') {
    return ImGuiKey_None;
  }

  return imguiKeyForPrintableCharacter(keyName[0]);
}

ImGuiKey imguiKeyForGlfwKey(int key, int scancode)
{
  const ImGuiKey translatedKey
      = imguiKeyForTranslatedPrintableGlfwKey(key, scancode);
  if (translatedKey != ImGuiKey_None) {
    return translatedKey;
  }

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
    case GLFW_KEY_WORLD_1:
    case GLFW_KEY_WORLD_2:
      return ImGuiKey_Oem102;
    case GLFW_KEY_RIGHT_BRACKET:
      return ImGuiKey_RightBracket;
    case GLFW_KEY_GRAVE_ACCENT:
      return ImGuiKey_GraveAccent;
    case GLFW_KEY_CAPS_LOCK:
      return ImGuiKey_CapsLock;
    case GLFW_KEY_SCROLL_LOCK:
      return ImGuiKey_ScrollLock;
    case GLFW_KEY_NUM_LOCK:
      return ImGuiKey_NumLock;
    case GLFW_KEY_PRINT_SCREEN:
      return ImGuiKey_PrintScreen;
    case GLFW_KEY_PAUSE:
      return ImGuiKey_Pause;
    case GLFW_KEY_KP_0:
      return ImGuiKey_Keypad0;
    case GLFW_KEY_KP_1:
      return ImGuiKey_Keypad1;
    case GLFW_KEY_KP_2:
      return ImGuiKey_Keypad2;
    case GLFW_KEY_KP_3:
      return ImGuiKey_Keypad3;
    case GLFW_KEY_KP_4:
      return ImGuiKey_Keypad4;
    case GLFW_KEY_KP_5:
      return ImGuiKey_Keypad5;
    case GLFW_KEY_KP_6:
      return ImGuiKey_Keypad6;
    case GLFW_KEY_KP_7:
      return ImGuiKey_Keypad7;
    case GLFW_KEY_KP_8:
      return ImGuiKey_Keypad8;
    case GLFW_KEY_KP_9:
      return ImGuiKey_Keypad9;
    case GLFW_KEY_KP_DECIMAL:
      return ImGuiKey_KeypadDecimal;
    case GLFW_KEY_KP_DIVIDE:
      return ImGuiKey_KeypadDivide;
    case GLFW_KEY_KP_MULTIPLY:
      return ImGuiKey_KeypadMultiply;
    case GLFW_KEY_KP_SUBTRACT:
      return ImGuiKey_KeypadSubtract;
    case GLFW_KEY_KP_ADD:
      return ImGuiKey_KeypadAdd;
    case GLFW_KEY_KP_ENTER:
      return ImGuiKey_KeypadEnter;
    case GLFW_KEY_KP_EQUAL:
      return ImGuiKey_KeypadEqual;
    case GLFW_KEY_LEFT_SHIFT:
      return ImGuiKey_LeftShift;
    case GLFW_KEY_LEFT_CONTROL:
      return ImGuiKey_LeftCtrl;
    case GLFW_KEY_LEFT_ALT:
      return ImGuiKey_LeftAlt;
    case GLFW_KEY_LEFT_SUPER:
      return ImGuiKey_LeftSuper;
    case GLFW_KEY_RIGHT_SHIFT:
      return ImGuiKey_RightShift;
    case GLFW_KEY_RIGHT_CONTROL:
      return ImGuiKey_RightCtrl;
    case GLFW_KEY_RIGHT_ALT:
      return ImGuiKey_RightAlt;
    case GLFW_KEY_RIGHT_SUPER:
      return ImGuiKey_RightSuper;
    case GLFW_KEY_MENU:
      return ImGuiKey_Menu;
    default:
      break;
  }

  if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9) {
    return static_cast<ImGuiKey>(ImGuiKey_0 + (key - GLFW_KEY_0));
  }
  if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z) {
    return static_cast<ImGuiKey>(ImGuiKey_A + (key - GLFW_KEY_A));
  }
  if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F24) {
    return static_cast<ImGuiKey>(ImGuiKey_F1 + (key - GLFW_KEY_F1));
  }

  return ImGuiKey_None;
}

void handleKey(
    GLFWwindow* window, int key, int scancode, int action, int /*mods*/)
{
  if (ImGui::GetCurrentContext() == nullptr
      || (action != GLFW_PRESS && action != GLFW_RELEASE
          && action != GLFW_REPEAT)) {
    return;
  }

  ImGuiIO& io = ImGui::GetIO();
  updateImGuiKeyModifiers(window, io);

  const ImGuiKey imguiKey = imguiKeyForGlfwKey(key, scancode);
  if (imguiKey == ImGuiKey_None) {
    return;
  }

  const bool pressed = action != GLFW_RELEASE;
  io.AddKeyEvent(imguiKey, pressed);
  io.SetKeyEventNativeData(imguiKey, key, scancode);
}

void handleChar(GLFWwindow*, unsigned int codepoint)
{
  if (ImGui::GetCurrentContext() == nullptr) {
    return;
  }

  ImGui::GetIO().AddInputCharacter(codepoint);
}

void handleScroll(GLFWwindow* window, double xOffset, double yOffset)
{
  // Forward the wheel event to ImGui so panels can scroll. Without this,
  // ImGui never sees the wheel and WantCaptureMouse stays false, so the
  // sidebar list (e.g. the Demos catalog) never scrolls under the cursor.
  if (ImGui::GetCurrentContext() != nullptr) {
    ImGui::GetIO().AddMouseWheelEvent(
        static_cast<float>(xOffset), static_cast<float>(yOffset));
    if (ImGui::GetIO().WantCaptureMouse) {
      return;
    }
  }

  auto* controller = static_cast<dart::gui::OrbitCameraController*>(
      glfwGetWindowUserPointer(window));
  if (controller != nullptr) {
    addOrbitCameraScroll(*controller, yOffset);
  }
}

int glfwKeyForShortcut(const KeyboardShortcut& shortcut)
{
  if (shortcut.character != '\0') {
    const unsigned char value = static_cast<unsigned char>(shortcut.character);
    if (std::isalpha(value)) {
      return std::toupper(value);
    }

    switch (shortcut.character) {
      case '[':
      case '{':
        return GLFW_KEY_LEFT_BRACKET;
      case ']':
      case '}':
        return GLFW_KEY_RIGHT_BRACKET;
      case '/':
      case '?':
        return GLFW_KEY_SLASH;
      case '\\':
        return GLFW_KEY_BACKSLASH;
      default:
        break;
    }

    return value;
  }

  switch (shortcut.key) {
    case KeyboardKey::Tab:
      return GLFW_KEY_TAB;
    case KeyboardKey::Enter:
      return GLFW_KEY_ENTER;
    case KeyboardKey::Backspace:
      return GLFW_KEY_BACKSPACE;
    case KeyboardKey::Delete:
      return GLFW_KEY_DELETE;
    case KeyboardKey::Up:
      return GLFW_KEY_UP;
    case KeyboardKey::Down:
      return GLFW_KEY_DOWN;
    case KeyboardKey::Left:
      return GLFW_KEY_LEFT;
    case KeyboardKey::Right:
      return GLFW_KEY_RIGHT;
    case KeyboardKey::PageUp:
      return GLFW_KEY_PAGE_UP;
    case KeyboardKey::PageDown:
      return GLFW_KEY_PAGE_DOWN;
    case KeyboardKey::GraveAccent:
      return GLFW_KEY_GRAVE_ACCENT;
    case KeyboardKey::Unknown:
      return GLFW_KEY_UNKNOWN;
  }

  return GLFW_KEY_UNKNOWN;
}

bool isShiftDown(GLFWwindow* window)
{
  return glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
         || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

bool isShortcutDown(GLFWwindow* window, const KeyboardShortcut& shortcut)
{
  const int glfwKey = glfwKeyForShortcut(shortcut);
  if (glfwKey == GLFW_KEY_UNKNOWN
      || glfwGetKey(window, glfwKey) != GLFW_PRESS) {
    return false;
  }

  switch (shortcut.character) {
    case '{':
    case '}':
    case '?':
      return isShiftDown(window);
    case '[':
    case ']':
    case '/':
      return !isShiftDown(window);
    default:
      break;
  }

  const unsigned char character
      = static_cast<unsigned char>(shortcut.character);
  if (std::isalpha(character)) {
    return (std::isupper(character) != 0) == isShiftDown(window);
  }

  return true;
}

void resizeKeyboardActionStateIfNeeded(
    const DartScene& scene, ApplicationInputState& state)
{
  if (state.customActionWasPressed.size() != scene.keyboardActions.size()
      || state.customActionSuppressUntilReleased.size()
             != scene.keyboardActions.size()) {
    state.customActionWasPressed.assign(scene.keyboardActions.size(), false);
    state.customActionSuppressUntilReleased.assign(
        scene.keyboardActions.size(), false);
  }
}

void updateKeyboardActionCaptureState(
    GLFWwindow* window, const DartScene& scene, ApplicationInputState& state)
{
  resizeKeyboardActionStateIfNeeded(scene, state);

  for (std::size_t i = 0; i < scene.keyboardActions.size(); ++i) {
    const bool isPressed
        = isShortcutDown(window, scene.keyboardActions[i].shortcut);
    state.customActionWasPressed[i] = isPressed;
    if (isPressed) {
      state.customActionSuppressUntilReleased[i] = true;
    } else {
      state.customActionSuppressUntilReleased[i] = false;
    }
  }
}

KeyboardActionContext makeKeyboardActionContext(
    dart::gui::ViewerLifecycleState& lifecycle,
    dart::gui::RenderSettings& renderSettings,
    OrbitCameraController& cameraController,
    const OrbitCamera& homeCamera)
{
  KeyboardActionContext context;
  context.lifecycle = &lifecycle;
  context.renderSettings = &renderSettings;
  context.resetCamera = [&cameraController, homeCamera]() {
    cameraController.camera = homeCamera;
    resetOrbitCameraTracking(cameraController);
  };

  return context;
}

void dispatchKeyboardActions(
    GLFWwindow* window,
    DartScene& scene,
    dart::gui::ViewerLifecycleState& lifecycle,
    OrbitCameraController& cameraController,
    const OrbitCamera& homeCamera,
    ApplicationInputState& state)
{
  resizeKeyboardActionStateIfNeeded(scene, state);

  KeyboardActionContext context = makeKeyboardActionContext(
      lifecycle, scene.renderSettings, cameraController, homeCamera);

  for (std::size_t i = 0; i < scene.keyboardActions.size(); ++i) {
    const auto& action = scene.keyboardActions[i];
    const bool isPressed = isShortcutDown(window, action.shortcut);
    const bool wasPressed = state.customActionWasPressed[i];
    const bool suppressUntilReleased
        = state.customActionSuppressUntilReleased[i];
    const bool shouldTrigger
        = !suppressUntilReleased
          && (action.trigger == KeyboardActionTrigger::Release
                  ? (!isPressed && wasPressed)
                  : (isPressed && (action.repeat || !wasPressed)));
    if (shouldTrigger && action.callback) {
      action.callback(context);
    }
    if (!isPressed) {
      state.customActionSuppressUntilReleased[i] = false;
    }
    state.customActionWasPressed[i] = isPressed;
  }
}

} // namespace

void attachOrbitCameraController(
    GLFWwindow* window, dart::gui::OrbitCameraController& controller)
{
  if (window == nullptr) {
    return;
  }

  glfwSetWindowUserPointer(window, &controller);
  glfwSetKeyCallback(window, handleKey);
  glfwSetCharCallback(window, handleChar);
  glfwSetScrollCallback(window, handleScroll);
}

bool isKeyDown(GLFWwindow* window, int key)
{
  return window != nullptr && glfwGetKey(window, key) == GLFW_PRESS;
}

void pollApplicationInput(
    GLFWwindow* window,
    DartScene& scene,
    SelectionController& selectionController,
    dart::gui::ViewerLifecycleState& lifecycle,
    OrbitCameraController& cameraController,
    const OrbitCamera& homeCamera,
    ApplicationInputState& state,
    bool& showPerfHud)
{
  if (window == nullptr) {
    return;
  }

  glfwPollEvents();
  const bool uiCapturesKeyboard = ImGui::GetCurrentContext() != nullptr
                                  && ImGui::GetIO().WantCaptureKeyboard;
  const bool isEscapePressed
      = glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
  if (!uiCapturesKeyboard && isEscapePressed && !state.wasEscapePressed) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
  state.wasEscapePressed = isEscapePressed;

  const bool isSpacePressed = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
  if (!uiCapturesKeyboard && isSpacePressed && !state.wasSpacePressed) {
    togglePaused(lifecycle);
  }
  state.wasSpacePressed = isSpacePressed;

  // F2 toggles the performance HUD at runtime (no need to pass --perf-hud).
  const bool isPerfHudKeyPressed
      = glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS;
  if (!uiCapturesKeyboard && isPerfHudKeyPressed
      && !state.wasPerfHudKeyPressed) {
    showPerfHud = !showPerfHud;
  }
  state.wasPerfHudKeyPressed = isPerfHudKeyPressed;

  const bool isStepPressed = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
  if (!uiCapturesKeyboard && isStepPressed && !state.wasStepPressed) {
    requestSingleStep(lifecycle, false);
  }
  state.wasStepPressed = isStepPressed;

  if (!uiCapturesKeyboard) {
    for (const auto& handle : scene.ikHandles) {
      if (glfwGetKey(window, handle.hotkey) == GLFW_PRESS) {
        selectionController.select(
            handle.targetRenderableId, handle.label + " IK target");
        lifecycle.paused = true;
      }
    }

    dispatchKeyboardActions(
        window, scene, lifecycle, cameraController, homeCamera, state);
  } else {
    updateKeyboardActionCaptureState(window, scene, state);
  }
}

void updateImGuiMouseInput(
    GLFWwindow* window,
    ImGuiIO& io,
    int framebufferWidth,
    int framebufferHeight)
{
#if DART_GUI_DETAIL_HAS_GLFW_STANDARD_CURSORS
  io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
#endif

  for (bool& mouseDown : io.MouseDown) {
    mouseDown = false;
  }

  if (window == nullptr) {
    const float offscreen = -std::numeric_limits<float>::max();
    io.MousePos = ImVec2(offscreen, offscreen);
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);
    return;
  }

  int windowWidth = framebufferWidth;
  int windowHeight = framebufferHeight;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  const float xScale = windowWidth > 0 ? static_cast<float>(framebufferWidth)
                                             / static_cast<float>(windowWidth)
                                       : 1.0f;
  const float yScale = windowHeight > 0 ? static_cast<float>(framebufferHeight)
                                              / static_cast<float>(windowHeight)
                                        : 1.0f;
  io.DisplayFramebufferScale = ImVec2(xScale, yScale);

  double cursorX = 0.0;
  double cursorY = 0.0;
  glfwGetCursorPos(window, &cursorX, &cursorY);
  io.MousePos = ImVec2(
      static_cast<float>(cursorX) * xScale,
      static_cast<float>(cursorY) * yScale);
  io.MouseDown[0]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  io.MouseDown[1]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  io.MouseDown[2]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
}

void updateImGuiMouseCursor(GLFWwindow* window, ImGuiIO& io)
{
#if DART_GUI_DETAIL_HAS_GLFW_STANDARD_CURSORS
  io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
  if (window == nullptr || ImGui::GetCurrentContext() == nullptr) {
    return;
  }

  auto& state = imguiMouseCursorState();
  initializeImGuiMouseCursors(state);

  if ((io.ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange) != 0
      || glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
    state.lastCursor = nullptr;
    state.cursorHidden = false;
    return;
  }

  const ImGuiMouseCursor imguiCursor = ImGui::GetMouseCursor();
  if (imguiCursor == ImGuiMouseCursor_None || io.MouseDrawCursor) {
    if (!state.cursorHidden) {
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
      state.lastCursor = nullptr;
      state.cursorHidden = true;
    }
    return;
  }

  GLFWcursor* cursor = cursorForImGuiMouseCursor(state, imguiCursor);
  if (state.cursorHidden) {
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    state.cursorHidden = false;
  }
  if (state.lastCursor != cursor) {
    glfwSetCursor(window, cursor);
    state.lastCursor = cursor;
  }
#else
  (void)window;
  (void)io;
#endif
}

void destroyImGuiMouseCursors(GLFWwindow* window)
{
#if DART_GUI_DETAIL_HAS_GLFW_STANDARD_CURSORS
  auto& state = imguiMouseCursorState();
  if (window != nullptr) {
    glfwSetCursor(window, nullptr);
  }
  for (GLFWcursor*& cursor : state.cursors) {
    if (cursor != nullptr) {
      glfwDestroyCursor(cursor);
      cursor = nullptr;
    }
  }
  state.lastCursor = nullptr;
  state.initialized = false;
  state.cursorHidden = false;
#else
  (void)window;
#endif
}

bool isSceneMouseInputCapturedByUi(bool showUi, const ImGuiIO& io)
{
  return showUi && io.WantCaptureMouse;
}

bool isDragModifierDown(GLFWwindow* window)
{
  return isKeyDown(window, GLFW_KEY_LEFT_CONTROL)
         || isKeyDown(window, GLFW_KEY_RIGHT_CONTROL);
}

bool isRotationDragModifierDown(GLFWwindow* window)
{
  return isDragModifierDown(window) && isShiftDown(window);
}

std::optional<Eigen::Vector3d> selectedDragAxisFromKeyboard(GLFWwindow* window)
{
  if (isKeyDown(window, GLFW_KEY_X) || isKeyDown(window, GLFW_KEY_1)) {
    return Eigen::Vector3d::UnitX();
  }
  if (isKeyDown(window, GLFW_KEY_Y) || isKeyDown(window, GLFW_KEY_2)) {
    return Eigen::Vector3d::UnitY();
  }
  if (isKeyDown(window, GLFW_KEY_Z) || isKeyDown(window, GLFW_KEY_3)) {
    return Eigen::Vector3d::UnitZ();
  }

  return std::nullopt;
}

Eigen::Vector3d selectedNudgeFromKeyboard(
    GLFWwindow* window, const dart::gui::OrbitCamera& camera, double stepSize)
{
  if (window == nullptr) {
    return Eigen::Vector3d::Zero();
  }

  DirectionalNudgeInput input;
  input.left = isKeyDown(window, GLFW_KEY_LEFT);
  input.right = isKeyDown(window, GLFW_KEY_RIGHT);
  input.forward = isKeyDown(window, GLFW_KEY_UP);
  input.backward = isKeyDown(window, GLFW_KEY_DOWN);
  input.up = isKeyDown(window, GLFW_KEY_PAGE_UP);
  input.down = isKeyDown(window, GLFW_KEY_PAGE_DOWN);
  input.fast = isKeyDown(window, GLFW_KEY_LEFT_SHIFT)
               || isKeyDown(window, GLFW_KEY_RIGHT_SHIFT);
  input.stepSize = stepSize;
  return computeCameraRelativeNudge(camera, input);
}

OrbitCameraControllerInput makeOrbitCameraControllerInput(
    double cursorX,
    double cursorY,
    bool leftMousePressed,
    bool rightMousePressed,
    bool middleMousePressed,
    bool suppressCameraMouse,
    bool dragModifierDown,
    const dart::gui::OrbitCameraControlOptions& controls)
{
  OrbitCameraControllerInput input;
  input.cursorX = cursorX;
  input.cursorY = cursorY;
  input.locked = controls.locked;
  const bool leftMouse
      = leftMousePressed && !suppressCameraMouse && !dragModifierDown;
  const bool rightOrMiddleMouse
      = !suppressCameraMouse && (rightMousePressed || middleMousePressed);
  input.orbit = leftMouse && controls.mouseMode == OrbitCameraMouseMode::Orbit;
  input.pan = (leftMouse && controls.mouseMode == OrbitCameraMouseMode::Pan)
              || rightOrMiddleMouse;
  input.zoom = leftMouse && controls.mouseMode == OrbitCameraMouseMode::Zoom;
  return input;
}

void updateCameraController(
    GLFWwindow* window,
    dart::gui::OrbitCameraController& controller,
    bool suppressCameraMouse,
    const dart::gui::OrbitCameraControlOptions& controls)
{
  if (window == nullptr) {
    return;
  }

  double x = 0.0;
  double y = 0.0;
  glfwGetCursorPos(window, &x, &y);

  const OrbitCameraControllerInput input = makeOrbitCameraControllerInput(
      x,
      y,
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS,
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS,
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS,
      suppressCameraMouse,
      isDragModifierDown(window),
      controls);
  updateOrbitCameraController(controller, input);
}

} // namespace dart::gui::detail
