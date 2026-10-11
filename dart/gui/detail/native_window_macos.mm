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

#include "native_window.hpp"
#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#import <AppKit/AppKit.h>
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#include <backend/platforms/PlatformMetal-ObjC.h>
#include <backend/platforms/PlatformMetal.h>
#include <filament/Engine.h>
namespace dart::gui::detail {
std::string configureFilamentMetalWindow(
    ::filament::Engine& engine, void* nativeWindow)
{
  auto* platform
      = static_cast<::filament::backend::PlatformMetal*>(engine.getPlatform());
  ::filament::backend::MetalDevice device;
  platform->createDevice(device);
  if (nativeWindow != nullptr)
    ((CAMetalLayer*)nativeWindow).device = device.device;
  return std::string([[device.device name] UTF8String]);
}
void* getFilamentNativeWindow(GLFWwindow* window)
{
  NSWindow* native = (NSWindow*)glfwGetCocoaWindow(window);
  NSView* view = [native contentView];
  [view setWantsLayer:YES];
  CAMetalLayer* layer = [CAMetalLayer layer];
  layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
  layer.framebufferOnly = NO;
  [view setLayer:layer];
  int width = 0, height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  resizeFilamentNativeWindow(window, width, height);
  return (void*)layer;
}
void resizeFilamentNativeWindow(GLFWwindow* window, int width, int height)
{
  NSWindow* native = (NSWindow*)glfwGetCocoaWindow(window);
  CAMetalLayer* layer = (CAMetalLayer*)[[native contentView] layer];
  layer.contentsScale = [native backingScaleFactor];
  layer.drawableSize = CGSizeMake(width, height);
}
void releaseFilamentNativeWindow(GLFWwindow* window)
{
  NSWindow* native = (NSWindow*)glfwGetCocoaWindow(window);
  [[native contentView] setLayer:nil];
}
}
