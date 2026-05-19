/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/vsg/debug_draw.hpp>
#include <dart/gui/vsg/simple_viewer.hpp>

#include <gtest/gtest.h>

#include <exception>
#include <optional>
#include <string>

#include <cstddef>

namespace dart_vsg = dart::gui::vsg;

namespace {

template <typename CreateViewer>
std::optional<dart_vsg::SimpleViewer> tryCreateHeadlessViewer(
    CreateViewer createViewer, std::string& error)
{
  try {
    return createViewer();
  } catch (const std::exception& e) {
    error = e.what();
  } catch (...) {
    error = "unknown exception";
  }

  return std::nullopt;
}

std::optional<dart_vsg::SimpleViewer> tryCreateHeadlessViewer(
    int width, int height, std::string& error)
{
  return tryCreateHeadlessViewer(
      [width, height] {
        return dart_vsg::SimpleViewer::headless(width, height);
      },
      error);
}

std::optional<dart_vsg::SimpleViewer> tryCreateHeadlessViewerWithTag(
    int width, int height, std::string& error)
{
  return tryCreateHeadlessViewer(
      [width, height] {
        return dart_vsg::SimpleViewer(
            dart_vsg::SimpleViewer::HeadlessTag{}, width, height);
      },
      error);
}

} // namespace

TEST(VsgSimpleViewer, HeadlessFactory)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  EXPECT_TRUE(viewer->isHeadless());
}

TEST(VsgSimpleViewer, HeadlessTagConstruction)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewerWithTag(320, 240, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  EXPECT_TRUE(viewer->isHeadless());
}

TEST(VsgSimpleViewer, GetRoot)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  auto root = viewer->getRoot();
  ASSERT_NE(root, nullptr);
}

TEST(VsgSimpleViewer, AddNode)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  auto axes = dart_vsg::createAxes(1.0);

  viewer->addNode(axes);
  SUCCEED();
}

TEST(VsgSimpleViewer, AddGrid)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  viewer->addGrid(5.0, 0.5);
  SUCCEED();
}

TEST(VsgSimpleViewer, AddAxes)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  viewer->addAxes(2.0);
  SUCCEED();
}

TEST(VsgSimpleViewer, LookAt)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  viewer->lookAt(
      Eigen::Vector3d(5.0, 5.0, 5.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d::UnitZ());
  SUCCEED();
}

TEST(VsgSimpleViewer, SetBackgroundColor)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  viewer->setBackgroundColor(Eigen::Vector4d(0.1, 0.2, 0.3, 1.0));
  SUCCEED();
}

TEST(VsgSimpleViewer, SetScene)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  auto axes = dart_vsg::createAxes(1.0);
  viewer->setScene(axes);
  SUCCEED();
}

TEST(VsgSimpleViewer, Clear)
{
  std::string error;
  auto viewer = tryCreateHeadlessViewer(640, 480, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  viewer->addAxes(1.0);
  viewer->addGrid(5.0, 1.0);
  viewer->clear();
  SUCCEED();
}

TEST(VsgSimpleViewer, HeadlessCaptureRendersBackground)
{
  constexpr int width = 128;
  constexpr int height = 128;
  constexpr int channels = 4;

  std::string error;
  auto viewer = tryCreateHeadlessViewer(width, height, error);
  if (!viewer) {
    GTEST_SKIP() << "Headless VSG unavailable: " << error;
  }

  viewer->setBackgroundColor(Eigen::Vector4d(0.1, 0.2, 0.3, 1.0));

  const auto buffer = viewer->captureBuffer();
  if (buffer.empty()) {
    GTEST_SKIP() << "Headless VSG capture unavailable";
  }

  ASSERT_EQ(buffer.size(), static_cast<std::size_t>(width * height * channels));

  bool hasBackgroundPixel = false;
  for (std::size_t i = 0; i + 2 < buffer.size(); i += channels) {
    if (buffer[i] != 0 || buffer[i + 1] != 0 || buffer[i + 2] != 0) {
      hasBackgroundPixel = true;
      break;
    }
  }

  EXPECT_TRUE(hasBackgroundPixel);
}
