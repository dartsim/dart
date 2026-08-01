/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include <dart/gui/detail/imgui_draw_data.hpp>

#include <gtest/gtest.h>
#include <imgui.h>

#include <algorithm>
#include <limits>

#include <cstdint>

namespace dart::gui::detail {
namespace {

class ImGuiContextGuard
{
public:
  ImGuiContextGuard()
  {
    ImGui::CreateContext();
  }

  ~ImGuiContextGuard()
  {
    ImGui::DestroyContext();
  }
};

TEST(ImGuiOverlayIndices, PreservesLargeDrawListVertexOffsets)
{
  ImGuiContextGuard context;
  ImGuiIO& io = ImGui::GetIO();
  configureImGuiOverlayRenderer(io);
  EXPECT_STREQ(io.BackendRendererName, "dart_gui_filament");
  EXPECT_NE(
      io.BackendFlags & ImGuiBackendFlags_RendererHasVtxOffset,
      ImGuiBackendFlags_None);

  io.DisplaySize = ImVec2(1280.0F, 720.0F);
  io.DeltaTime = 1.0F / 60.0F;
  io.Fonts->AddFontDefault();
  unsigned char* fontPixels = nullptr;
  int fontWidth = 0;
  int fontHeight = 0;
  io.Fonts->GetTexDataAsRGBA32(&fontPixels, &fontWidth, &fontHeight);
  ASSERT_NE(fontPixels, nullptr);
  ASSERT_GT(fontWidth, 0);
  ASSERT_GT(fontHeight, 0);

  ImGui::NewFrame();
  ImDrawList* const drawList = ImGui::GetForegroundDrawList();
  constexpr int rectangleCount = 20'000;
  for (int index = 0; index < rectangleCount; ++index) {
    const float x = static_cast<float>(index % 100);
    const float y = static_cast<float>(index / 100);
    drawList->AddRectFilled(
        ImVec2(x, y), ImVec2(x + 1.0F, y + 1.0F), IM_COL32_WHITE);
  }
  ImGui::Render();

  const ImDrawData* const drawData = ImGui::GetDrawData();
  ASSERT_NE(drawData, nullptr);
  ASSERT_GT(
      drawData->TotalVtxCount,
      static_cast<int>(std::numeric_limits<ImDrawIdx>::max()));

  bool hasNonzeroVertexOffset = false;
  for (int listIndex = 0; listIndex < drawData->CmdListsCount; ++listIndex) {
    for (const ImDrawCmd& command : drawData->CmdLists[listIndex]->CmdBuffer) {
      hasNonzeroVertexOffset |= command.VtxOffset != 0u;
    }
  }
  ASSERT_TRUE(hasNonzeroVertexOffset);

  const ImGuiOverlayDrawPlan plan
      = buildImGuiOverlayDrawPlan(*drawData, 1280u, 720u);
  ASSERT_EQ(
      plan.indices.size(), static_cast<std::size_t>(drawData->TotalIdxCount));
  ASSERT_FALSE(plan.indices.empty());
  ASSERT_FALSE(plan.commands.empty());
  for (const ImGuiOverlayDrawCommand& command : plan.commands) {
    EXPECT_GT(command.indexCount, 0u);
    EXPECT_LE(command.indexOffset + command.indexCount, plan.indices.size());
    EXPECT_GT(command.scissorWidth, 0u);
    EXPECT_GT(command.scissorHeight, 0u);
    EXPECT_LE(command.scissorLeft + command.scissorWidth, 1280u);
    EXPECT_LE(command.scissorBottom + command.scissorHeight, 720u);
  }
  const std::uint32_t maximumIndex
      = *std::max_element(plan.indices.begin(), plan.indices.end());
  EXPECT_GT(
      maximumIndex,
      static_cast<std::uint32_t>(std::numeric_limits<ImDrawIdx>::max()));
  EXPECT_LT(maximumIndex, static_cast<std::uint32_t>(drawData->TotalVtxCount));
}

TEST(ImGuiOverlayIndices, PreservesCommandClipRectangles)
{
  ImGuiContextGuard context;
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(1280.0F, 720.0F);
  // DART's GUI pipeline supplies DisplaySize, mouse positions, vertices, and
  // clip rectangles in framebuffer pixels. A GLFW content scale therefore
  // must not be applied to the scissors a second time.
  io.DisplayFramebufferScale = ImVec2(2.0F, 2.0F);
  io.DeltaTime = 1.0F / 60.0F;
  io.Fonts->AddFontDefault();
  unsigned char* fontPixels = nullptr;
  int fontWidth = 0;
  int fontHeight = 0;
  io.Fonts->GetTexDataAsRGBA32(&fontPixels, &fontWidth, &fontHeight);

  ImGui::NewFrame();
  ImDrawList* const drawList = ImGui::GetForegroundDrawList();
  drawList->PushClipRect(ImVec2(10.0F, 20.0F), ImVec2(110.0F, 220.0F), true);
  drawList->AddRectFilled(
      ImVec2(20.0F, 30.0F), ImVec2(100.0F, 210.0F), IM_COL32_WHITE);
  drawList->PopClipRect();
  drawList->PushClipRect(ImVec2(300.0F, 100.0F), ImVec2(500.0F, 250.0F), true);
  drawList->AddRectFilled(
      ImVec2(310.0F, 110.0F), ImVec2(490.0F, 240.0F), IM_COL32_WHITE);
  drawList->PopClipRect();
  drawList->PushClipRect(
      ImVec2(1400.0F, 800.0F), ImVec2(1500.0F, 900.0F), true);
  drawList->AddRectFilled(
      ImVec2(1410.0F, 810.0F), ImVec2(1490.0F, 890.0F), IM_COL32_WHITE);
  drawList->PopClipRect();
  ImGui::Render();

  const ImDrawData* const drawData = ImGui::GetDrawData();
  ASSERT_NE(drawData, nullptr);
  const ImGuiOverlayDrawPlan plan
      = buildImGuiOverlayDrawPlan(*drawData, 1280u, 720u);
  ASSERT_EQ(plan.commands.size(), 2u);
  ASSERT_EQ(plan.indices.size(), 12u);

  EXPECT_EQ(plan.commands[0].indexOffset, 0u);
  EXPECT_EQ(plan.commands[0].indexCount, 6u);
  EXPECT_EQ(plan.commands[0].scissorLeft, 10u);
  EXPECT_EQ(plan.commands[0].scissorBottom, 500u);
  EXPECT_EQ(plan.commands[0].scissorWidth, 100u);
  EXPECT_EQ(plan.commands[0].scissorHeight, 200u);

  EXPECT_EQ(plan.commands[1].indexOffset, 6u);
  EXPECT_EQ(plan.commands[1].indexCount, 6u);
  EXPECT_EQ(plan.commands[1].scissorLeft, 300u);
  EXPECT_EQ(plan.commands[1].scissorBottom, 470u);
  EXPECT_EQ(plan.commands[1].scissorWidth, 200u);
  EXPECT_EQ(plan.commands[1].scissorHeight, 150u);
}

} // namespace
} // namespace dart::gui::detail
