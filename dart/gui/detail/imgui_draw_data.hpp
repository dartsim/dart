/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_GUI_DETAIL_IMGUI_DRAW_DATA_HPP_
#define DART_GUI_DETAIL_IMGUI_DRAW_DATA_HPP_

#include <imgui.h>

#include <algorithm>
#include <limits>
#include <vector>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace dart::gui::detail {

struct ImGuiOverlayDrawCommand
{
  std::size_t indexOffset = 0u;
  std::size_t indexCount = 0u;
  std::uint32_t scissorLeft = 0u;
  std::uint32_t scissorBottom = 0u;
  std::uint32_t scissorWidth = 0u;
  std::uint32_t scissorHeight = 0u;
};

struct ImGuiOverlayDrawPlan
{
  std::vector<std::uint32_t> indices;
  std::vector<ImGuiOverlayDrawCommand> commands;
};

/// Advertise the renderer capabilities consumed by Dear ImGui while building
/// draw lists. In particular, large panels need command-local vertex offsets
/// once a 16-bit ImDrawIdx draw list exceeds 64K vertices.
inline void configureImGuiOverlayRenderer(ImGuiIO& io)
{
  io.BackendRendererName = "dart_gui_filament";
  io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
}

/// Convert Dear ImGui command lists into the 32-bit index ranges and scissor
/// rectangles consumed by the Filament overlay. User callback commands and
/// commands clipped entirely outside the framebuffer are omitted.
inline ImGuiOverlayDrawPlan buildImGuiOverlayDrawPlan(
    const ImDrawData& drawData,
    std::uint32_t framebufferWidth,
    std::uint32_t framebufferHeight)
{
  ImGuiOverlayDrawPlan plan;
  plan.indices.reserve(static_cast<std::size_t>(drawData.TotalIdxCount));

  std::uint32_t vertexBase = 0u;
  for (int listIndex = 0; listIndex < drawData.CmdListsCount; ++listIndex) {
    const ImDrawList& commandList = *drawData.CmdLists[listIndex];
    for (const ImDrawCmd& command : commandList.CmdBuffer) {
      if (command.UserCallback != nullptr || command.ElemCount == 0u) {
        continue;
      }

      const float clipMinX = command.ClipRect.x - drawData.DisplayPos.x;
      const float clipMinY = command.ClipRect.y - drawData.DisplayPos.y;
      const float clipMaxX = command.ClipRect.z - drawData.DisplayPos.x;
      const float clipMaxY = command.ClipRect.w - drawData.DisplayPos.y;
      const float clampedLeft
          = std::clamp(clipMinX, 0.0F, static_cast<float>(framebufferWidth));
      const float clampedTop
          = std::clamp(clipMinY, 0.0F, static_cast<float>(framebufferHeight));
      const float clampedRight
          = std::clamp(clipMaxX, 0.0F, static_cast<float>(framebufferWidth));
      const float clampedBottom
          = std::clamp(clipMaxY, 0.0F, static_cast<float>(framebufferHeight));
      if (clampedRight <= clampedLeft || clampedBottom <= clampedTop) {
        continue;
      }

      const std::uint32_t left
          = static_cast<std::uint32_t>(std::floor(clampedLeft));
      const std::uint32_t top
          = static_cast<std::uint32_t>(std::floor(clampedTop));
      const std::uint32_t right
          = static_cast<std::uint32_t>(std::ceil(clampedRight));
      const std::uint32_t bottom
          = static_cast<std::uint32_t>(std::ceil(clampedBottom));
      ImGuiOverlayDrawCommand drawCommand;
      drawCommand.indexOffset = plan.indices.size();
      drawCommand.indexCount = static_cast<std::size_t>(command.ElemCount);
      drawCommand.scissorLeft = left;
      drawCommand.scissorBottom = framebufferHeight - bottom;
      drawCommand.scissorWidth = right - left;
      drawCommand.scissorHeight = bottom - top;

      for (unsigned int element = 0; element < command.ElemCount; ++element) {
        const std::size_t indexOffset
            = static_cast<std::size_t>(command.IdxOffset) + element;
        assert(
            indexOffset < static_cast<std::size_t>(commandList.IdxBuffer.Size));
        const std::uint32_t localIndex
            = command.VtxOffset
              + static_cast<std::uint32_t>(commandList.IdxBuffer[indexOffset]);
        assert(
            localIndex
            < static_cast<std::uint32_t>(commandList.VtxBuffer.Size));
        assert(
            vertexBase
            <= std::numeric_limits<std::uint32_t>::max() - localIndex);
        plan.indices.push_back(vertexBase + localIndex);
      }
      plan.commands.push_back(drawCommand);
    }
    assert(
        vertexBase
        <= std::numeric_limits<std::uint32_t>::max()
               - static_cast<std::uint32_t>(commandList.VtxBuffer.Size));
    vertexBase += static_cast<std::uint32_t>(commandList.VtxBuffer.Size);
  }
  return plan;
}

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_IMGUI_DRAW_DATA_HPP_
