# Patch: Add null pointer validation to ImGui font loading functions
# Fixes: https://github.com/dartsim/dart/issues/2516
#
# ImGui v1.84.2 does not validate input pointers in AddFontFromMemory*
# functions, causing SEGV when nullptr is passed (e.g., from a failed
# resource load). This patch adds early-return guards.
#
# Usage: cmake -DIMGUI_SOURCE_DIR=<path> -P imgui_null_font_guard.cmake

if(NOT DEFINED IMGUI_SOURCE_DIR)
  message(FATAL_ERROR "IMGUI_SOURCE_DIR must be set")
endif()

set(file "${IMGUI_SOURCE_DIR}/imgui_draw.cpp")

if(NOT EXISTS "${file}")
  message(FATAL_ERROR "imgui_draw.cpp not found at ${file}")
endif()

file(READ "${file}" content)

# Guard: only patch once (idempotent)
string(FIND "${content}" "DART patch" already_patched)
if(NOT already_patched EQUAL -1)
  message(STATUS "imgui_draw.cpp already patched, skipping")
  return()
endif()

# Patch AddFontFromMemoryTTF
string(REPLACE
  "IM_ASSERT(!Locked && \"Cannot modify a locked ImFontAtlas between NewFrame() and EndFrame/Render()!\");
    ImFontConfig font_cfg = font_cfg_template ? *font_cfg_template : ImFontConfig();
    IM_ASSERT(font_cfg.FontData == NULL);
    font_cfg.FontData = ttf_data;"
  "IM_ASSERT(!Locked && \"Cannot modify a locked ImFontAtlas between NewFrame() and EndFrame/Render()!\");
    // [DART patch] Guard against null/empty input (issue #2516)
    if (ttf_data == NULL || ttf_size <= 0)
        return NULL;
    ImFontConfig font_cfg = font_cfg_template ? *font_cfg_template : ImFontConfig();
    IM_ASSERT(font_cfg.FontData == NULL);
    font_cfg.FontData = ttf_data;"
  content "${content}"
)

# Patch AddFontFromMemoryCompressedTTF
string(REPLACE
  "const unsigned int buf_decompressed_size = stb_decompress_length((const unsigned char*)compressed_ttf_data);"
  "// [DART patch] Guard against null/empty input to prevent SEGV in stb_decompress (issue #2516)
    if (compressed_ttf_data == NULL || compressed_ttf_size <= 0)
        return NULL;

    const unsigned int buf_decompressed_size = stb_decompress_length((const unsigned char*)compressed_ttf_data);"
  content "${content}"
)

# Patch AddFontFromMemoryCompressedBase85TTF
string(REPLACE
  "int compressed_ttf_size = (((int)strlen(compressed_ttf_data_base85) + 4) / 5) * 4;"
  "// [DART patch] Guard against null input (issue #2516)
    if (compressed_ttf_data_base85 == NULL)
        return NULL;
    int compressed_ttf_size = (((int)strlen(compressed_ttf_data_base85) + 4) / 5) * 4;"
  content "${content}"
)

# Verify all three patches were applied (detect source drift from ImGui updates)
string(FIND "${content}" "DART patch" _patch_found)
if(_patch_found EQUAL -1)
  message(FATAL_ERROR
    "ImGui null-pointer patch failed: none of the expected patterns were found "
    "in ${file}. The ImGui source may have changed. Update the patch patterns "
    "to match the current ImGui version."
  )
endif()

file(WRITE "${file}" "${content}")
message(STATUS "Patched imgui_draw.cpp with null pointer guards (issue #2516)")
