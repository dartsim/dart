# Resume: LCP Epic

> **For AI Agents**: Read this entire file before taking any action.

## Quick Status

- **Branch**: `refactor/lcp_plan`
- **PR**: #2464
- **State**: Scaled lcp_physics widget layout to font size, needs visual verification
- **Blocker**: Human visual test needed to confirm lcp_physics is crisp at scale 2

---

## What Was Done (This Session)

### 1. Disable FontGlobalScale When setFontScale Is Active

`applyImGuiScale()` now skips `FontGlobalScale` if `setFontScale()` was used,
avoiding bitmap scaling when ImGui is rebuilt at the target size.

**Files changed**:

- `dart/gui/ImGuiHandler.cpp` - Track opt-out flag and preserve FontGlobalScale

### 2. Normalize Framebuffer Scale When Viewport/Traits Are Flipped

`newFrame()` now treats the larger of viewport/traits as framebuffer size when
the ratio falls below 1, avoiding downscaling in ImGui on some OSG setups.

**Files changed**:

- `dart/gui/ImGuiHandler.cpp` - Scale fallback for viewport vs traits

### 3. Scale lcp_physics Widget Layout with Font Size

Fixed-size ImGui widget measurements (panel size, button width, plot size,
wrap width) now scale from the current font size to avoid cramped or broken
layout at `--gui-scale > 1`.

**Files changed**:

- `examples/lcp_physics/main.cpp` - Apply font-based UI scaling

---

## Files Ready to Commit

```
M  CHANGELOG.md
M  docs/dev_tasks/lcp_epic/README.md
M  docs/dev_tasks/lcp_epic/RESUME.md
M  examples/lcp_physics/main.cpp
```

---

## Next Steps

### Step 1: Visual Test (Requires Human)

```bash
cd /home/js/dev/dartsim/dart/task_4

# Test any example at high DPI
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2
./build/default/cpp/Release/bin/imgui --gui-scale 2
./build/default/cpp/Release/bin/box_stacking --gui-scale 2
```

**Check**: Is the ImGui text crisp or blurry at scale 2?

### Step 2: Based on Visual Test Result

**If text is CRISP** (fix works):

```bash
git add -A
git commit -m "fix(examples): scale lcp_physics widget layout with font size

Scale the fixed ImGui measurements (panel size, button widths, plot size,
wrap width) from the current font size to prevent cramped layout at
--gui-scale > 1."

git push origin refactor/lcp_plan
```

**If text is still BLURRY** (needs debugging):

1. Check that `mFramebufferScale` is being detected correctly
2. Verify the font atlas is actually being rebuilt (add logging)
3. Log viewport vs traits sizes to confirm the ratio direction
4. Compare `ImGui::GetFontSize()` vs expected gui-scale in lcp_physics

---

## Technical Details

### How setFontScale() Works

```cpp
// In setFontScale():
mFontScale = scale;
mUseFontScale = true;
mFontScaleDirty = true;

// In newFrame():
if (mUseFontScale && (mFontScaleDirty || scaleChanged)) {
    rebuildDefaultFont(mFontScale, framebufferScale);
    mFontScaleDirty = false;
}

// In rebuildDefaultFont():
config.SizePixels = 13.0f * fontScale * framebufferScale;
io.FontGlobalScale = 1.0f / framebufferScale;
```

### How applyImGuiScale() Avoids Blur

```cpp
// In setFontScale():
gDisableFontGlobalScale = true;

// In applyImGuiScale():
if (!gDisableFontGlobalScale) {
  io.FontGlobalScale = scale;
}
```

### How Framebuffer Scale Is Normalized

```cpp
float scaleX = viewportWidth / traitsWidth;
float scaleY = viewportHeight / traitsHeight;
if (scaleX < 1.0f || scaleY < 1.0f) {
  scaleX = traitsWidth / viewportWidth;
  scaleY = traitsHeight / viewportHeight;
  displaySize = ImVec2(viewportWidth, viewportHeight);
}
```

### How the lcp_physics Widget Scales Layout

```cpp
const float uiScale = ImGui::GetFontSize() / 13.0f;
ImGui::SetNextWindowSize(ImVec2(340.0f * uiScale, 600.0f * uiScale), ...);
ImGui::Button("Play", ImVec2(70.0f * uiScale, 0.0f));
```

### Why FontGlobalScale Alone Causes Blur

ImGui's default font is a 13px bitmap. When you set `FontGlobalScale = 2.0`:

1. Renders the 13px bitmap
2. Scales it up 2x at display time
3. Result: 26px but blurry (upscaled bitmap)

With `setFontScale()`:

1. Loads a 26px font at initialization
2. Renders natively at 26px
3. Result: 26px and crisp (native resolution)

---

## Clean Up Corrupted Frame Files

If you see files with non-ASCII names in git status:

```bash
find . -maxdepth 1 -type f -name $'*[\x80-\xff]*' -delete
```

---

## How to Fully Resume

```bash
cd /home/js/dev/dartsim/dart/task_4
git checkout refactor/lcp_plan
git status  # Should show all files above

# Build (may already be built)
pixi run build

# Visual test
./build/default/cpp/Release/bin/lcp_physics --gui-scale 2

# If crisp: commit and push (see commands above)
```

---

## Previous Session Work (Already Committed)

| Commit        | Description                                         |
| ------------- | --------------------------------------------------- |
| `4639cd41e5b` | Load fonts at scaled size (per-example, superseded) |
| `6ee024c0f6c` | Fix GUI scaling and slip compliance warnings        |
| `bc4e6e7a8e7` | Add ImGui widget to lcp_physics                     |
| `c9c630b0464` | Remove lcp_solvers example (moved to benchmarks)    |

---

## Phase 4 Future Work (Not Started)

- Additional LCP solvers (APGD, TGS, ADMM)
- Runtime solver switching
- Benchmark infrastructure
- Documentation expansion

---

**Last Updated**: 2026-01-19 13:50 PST
