# Resume: LCP Epic

> **For AI Agents**: Read this entire file before taking any action.

## Quick Status

- **Branch**: `refactor/lcp_plan`
- **PR**: #2464
- **State**: Updated ImGui scaling logic, needs visual verification
- **Blocker**: Human visual test needed to confirm lcp_physics is crisp at scale 2

---

## What Was Done (This Session)

### 1. Disable FontGlobalScale When setFontScale Is Active

`applyImGuiScale()` now skips `FontGlobalScale` if `setFontScale()` was used,
avoiding bitmap scaling when ImGui is rebuilt at the target size.

**Files changed**:

- `dart/gui/ImGuiHandler.cpp` - Track opt-out flag and preserve FontGlobalScale

---

## Files Ready to Commit

```
M  dart/gui/ImGuiHandler.cpp
M  docs/dev_tasks/lcp_epic/README.md
M  docs/dev_tasks/lcp_epic/RESUME.md
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
git commit -m "fix(gui): avoid FontGlobalScale when using setFontScale

Skip FontGlobalScale in applyImGuiScale once setFontScale opts in to
rebuilt fonts, preventing bitmap scaling blur at --gui-scale > 1."

git push origin refactor/lcp_plan
```

**If text is still BLURRY** (needs debugging):

1. Check that `mFramebufferScale` is being detected correctly
2. Verify the font atlas is actually being rebuilt (add logging)
3. Test with explicit pixel sizes

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
