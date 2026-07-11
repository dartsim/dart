# FBF GUI Capture Report

This report records the current `dart-demos` visual evidence for the DART 6.20
exact-Coulomb FBF paper scenes. It is a GUI/snapshot smoke, not paper figure
parity.

## Scope

Implemented evidence:

- all nine `fbf_paper_*` scenes instantiate through the consolidated
  `dart-demos` app,
- the shared `dart-demos` host renders each scene's documentation metadata in
  the `Scene` tab before custom controls,
- each capture shows the `Scene` tab with the scene overview, expected result,
  and coverage limitation text visible,
- a one-step off-screen PNG capture exists for each scene,
- an action-aware off-screen PNG capture exists for the reduced 26-card
  projectile launch scaffold,
- an action-aware off-screen PNG capture exists for the reduced 25-stone arch
  crown-projectile scaffold,
- a contact sheet summarizes the current visual state.

Still missing:

- paper-matched camera framing and figure-by-figure snapshots,
- long-run 26-card settle/projectile snapshots,
- full-contact 25/101-stone arch snapshots,
- dynamic 10-level exact-FBF card-house snapshots,
- external baseline snapshots from Kamino, MuJoCo, or the paper implementation.

## Commands

The captures were produced on a host with `DISPLAY=:0`. Use `xvfb-run` on a
headless host with no X server. A single scene can be captured through the
Pixi task, with the final argument selecting the deterministic step count:

```bash
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 1
pixi run image-verdict /tmp/fbf_paper_backspin.png
```

After the host-level Scene-tab metadata renderer was added, this fresh focused
smoke also passed and showed the overview, expected result, and coverage text
visible beside the rendered backspin scene:

```bash
pixi run capture fbf_paper_backspin /tmp/fbf_paper_backspin.png 640 480 5
pixi run image-verdict /tmp/fbf_paper_backspin.png
```

For scene states exposed through key actions, use the action-aware capture
task. The 26-card scene binds `p` to the reduced four-projectile launch
scaffold:

```bash
pixi run capture-action fbf_paper_card_house_26 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png 1280 720 0
pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures/fbf_paper_card_house_26_projectiles.png
```

That action capture passed the default non-blank verdict. The contrast check
is recorded in the JSON and remains weak, but contrast is not required by the
default gate.

The 25-stone arch scene also binds `p` to a reduced projectile aimed through
the crown:

```bash
pixi run capture-action fbf_paper_masonry_arch_25 p docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png 1280 720 0
pixi run image-verdict docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_paper_masonry_arch_25_projectile.png
```

That action capture passed the default non-blank verdict. The refreshed image
shows the projectile at the crown, the Scene-tab overview/expected/coverage
text, and the shared diagnostics widget reporting `not run yet` before the
first exact solve.

The full batch below uses the already-built executable inside the Pixi shell so
all scenes share one build and the ordered contact sheet matches the table:

```bash
pixi run bash -lc '
set -euo pipefail
out=docs/dev_tasks/fbf_exact_coulomb_friction/assets/gui_captures
mkdir -p "$out"
scenes=(
  fbf_paper_incline
  fbf_paper_backspin
  fbf_paper_turntable
  fbf_paper_painleve
  fbf_paper_card_aframe
  fbf_paper_card_house_26
  fbf_paper_card_house_10
  fbf_paper_masonry_arch_25
  fbf_paper_masonry_arch_101
)
inputs=()
for scene in "${scenes[@]}"
do
  ./build/default/cpp/Release/bin/dart-demos \
    --scene "$scene" \
    --headless \
    --shot "$out/${scene}.png" \
    --steps 1 \
    --width 640 \
    --height 480
  python scripts/image_verdict.py "$out/${scene}.png" \
    > "$out/${scene}.verdict.json"
  inputs+=("$out/${scene}.png")
done
python scripts/image_sheet.py "${inputs[@]}" \
  --out docs/dev_tasks/fbf_exact_coulomb_friction/assets/fbf_gui_capture_sheet.png \
  --grid 3x3
'
```

## Contact Sheet

[FBF GUI capture sheet](assets/fbf_gui_capture_sheet.png)

| Label | Scene ID | Status |
| ---: | --- | --- |
| 0 | `fbf_paper_incline` | Non-blank capture, one-step visual smoke |
| 1 | `fbf_paper_backspin` | Non-blank capture, one-step visual smoke |
| 2 | `fbf_paper_turntable` | Non-blank capture, one-step visual smoke |
| 3 | `fbf_paper_painleve` | Non-blank capture, one-step proxy-scene smoke |
| 4 | `fbf_paper_card_aframe` | Non-blank capture, two-card precursor smoke |
| 5 | `fbf_paper_card_house_26` | Non-blank capture, reduced-contact scaffold smoke |
| 6 | `fbf_paper_card_house_10` | Non-blank capture, construction-only scaffold smoke |
| 7 | `fbf_paper_masonry_arch_25` | Non-blank capture, reduced-contact scaffold smoke |
| 8 | `fbf_paper_masonry_arch_101` | Non-blank capture, reduced-contact scaffold smoke |

## Verdict Summary

All nine captures passed the repo's default `image_verdict.py` gate. The
default gate requires non-blank output; it reports contrast diagnostics but does
not require contrast unless `--require-contrast` is supplied.

| Scene ID | Verdict | Non-blank |
| --- | --- | --- |
| `fbf_paper_incline` | Pass | Pass |
| `fbf_paper_backspin` | Pass | Pass |
| `fbf_paper_turntable` | Pass | Pass |
| `fbf_paper_painleve` | Pass | Pass |
| `fbf_paper_card_aframe` | Pass | Pass |
| `fbf_paper_card_house_26` | Pass | Pass |
| `fbf_paper_card_house_10` | Pass | Pass |
| `fbf_paper_masonry_arch_25` | Pass | Pass |
| `fbf_paper_masonry_arch_101` | Pass | Pass |

## Interpretation

The current GUI examples are self-contained enough to inspect the implemented
DART-side fixtures from the app: the rendered world, solver diagnostics, and
`Scene` explanation are visible in a single capture. This does not close the
paper GUI gap. Full paper parity still needs paper-matched snapshots, long-run
physical outcome captures, and external baseline images where those
implementations are available.

The extra
[26-card projectile action capture](assets/gui_captures/fbf_paper_card_house_26_projectiles.png)
shows the four launched projectile spheres beside the reduced card-house
scaffold with the `Scene` explanation visible. This proves the GUI control and
headless action-capture path are reproducible, but it is not a paper-matched
post-impact snapshot.

The extra
[25-stone arch projectile action capture](assets/fbf_paper_masonry_arch_25_projectile.png)
shows the reduced projectile at the crown with the self-contained `Scene`
explanation visible. This proves the GUI control, key action, capture path, and
pre-solve diagnostics are usable for the reduced Fig. 7 scaffold, but it is not
a paper-matched post-impact arch snapshot.
