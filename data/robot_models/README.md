# Robot model catalog

The XML manifests pin complete model bundles by immutable source revision,
SHA-256, and byte count. The model files are downloaded to a persistent user
cache; only this catalog is shipped with DART. License and provenance files
travel with every bundle.

| Model | Entrypoint | Structure | License |
| --- | --- | --- | --- |
| Hydraulic-era Atlas v5, without head | `atlas_v5_no_head.urdf` | 36 bodies, 29 actuated joints, 28 meshes and 5 textures | Apache-2.0 |
| Unitree G1, mode 15 | `g1_29dof_mode_15.urdf` | 38 bodies, 29 actuated joints and 34 meshes | BSD-3-Clause |

Both models have 29 degrees of freedom with a fixed root and 35 with a floating
root. These are model-inspection assets, without locomotion controllers or
hardware-fidelity guarantees. Atlas v5 is an update to the hydraulic Atlas
description, not a description of the current electric Atlas.

## Use

Normal Pixi environments enable the optional `utils-assets` component. Prefetch
the models before opening a GUI:

```bash
pixi run fetch-robot-assets                   # both models
pixi run fetch-robot-assets unitree-g1         # one model
pixi run fetch-robot-assets --offline          # verify the existing cache
pixi run demos -- --scene atlas_v5
pixi run py-demos -- --scene unitree_g1
```

The C++ scenes provide a joint slider and reset button, alongside the existing
body and geometry inspector. In Python, `[` and `]` select a joint, `-` and `=`
change its angle within the model limits, and `t` resets the pose. Both scenes
have a fixed base and disable physics simulation. Scene factories use the
verified cache offline, so switching scenes never starts a download.

`dart::utils::ModelResourceRetriever` and `dartpy.utils.ModelResourceRetriever`
accept `model://<id>/<revision>/<relative-file>` URIs. The revision is the full
commit recorded in the corresponding manifest. Retrieval verifies the complete
bundle on first acquisition and preserves relative mesh and texture paths.
Custom cache locations are available through the retriever constructor or the
prefetch command's `--cache` argument. Corrupt published bundles must be removed
explicitly before downloading a replacement.

For application usage, installed components, custom manifests, and verification
commands, see [IO and model loading](../../docs/onboarding/io-parsing.md).

## Qualification

After prefetching, run `pixi run test-robot-assets -- --offline`. This checks all
bundle hashes and sizes, fixed/floating topology, declared inertias and joint
limits, visual/collision geometry, mesh bounds, and 100 finite simulation steps
at a 1 ms timestep. The pinned Atlas has 31 visual and 28 collision shapes; G1
has 34 visual and 35 collision shapes. Short free simulation establishes
numerical loading sanity, without ground contact or locomotion validation.

Capture the actual cache-only demo worlds with core OSG debug layers:

```bash
pixi run agent-capture -- --factory examples.demos.scenes.modern_humanoids:atlas_world \
    --layers body_frames collision_bounds --auto-views 1 --out /tmp/atlas-model
pixi run agent-capture -- --factory examples.demos.scenes.modern_humanoids:g1_world \
    --layers body_frames collision_bounds --auto-views 1 --out /tmp/g1-model
```

On Linux these require a GLX display or Xvfb. Inspect the selected PNGs alongside
the text checks: complete robots, retained textures, aligned body frames and
collision bounds. Native DART 6 qualification passed for these pins, including
the C++/Python demos and installed static consumers. Hands are deferred, and
Windows/macOS runtime qualification remains for platform CI.

## Provenance and prior work

- Atlas uses DART's [adapted v5 description](https://github.com/dartsim/dart/tree/0d92c25f336db51049a9516de46886838e6ea596/data/sdf/atlas),
  introduced by [PR #2684](https://github.com/dartsim/dart/pull/2684), and the
  [pinned DRCSim mesh and texture package](https://github.com/Hurisa/drcsim/tree/a2a606ae475e682df1d5214e54de2cbd4f9b016f/atlas_description).
  The adapted URDF, package metadata, Apache license, and attribution README
  are preserved with the original mesh/texture layout.
- G1 uses Unitree's [pinned mode-15 description](https://github.com/unitreerobotics/unitree_ros/tree/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description).
  Its [variant table](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description/README.md)
  deprecates the older bare G1 description. The bundle includes the upstream
  README and repository BSD license.
- [PR #464](https://github.com/dartsim/dart/pull/464#issuecomment-126840292)
  proposed online model retrieval with caching and versioning.
  [PR #2138](https://github.com/dartsim/dart/pull/2138) added DART 7 HTTP
  retrieval and G1. This DART 6 catalog adds verified, versioned complete
  bundles while keeping existing parser defaults.
- [PR #972](https://github.com/dartsim/dart/pull/972) established the local
  filesystem path requirement for textures. Complete cached directories retain
  those paths for DART 6 rendering.

## Retained and retired samples

| Sample | Decision and purpose |
| --- | --- |
| Atlas v3 | Retained for the existing puppet and SIMBICON controllers, plus parser/regression fixtures. Those controllers depend on its joints and geometry. |
| DRC-Hubo | Puppet scene and unused `data/urdf/drchubo/` assets removed. Recent HuboLab research concerns new hardware; no maintained replacement for this full-body description was established. |
| Fetch | Demo removed following the research platform's official discontinuation. The OpenAI Gym MJCF assets remain as parser fixtures. |
| KR5 and WAM | Retained as actively consumed manipulation, IK, tutorial, and regression assets. |
| Synthetic worlds and historical MJCF fixtures | Retained for behavior and parser coverage rather than promoted as supported current hardware. |

Hubo community research continued in
[2023](https://link.springer.com/article/10.1007/s12555-023-0387-6), and
[HuboLab's 2025 work](https://news.kaist.ac.kr/newsen/html/news/?mode=V&mng_no=52351)
uses new hardware. The retirement applies to the unmaintained DRC-Hubo asset,
not to the research group. Fetch's
[official platform notice](https://fetchrobotics.github.io/docs/) states that
its research platform was discontinued and unsupported in 2024.

Standalone hand assets are deferred.
