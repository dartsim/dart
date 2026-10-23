# IO And Model Loading

Model loading and parser changes can affect downstream users and Gazebo
compatibility. For dependency cleanup, treat parser dependencies and installed
headers as compatibility surfaces.

Run focused IO tests when touching parser code, model-loading dependencies, or
package metadata used by IO components.

## SDF Loading

`SdfParser` uses sdformat for schema validation, version conversion and frame
resolution. SDF support in `dart-io` is optional and requires sdformat 14.8
or newer; supported library majors are 14, 15 and 16. Missing sdformat warns
and builds IO without SDF, leaving URDF, SKEL and MJCF available. Set
`DART_SKIP_SDFORMAT=ON` to disable SDF deliberately. Standalone builds do not
fetch sdformat. Consumers requiring SDF should request the `io-sdf` CMake
component, which is not found on an SDF-off install. `HAVE_SDFORMAT` in
`dart/config.hpp` identifies SDF support. Shared consumers do not need
sdformat development files; static consumers require the selected major.
On an SDF-off build, Python `SdfParser.readWorld()` and `readSkeleton()` raise
`RuntimeError` explaining how to enable SDF. The accepted document versions
on SDF-on builds follow the selected library.
Standard Pixi environments use sdformat 16.1 or newer within major 16 and
urdfdom 6. SDFormat 16 [shares schema descriptions](https://github.com/gazebosim/sdformat/pull/1589),
avoiding repeated schema copies when loading large worlds. The Ionic
environment uses sdformat 16 for DART and 15 for Gazebo; the isolated Harmonic
and Jetty lanes use 14 and 16 respectively.
SDF-on builds retain the public `dart::utils::SdfParser` entrypoints and options.
Authored world gravity, physics timestep and joint limits, including effort and
velocity, are honored. Existing controllers that relied on ignored limits must
configure their intended limits explicitly after loading.

Named pose frames, joint endpoint frames, axes expressed in other frames,
world-level includes and merged includes use sdformat semantics. Inertia is
rotated into the link frame; inertial poses must use their owning link frame.
DART retains its soft-shape extensions, root-joint selection and plane-loading
option. General nested models, closed kinematic
loops, world-level joints, multiple incoming joints per link, mesh submesh
selection and unsupported geometry fail instead of loading incomplete models.

Malformed input, invalid numeric values, unresolved resources or frames, and
unsupported model structures produce diagnostics and return `nullptr` in C++
or `None` in Python. Check the returned value instead of relying on SDF pose
exceptions. Empty poses are identity transforms; finite nonzero quaternions are
normalized safely even at extreme magnitudes. Other XML parsers retain their
existing error contracts.

Includes use the caller's resource retriever and resolve relative to the source
document containing each include. Meshes resolve relative to their own source
document, including merged models. Private staging files are removed after
model construction or a loading failure; parser callbacks are local to each
load. Loading itself adds no network retriever or automatic download behavior.

The self-contained `data/sdf/test/pose_frames.world` and
`python/examples/sdf_pose/main.py` illustrate the local geometry and
inertia frames. Capture with:

```bash
pixi run agent-capture -- --factory python.examples.sdf_pose.main:make_world \
    --steps 0 --layers body_frames inertia_boxes labels --auto-views 1 \
    --out build/sdf-pose-evidence
```

## Online Robot Models

The optional `utils-assets` component supplies `ModelResourceRetriever` in C++
and dartpy. Normal Pixi builds enable it; native CMake builds opt in with
`-DDART_BUILD_UTILS_ASSETS=ON` and require libcurl 7.85 or newer and OpenSSL
Crypto 1.1.1 or newer at build time. Shared installs link them privately, so
consumers do not need their development packages, only their runtime
libraries; static installs also require the development packages from
consumers.
Existing components and parser defaults do not acquire networking dependencies.

The catalog and source pins live in [`data/robot_models`](../../data/robot_models/).
Manifests enumerate every model, mesh, texture, license, and metadata file with
an immutable HTTPS source URL, SHA-256 digest, and exact size. Review source
licenses and model semantics before changing a pin; a newer download alone
does not establish DART compatibility.

```bash
pixi run fetch-robot-assets
pixi run test-robot-assets
pixi run demos -- --scene atlas_v5
pixi run demos -- --scene unitree_g1
```

GUI scenes use the verified cache. The retriever itself can download on first
access when explicitly constructed online. Offline mode performs no network
requests. Cache identities include model ID, revision, and manifest content;
whole-directory publication preserves DART's mesh-relative texture loading.
Corruption fails with the affected cache directory; remove that exact directory
and prefetch again to repair it. Do not change cache contents while a loaded
scene is using them: the GUI renderer reads some textures directly from local
paths.

Verification pairs `test-robot-assets` topology/geometry/inertia/short-step
checks with inspected Filament captures. The default C++/Python retriever tests
use deterministic local fixtures rather than online vendor availability.

The Hubo puppet scene and installed `dart://sample/urdf/drchubo/` resources
have been retired. The Fetch demo and its noncommercial MJCF assets are
retired. Include, mesh and multi-root parser coverage uses the DART-authored
BSD fixture at
`dart://sample/mjcf/test/manipulation/scene.xml`. The former Roboti-derived
MJCF samples are replaced by pinned Apache-2.0
[DeepMind Control Suite models](../../data/mjcf/deepmind/README.md), for example
`dart://sample/mjcf/deepmind/cartpole.xml`. That README records source revisions,
hashes, adaptations and test mappings. Atlas v3 control and soft-foot examples
retain their tuned models. The DART-authored six-axis arm, WAM and synthetic
skeletons remain useful teaching or regression fixtures; model age alone is
not a removal criterion.
