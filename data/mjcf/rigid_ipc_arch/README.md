# Rigid-IPC 101-stone masonry arch (MuJoCo scene)

Source repository: <https://github.com/ipc-sim/rigid-ipc>, commit
`23b6ba6fbf8434056444ae106356fd2209136988` ("Write GLTF in input orientation",
2025-06-13). License: MIT, see [`LICENSE.md`](LICENSE.md) (Copyright (c) 2021
Zachary Ferguson and the IPC Simulation organization).

Rigid-IPC implements Ferguson et al., "Intersection-free Rigid Body
Dynamics" (SIGGRAPH 2021), and is the masonry-arch dataset credited by Song,
Fan, Ascher, and Pai's exact-Coulomb FBF paper (SCA 2026). The FBF paper's
own repository (`https://github.com/matthcsong/fbf-sca-2026`) returned
HTTP 404 on 2026-07-09 (checked before and after this extraction), so this is
the only available author-adjacent geometry source for the paper's Fig. 8
(101-stone arch) scene.

Files here are a benchmark/example-only data asset for
`tests/benchmark/integration/fbf_paper_mujoco_baseline.py`. They are not read
by any DART library target and add no core dependency; see
`docs/ai/principles.md` / `AGENTS.md` for the repository's compatibility-first
policy on this kind of optional external-comparison asset.

## Contents

- `arch-101-stones_mjc.xml`: adapted from
  `comparisons/MuJoCo/friction/arch/arch-101-stones_mjc.xml` in the upstream
  repository (a MuJoCo port Rigid-IPC ships itself, "reference only" per the
  extraction manifest).
- `plane.obj`: the flat ground-plane mesh (`meshes/plane.obj` upstream).
- `arch/num_stones=101/stone-01.obj` .. `stone-101.obj`: the 101 voussoir
  wedge meshes (`meshes/arch/num_stones=101/` upstream), 8 vertices / 12
  triangles each.

## Adaptations from the upstream XML (attribute-only, no vertex data changed)

1. `<mesh file="....stl">` -> `<mesh file="....obj">`. This tree does not
   carry a `.stl` mesh export step; we ship the `.obj` meshes the upstream
   repository also carries, and MuJoCo loads `.obj` natively.
2. Every `<mesh>` element gained a `0.01` per-axis scale factor (combined
   multiplicatively with the one pre-existing `plane_0` scale). The raw OBJ
   vertex coordinates are unitless numbers on the order of the paper's
   stated "cm" dimensions (roughly 0-65 for the 101-stone arch); taken
   literally as meters the stones would be tens of meters tall. This is
   exactly the cm -> m conversion recorded in
   [`PROVENANCE.txt`](../../../docs/dev_tasks/fbf_exact_coulomb_friction/PROVENANCE.txt),
   so the arch here is close to the
   ~0.6 m-tall scale the source generator's own parameters describe
   (`fc=60`, `Qb=100`, `Qt=49`, `L=30`, all in cm).
3. An explicit `<option timestep="0.005" gravity="0 0 -9.8" cone="elliptic"/>`
   was added. The upstream file has no `<option>` block and relies on
   whatever harness loads it; `9.8` matches Rigid-IPC's own gravity
   magnitude (source scene JSON uses `gravity: [0, -9.8, 0]`, Y-up).

Body/geom friction values (`0.5`, uniform), stone ordering, and the
axis-remapping `quat` on every body (Rigid-IPC is Y-up; this rotates the
scene into MuJoCo's Z-up convention, matching the upstream file) are
unmodified.

## Known limitations

- This is Rigid-IPC's own MuJoCo port of the scene, not a MuJoCo scene
  authored by the FBF paper itself (which remains unavailable; see above).
  Treat any comparison as "does an independent Coulomb-friction simulator
  handle this masonry-arch geometry the same way", not a byte-for-byte paper
  reproduction.
- Density is left at MuJoCo's per-mesh default (`1000 kg/m^3`), matching
  Rigid-IPC's own unspecified-density default
  (`src/io/read_rb_scene.cpp:68` upstream).
- All 101 stones are dynamic; only the ground plane is fixed, matching the
  source JSON (`is_dof_fixed` is only set on the ground body). This differs
  from DART's current approximate-arch test scaffold, which additionally
  pins the two endpoint stones as static; see
  `docs/dev_tasks/fbf_exact_coulomb_friction/PR_REPORT.md` for the DART-side
  arch fixture notes.
