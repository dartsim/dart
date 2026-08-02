# Deterministic 101-stone MuJoCo comparison geometry

The optional MuJoCo comparison generates its 101 masonry stones in memory from
the source parameters instead of checking 101 mechanical OBJ files and an
expanded MJCF file into the repository. The implementation lives in
`tests/benchmark/integration/fbf_paper_mujoco_baseline.py`; this directory keeps
only the source license and this provenance record.

The geometry is source-bound to the public FBF repository,
<https://github.com/matthcsong/fbf-sca-2026>, commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, path
`meshes/arch/num_stones=101/`. That repository credits the
[Rigid-IPC dataset](https://github.com/ipc-sim/rigid-ipc). The independent
upstream pin remains Rigid-IPC commit
`23b6ba6fbf8434056444ae106356fd2209136988` ("Write GLTF in input orientation",
2025-06-13). Rigid-IPC implements Ferguson et al., "Intersection-free Rigid
Body Dynamics" (SIGGRAPH 2021). Its MIT license is retained in
[`LICENSE.md`](LICENSE.md).

## Reproduction contract

The dependency-free generator mirrors the weighted-catenary parameters and
operations used by the source:

- `fc=60 cm`, `Qb=100 cm^2`, `Qt=49 cm^2`, and `L=30 cm`;
- composite Simpson integration and equal-arc-length bisection;
- constant per-stone square cross sections, source offsets, springer
  flattening, and the `0.1 cm` height normalization;
- source OBJ vertex order, twelve face triangles, and six-decimal coordinate
  quantization; and
- the source y/z-to-MuJoCo axis rotation, `0.01` cm-to-m scale, uniform `0.5`
  friction, `0.005 s` timestep, and `9.8 m/s^2` gravity.

The ordered 2,424-coordinate inventory is pinned by FNV-1a64
`0x528596c9206aef89`, the same digest used by the DART Figure 8 construction
test. Before removing the vendored files, an independent audit proved:

- all 101 generated OBJ byte streams exactly matched the pinned copies;
- file-backed and generated MuJoCo models had identical dimensions, masses,
  inertias, initial positions, mesh vertices, and mesh faces under MuJoCo
  3.11.0; and
- their first five simulation steps were bit-identical in `qpos` and `qvel`.

Unit tests keep the inventory digest, representative vertices, self-contained
MJCF structure, and absence of file-backed mesh references fail-closed.

## Scope and limitations

This is a benchmark/example-only comparison path. No core DART target reads it
and it adds no core dependency. The generated MJCF preserves Rigid-IPC's MuJoCo
port semantics; it is not a MuJoCo scene authored by the FBF paper and does not
establish historical paper parity. Density remains MuJoCo's per-mesh default
(`1000 kg/m^3`), matching Rigid-IPC's unspecified-density default. All 101
stones remain dynamic and only the ground is fixed, matching the source JSON
but differing from DART's current Figure 8 adapter, which fixes both springers.
