# Filament GUI Replacement - Visual Quality Requirements

## Quality target

The Filament path should only be promoted if it can make DART's built-in viewer
high quality for interactive robotics and physics debugging.

This target is about DART's maintained built-in visualization workflow:

- clear scene readability
- physically plausible lighting and materials
- robust shadows
- high-quality meshes and textures
- fast interactive camera/navigation
- debug overlays for physics state
- deterministic screenshot and smoke-test output

It is not a commitment to reproduce a full production rendering or content
pipeline inside DART. If reviewers require that scope, Filament should not be
promoted without revisiting the renderer choice and project scope.

## Capability conclusion

Filament covers the rendering features DART needs for a high-quality built-in
visual-debugging viewer: real-time PBR materials, physically based lighting,
directional/point/spot shadows, cascaded and soft shadow modes, contact
shadows, tone mapping, anti-aliasing, image-based lighting, glTF-oriented asset
tooling, and framebuffer readback. The remaining question is whether DART can
package and integrate those capabilities cleanly, not whether the renderer has
the core features.

## Non-negotiable features

The following must work before first-class promotion:

| Area            | Required capability                                                                           | Filament capability to use                                                                                 | DART validation gate                                                                                                            |
| --------------- | --------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| Shadows         | Dynamic objects cast and receive stable shadows from the default lights.                      | Directional, point, and spot light shadows; cascaded shadows; PCF/DPCF/PCSS/EVSM options; contact shadows. | A fixture with ground plus moving bodies renders visible stable shadows and passes a screenshot luminance-contrast smoke check. |
| PBR materials   | Default materials look better than flat/debug shading and preserve DART visual aspect colors. | Cook-Torrance PBR, metallic/roughness workflow, clear coat, normal and ambient-occlusion maps.             | Primitive and mesh examples show color, roughness, normals, and texture.                                                        |
| Lighting        | Scenes are readable without manual setup and allow higher-quality authored lighting.          | Physical light units, directional/point/spot lights, image-based lighting, HDR/linear lighting.            | Default lighting works; optional IBL improves mesh-heavy scenes.                                                                |
| Camera/exposure | Camera output is stable across common scene scales and produces inspectable screenshots.      | Physically based camera, exposure controls, tone mapping, color management.                                | Default camera frames small and large DART scenes without washout.                                                              |
| Anti-aliasing   | Edges and debug lines are clean enough for screenshots and demos.                             | TAA, FXAA, MSAA, specular anti-aliasing.                                                                   | Screenshot fixture does not show unacceptable jagged edges.                                                                     |
| Ambient detail  | Contact-rich scenes have readable depth cues beyond direct lighting.                          | Screen-space ambient occlusion and contact shadows.                                                        | Stacked bodies and robot-foot scenes show clear contact/depth cues.                                                             |
| Meshes/assets   | Robot and environment meshes load with useful material fidelity.                              | glTF 2.0 support through Filament tooling plus DART's existing mesh loading where appropriate.             | At least one textured robot/environment mesh renders correctly.                                                                 |
| Transparency    | Transparent or semi-transparent debug visuals remain useful.                                  | Transparent materials and transparent shadows where needed.                                                | Collision/limit/selection overlays remain legible with depth.                                                                   |
| Debug overlays  | Physics debugging remains convenient instead of visual polish hiding simulation state.        | DART-owned debug primitives rendered through Filament materials/buffers.                                   | Contacts, frames, forces, COM, selection, and grids are clear.                                                                  |
| Capture/testing | Visual quality regressions are detectable without manual demos.                               | Screenshot readback from windowed or headless swap-chain runs.                                             | CI or local smoke fixtures check nonblank shadowed screenshots plus scene-region luminance spread.                              |

## Promotion rule

Do not promote Filament to the primary `dart::gui` implementation unless:

1. The shadow gate passes.
2. The default scene quality is clearly better than the current OSG and Raylib
   paths.
3. Maintainers judge the built-in DART workflow to meet the visual-quality bar
   for interactive visualization/debugging.
4. Any remaining gap against broader production-rendering workflows is
   documented as an intentional scope boundary, not ignored.

## MVP visual fixture

The MVP should include one deterministic visual fixture that can be reused as a
smoke test:

- ground plane
- two or more dynamic bodies at different heights
- at least one TriMesh or textured primitive
- grid, world-frame, contact, normal, and force-vector debug overlays
- directional light with shadows enabled
- optional contact shadows or SSAO
- orbit camera preset
- `--frames <n>` mode
- screenshot output once capture exists

The first MVP can use a procedural texture to prove the material path, but
promotion requires captured evidence with at least one real robot or environment
asset using authored mesh/material data.

Current progress: the MVP now includes an imported textured WAM Collada mesh,
the full WAM URDF skeleton, a required Atlas DAE torso mesh, and a full Atlas
SDF robot fixture. It also uses the multi-material glTF PBR asset as a required
four-panel environment layout, carrying authored mesh UVs, submesh material
ranges, material colors, typed PBR texture paths, emissive colors,
metallic/roughness factors, and typed PNG/JPEG texture images into Filament
mesh rendering. The textured material consumes base color, metallic, roughness,
combined metallic-roughness, normal, occlusion, and emissive maps with explicit
fallback samplers for partially-authored assets, using anisotropic sampling for
inspection views. The default scene now combines PCSS/cascaded SUN shadows,
contact shadows, a neutral skybox, spherical harmonics indirect lighting, and
high-quality color grading. Debug lines for grid, world/body frames,
center-of-mass markers, contacts, normals, and force vectors are generated by
`dart-gui-experimental` and rendered by the Filament example. Checked-in glTF
PBR fixtures now validate authored PBR texture slots, alpha-bearing material
factors, UV metadata, and single-/multi-material submesh ranges through the real
Assimp importer without requiring a graphics context. The smoke scene also loads
those fixtures and routes alpha-bearing solid, textured, and mesh visual paths
through transparent lit Filament materials. The headless CTest smoke now checks
the rendered fixture for dark, mid-tone, and bright pixel populations plus
luminance spread, so shadow and lighting regressions fail even when the frame is
merely nonblank. Broader human visual review with larger authored environment
and PBR assets beyond the current WAM, Atlas, and PBR panel fixtures is still
required before this gate can be considered complete.
