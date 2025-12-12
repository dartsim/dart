Minimal example using DART's experimental VulkanSceneGraph (VSG) GUI backend
(`dart-gui-vsg`).

## pixi

Build + run the example:

    $ pixi run ex vsg_hello_world

By default the executable exits without opening a window (useful for CI/headless
setups). To create a window:

    $ DART_VSG_RUN=1 pixi run ex vsg_hello_world

Optionally run a fixed number of frames (to avoid an infinite render loop):

    $ DART_VSG_RUN=1 DART_VSG_FRAMES=60 pixi run ex vsg_hello_world

## Notes

- A window requires a working display (`DISPLAY` or `WAYLAND_DISPLAY`) and a
  Vulkan ICD.
- Some VSG builds are compiled without GLSLang shader compiler support. If you
  see a shader-compiler error, rebuild/install VSG with
  `VSG_SUPPORTS_ShaderCompiler=1`, or switch to a precompiled-shader setup.
