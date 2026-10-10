# IO And Model Loading

Model loading and parser changes can affect downstream users and Gazebo
compatibility. For dependency cleanup, treat parser dependencies and installed
headers as compatibility surfaces.

Run focused IO tests when touching parser code, model-loading dependencies, or
package metadata used by IO components.

## Online Robot Models

The optional `utils-assets` component supplies `ModelResourceRetriever` in C++
and dartpy. Normal Pixi builds enable it; native CMake builds opt in with
`-DDART_BUILD_UTILS_ASSETS=ON` and require libcurl 7.85 or newer and OpenSSL
Crypto 1.1.1 or newer. Existing
components and parser defaults do not acquire networking dependencies.

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
whole-directory publication preserves DART 6 mesh-relative texture loading.
Corruption fails with the affected cache directory; remove that exact directory
and prefetch again to repair it. Do not change cache contents while a loaded
scene is using them: OSG reads some textures directly from local paths.

Verification pairs `test-robot-assets` topology/geometry/inertia/short-step
checks with inspected OSG captures. The default C++/Python retriever tests use
deterministic local fixtures rather than online vendor availability.

The Hubo puppet scene and installed `dart://sample/urdf/drchubo/` resources
have been retired. The Fetch demo is retired, while its model remains a parser
fixture. Atlas v3 control and soft-foot examples retain their tuned models.
KR5, WAM, synthetic skeletons, and historical MJCF files remain useful teaching
or regression fixtures; model age alone is not a removal criterion.
