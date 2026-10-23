Section: Breaking Changes

  * Remove the installed `dart/utils/urdf/IncludeUrdf.hpp` and
    `dart/utils/urdf/urdf_world_parser.hpp` headers, including
    `dart::utils::urdf_parsing::parseWorldURDF`. Use
    `dart::utils::DartLoader::parseWorld` or `parseWorldString` instead.
    Shared DART consumers no longer need urdfdom development packages;
    static consumers and DART builds still require urdfdom.
    `dart/utils/urdf/urdf.hpp` no longer includes `BackwardCompatibility.hpp`;
    code using its `URDFDOM_HEADERS_VERSION_*` macros must include
    `dart/utils/urdf/BackwardCompatibility.hpp` directly.
    [#3888](https://github.com/dartsim/dart/pull/3888),
    [#3884](https://github.com/dartsim/dart/pull/3884)
