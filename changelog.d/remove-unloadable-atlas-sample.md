Section: Examples

  * Remove the `dart://sample/sdf/atlas/atlas_v3.urdf` sample. It never
    loaded: its neck joint named a `head` link the file did not define. Use
    `atlas_v3_no_head.urdf` or `atlas_v3_no_head.sdf` from the same directory.
    [#3921](https://github.com/dartsim/dart/pull/3921)
