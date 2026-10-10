"""Native dispatch helpers for both binders; never starts a viewer loop."""
import dartpy

if getattr(dartpy, "_binder", None) == "nanobind":
    probe = dartpy._probe
else:
    import _dartpy_gui_probe as probe
