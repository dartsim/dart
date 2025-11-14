import sys

import dartpy_nb

# Alias dartpy imports to the nanobind module so copied tests run unmodified.
sys.modules.setdefault("dartpy", dartpy_nb)
