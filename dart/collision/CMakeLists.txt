# Search all header and source files
dart_add_subsystem_library(
  NAME collision
  TARGET_DIR "dart/collision/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADER_EXCLUDE SmartPointer.hpp
  HEADER_EXTRAS dart/All.hpp fcl/All.hpp
  EXTRA_HEADER_GLOBS
    "dart/*.hpp" "fcl/*.hpp" "ode/*.hpp" "ode/detail/*.hpp" "bullet/*.hpp" "bullet/detail/*.hpp"
  SUB_DIRS dart fcl ode bullet
)
