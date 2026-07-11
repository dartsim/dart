# Copyright (c) The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

dart_benchmarks(
  TYPE BM_INTEGRATION
  SOURCES
    bm_empty.cpp
)

dart_benchmarks(
  TYPE BM_INTEGRATION
  SOURCES
    bm_inverse_dynamics.cpp
  LINK_LIBRARIES
    dart
)

dart_benchmarks(
  TYPE BM_INTEGRATION
  SOURCES
    bm_exact_coulomb_fbf_paper.cpp
  LINK_LIBRARIES
    dart
)

add_executable(fbf_paper_trace fbf_paper_trace.cpp)
target_link_libraries(fbf_paper_trace PRIVATE dart)

add_executable(fbf_paper_gamma_sweep fbf_paper_gamma_sweep.cpp)
target_link_libraries(fbf_paper_gamma_sweep PRIVATE dart)

add_executable(fbf_paper_card_house_probe fbf_paper_card_house_probe.cpp)
target_link_libraries(fbf_paper_card_house_probe PRIVATE dart)

add_executable(fbf_paper_arch_probe fbf_paper_arch_probe.cpp)
target_link_libraries(fbf_paper_arch_probe PRIVATE dart)

if(TARGET dart-collision-bullet)
  dart_benchmarks(
    TYPE BM_INTEGRATION
    SOURCES
      bm_boxes.cpp
    LINK_LIBRARIES
      dart
      dart-collision-bullet
  )

  # Headless boxes driver: doubles as a determinism/correctness oracle (prints
  # state checksums) and a headless profiling target (dumps the built-in text
  # profiler when DART_BUILD_PROFILE is on). Not a Google Benchmark target.
  add_executable(boxes_headless boxes_headless.cpp)
  target_link_libraries(boxes_headless PRIVATE dart dart-collision-bullet)
endif()

if(TARGET dart-collision-ode)
  set(contact_container_link_libraries dart dart-collision-ode)
  if(TARGET dart-collision-bullet)
    list(APPEND contact_container_link_libraries dart-collision-bullet)
  endif()

  dart_benchmarks(
    TYPE BM_INTEGRATION
    SOURCES
      bm_contact_container.cpp
    INCLUDE_DIRS
      ${DART_SOURCE_DIR}/examples/contact_benchmark
    LINK_LIBRARIES
      ${contact_container_link_libraries}
  )

  if(TARGET dart-collision-bullet)
    target_compile_definitions(
      BM_INTEGRATION_contact_container
      PRIVATE DART_CONTACT_CONTAINER_BENCHMARK_HAS_BULLET
    )
  endif()
endif()

# Headless contact-aware inverse-dynamics driver (checksum oracle + stepping-
# loop timer + text-profiler dump). Core dynamics only; no collision backend
# needed. Not a Google Benchmark target.
add_executable(id_headless id_headless.cpp)
target_link_libraries(id_headless PRIVATE dart)

if(TARGET dart-utils)
  dart_benchmarks(
    TYPE BM_INTEGRATION
    SOURCES
      bm_kinematics.cpp
    LINK_LIBRARIES
      dart-utils
  )

  dart_benchmarks(
    TYPE BM_INTEGRATION
    SOURCES
      bm_soft_body.cpp
    LINK_LIBRARIES
      dart-utils
  )

  add_executable(soft_body_headless soft_body_headless.cpp)
  target_link_libraries(soft_body_headless PRIVATE dart-utils)

  foreach(soft_body_collision_library dart-collision-bullet dart-collision-ode)
    if(TARGET ${soft_body_collision_library})
      target_link_libraries(
        BM_INTEGRATION_soft_body
        PRIVATE ${soft_body_collision_library}
      )
      target_link_libraries(
        soft_body_headless
        PRIVATE ${soft_body_collision_library}
      )
    endif()
  endforeach()
endif()

# Opt-in off-screen GLX-pbuffer capture smoke for the dart-gui-osg helper
# (dart/gui/osg/OffscreenViewer). Default OFF (see DART_ENABLE_GUI_OSG_SMOKE_TESTS);
# needs a DISPLAY, so run headless hosts under xvfb-run. The test returns the
# ctest skip code (77) with a clear message when no DISPLAY is present.
if(DART_ENABLE_GUI_OSG_SMOKE_TESTS AND TARGET dart-gui-osg)
  add_executable(offscreen_capture_smoke offscreen_capture_smoke.cpp)
  target_link_libraries(offscreen_capture_smoke PRIVATE dart dart-gui-osg)
  if(TARGET tests)
    add_dependencies(tests offscreen_capture_smoke)
  endif()
  if(TARGET tests_and_run)
    add_dependencies(tests_and_run offscreen_capture_smoke)
  endif()
  add_test(NAME offscreen_capture_smoke COMMAND offscreen_capture_smoke)
  set_tests_properties(offscreen_capture_smoke PROPERTIES SKIP_RETURN_CODE 77)
endif()
