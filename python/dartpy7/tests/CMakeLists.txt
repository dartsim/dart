if(NOT TARGET dartpy7)
  return()
endif()

set(dartpy7_test_sources
  test_gui.py
)

if(MSVC OR XCODE)
  set(dartpy7_pythonpath "${DART_DARTPY7_BUILD_DIR}/$<CONFIG>")
else()
  set(dartpy7_pythonpath "${DART_DARTPY7_BUILD_DIR}")
endif()

add_custom_target(dartpy7-tests
  COMMAND ${CMAKE_COMMAND} -E echo "Running dartpy7 unit tests"
  COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${dartpy7_pythonpath} ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR} -p test_*.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  SOURCES ${dartpy7_test_sources}
  DEPENDS dartpy7
)
