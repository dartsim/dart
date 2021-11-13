# Copyright (c) 2011-2021, The DART development contributors

dart_documentation(
  DOXYFILE ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
  PROJECT_NAME DART
  PROJECT_VERSION ${DART_VERSION}
  INPUT
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/
  EXCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/
  STRIP_FROM_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
