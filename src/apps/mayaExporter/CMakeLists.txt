###############################################
# apps/mayaExporter
PROJECT(mayaExporter)
FILE(GLOB maya_export_model_srcs "exportSkeleton.cpp")
FILE(GLOB maya_export_motion_srcs "exportMotion.cpp")
ADD_EXECUTABLE(mayaExportSkeleton ${maya_export_model_srcs})
ADD_EXECUTABLE(mayaExportMotion ${maya_export_motion_srcs})
TARGET_LINK_LIBRARIES(mayaExportSkeleton kinematics utils)
TARGET_LINK_LIBRARIES(mayaExportMotion kinematics utils)
LINK_THIRDPARTY_GFLAGS(mayaExportSkeleton)
LINK_THIRDPARTY_GLOG(mayaExportSkeleton)
LINK_THIRDPARTY_GFLAGS(mayaExportMotion)
LINK_THIRDPARTY_GLOG(mayaExportMotion)

SET_TARGET_PROPERTIES(mayaExportSkeleton PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
SET_TARGET_PROPERTIES(mayaExportMotion PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
