###############################################
# apps/mayaExporter
PROJECT(mayaExporter)
FILE(GLOB maya_export_model_srcs "exportSkeleton.cpp")
FILE(GLOB maya_export_motion_srcs "exportMotion.cpp")
ADD_EXECUTABLE(mayaExportSkeleton ${maya_export_model_srcs})
ADD_EXECUTABLE(mayaExportMotion ${maya_export_motion_srcs})
TARGET_LINK_LIBRARIES(mayaExportSkeleton kinematics utils ticpp gflags glog)
TARGET_LINK_LIBRARIES(mayaExportMotion kinematics utils ticpp gflags glog)

SET_TARGET_PROPERTIES(mayaExportSkeleton PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
SET_TARGET_PROPERTIES(mayaExportMotion PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
