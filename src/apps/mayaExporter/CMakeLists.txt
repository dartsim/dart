###############################################
# apps/mayaExporter
PROJECT(mayaExporter)
FILE(GLOB maya_export_model_srcs "exportModel.cpp")
FILE(GLOB maya_export_motion_srcs "exportMotion.cpp")
ADD_EXECUTABLE(mayaExportModel ${maya_export_model_srcs})
ADD_EXECUTABLE(mayaExportMotion ${maya_export_motion_srcs})
TARGET_LINK_LIBRARIES(mayaExportModel model3d mayaexports ticpp gflags glog)
TARGET_LINK_LIBRARIES(mayaExportMotion model3d mayaexports ticpp gflags glog)

SET_TARGET_PROPERTIES(mayaExportModel PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
SET_TARGET_PROPERTIES(mayaExportMotion PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
