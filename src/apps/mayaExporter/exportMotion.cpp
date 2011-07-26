#include "mayaexports/MayaExportMotion.h"
#include "model3d/FileInfoSkel.hpp"
#include "model3d/FileInfoDof.h"
#include "utils/Paths.h"

using namespace model3d;

int main(int argc, char* argv[])
{
    FileInfoSkel<Skeleton> model;
    model.loadFile(GROUNDZERO_DATA_PATH"skel/YutingEuler.skel", model3d::SKEL);

    FileInfoDof motion(model.getSkel());
    motion.loadFile(GROUNDZERO_DATA_PATH"dof/RHand.dof");

    mayaexports::MayaExportMotion exporter(model.getSkel(),&motion);
    exporter.exportMayaAnim("motion.anim", 0, motion.getNumFrames()-1, "", model.getSkel()->getNumNodes());

    return 0;
}
