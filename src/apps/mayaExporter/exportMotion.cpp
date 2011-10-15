#include "utils/MayaExportMotion.h"
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/FileInfoDof.h"
#include "utils/Paths.h"

using namespace kinematics;

int main(int argc, char* argv[])
{
    FileInfoSkel<Skeleton> model;
    model.loadFile(DART_DATA_PATH"skel/YutingEuler.skel", kinematics::SKEL);

    FileInfoDof motion(model.getSkel());
    motion.loadFile(DART_DATA_PATH"dof/RHand.dof");

    utils::mayaexports::MayaExportMotion exporter(model.getSkel(),&motion);
    exporter.exportMayaAnim("motion.anim", 0, motion.getNumFrames()-1, "", model.getSkel()->getNumNodes());

    return 0;
}
