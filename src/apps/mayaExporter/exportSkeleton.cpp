#include "utils/MayaExportSkeleton.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace kinematics;

int main(int argc, char* argv[])
{
    FileInfoSkel<Skeleton> model;
    model.loadFile(GROUNDZERO_DATA_PATH"skel/YutingEuler.skel", kinematics::SKEL);
    ofstream out("model.ma");
    utils::mayaexports::MayaExportSkeleton::exportMayaAscii(model.getSkel(), out);
    out.close();

    return 0;
}
