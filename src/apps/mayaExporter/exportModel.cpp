#include "mayaexports/MayaExportSkeleton.h"
#include "model3d/FileInfoSkel.hpp"
#include "utils/Paths.h"

using namespace model3d;

int main(int argc, char* argv[])
{
    FileInfoSkel<Skeleton> model;
    model.loadFile(GROUNDZERO_DATA_PATH"skel/YutingEuler.skel", model3d::SKEL);
    ofstream out("model.ma");
    mayaexports::MayaExportSkeleton::exportMayaAscii(model.getSkel(), out);
    out.close();

    return 0;
}
