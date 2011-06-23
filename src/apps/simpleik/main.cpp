#include <iostream>
using namespace std;

#include <glog/logging.h>
using namespace google;

int main(int argc, char* argv[]) {
    // Init google libraries
    ParseCommandLineFlags(&argc, &argv, true);
    InitGoogleLogging(argv[0]);

    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = INFO;
    FLAGS_v = 0;

    LOG(INFO) << "simpleik begins";

    LOG(INFO) << "simpleik OK";
}
