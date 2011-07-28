#include "Eigen/Dense"
using namespace Eigen;
#include "glog/logging.h"
using namespace google;
#include <vector>
using namespace std;


namespace lcpsolver {
    int Lemke(const MatrixXd& _M, const VectorXd& _q, VectorXd& _z);
} //namespace lcpsolver