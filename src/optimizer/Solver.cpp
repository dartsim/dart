#include "Solver.h"

namespace optimizer {

    Solver::Solver()
        : mProb(NULL) {
    }

    Solver::Solver(Problem *prob)
        : mProb(prob) {
    }

    Solver::~Solver() {
    }

    void Solver::setProblem(Problem * prob) {
        this->mProb = prob;
    }

    Problem* Solver::getProblem() const {
        return mProb;
    }

} // namespace optimizer
