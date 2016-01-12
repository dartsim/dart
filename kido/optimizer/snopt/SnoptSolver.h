/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef KIDO_OPTIMIZER_SNOPT_SNOPT_H
#define KIDO_OPTIMIZER_SNOPT_SNOPT_H

#include <vector>
#include <Eigen/Dense>
#include "kido/optimizer/snopt/SnoptInterface.h"
#include "kido/optimizer/Solver.h"

namespace kido {
namespace optimizer {

class Problem;
class ConstraintSet;
class ObjectiveSet;
class Variable;

namespace snopt {

class SnoptSolver : public Solver {
public:
    SnoptSolver(Problem *problem);
    virtual ~SnoptSolver();

    virtual bool solve();

    virtual void resetSolver();
    static int iterUpdate(long mask, int compute_gradients, double *coefs, void *update_data);
private:
    SnoptInterface *mSnopt;
    /* std::vector<Dofs> mVariables; */
    /* ConstrBox* mConstrBox; */
    /* ObjBox* mObjBox; */

    ConstraintSet* conBox();
    ObjectiveSet* objBox();

    bool mNoDisplay;
    int mSolverIter;

    int mTotalDofs;
    int mOptCount;
    bool mPrint;
    int mUnit;


};

} // namespace snopt
} // namespace optimizer
} // namespace kido

#endif // #ifndef KIDO_OPTIMIZER_SNOPT_SNOPT_H

