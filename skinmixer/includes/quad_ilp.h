#ifndef QUAD_ILP_H
#define QUAD_ILP_H

#include "quad_charts.h"

namespace QuadBoolean {
namespace internal {

enum ILPStatus { SOLUTIONFOUND, SOLUTIONWRONG, INFEASIBLE };

enum ILPMethod { LEASTSQUARES, ABS };

std::vector<int> solveILP(
        ChartData& chartData,
        const double alpha,
        const double beta,
        const ILPMethod& method,
        const bool isometry,
        const double timeLimit,
        double& gap,
        ILPStatus& status);

}
}

#include "quad_ilp.cpp"

#endif // QUAD_ILP_H
