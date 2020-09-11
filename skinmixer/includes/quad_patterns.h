#ifndef QUAD_PATTERNS_H
#define QUAD_PATTERNS_H

#include <vector>

#include <Eigen/Core>

namespace QuadBoolean {
namespace internal {

void computePattern(
        const Eigen::VectorXi &l,
        Eigen::MatrixXd& patchV,
        Eigen::MatrixXi& patchF,
        std::vector<size_t>& borders,
        std::vector<size_t>& corners);

}
}

#endif // QUAD_PATTERNS_H
