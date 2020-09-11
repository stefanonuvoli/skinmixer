#ifndef QUAD_FEASIBILITY_H
#define QUAD_FEASIBILITY_H

namespace QuadBoolean {
namespace internal {

enum FeasibilityResult { NonConsistant, AlreadyOk, SolvedQuadOnly, SolvedQuadDominant, NonSolved };

template <class PolyMeshType, class TriangleMeshType>
FeasibilityResult solveFeasibility(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        const bool polychordSolver,
        const bool splitSolver);

}
}

#include "quad_feasibility.cpp"

#endif // QUAD_FEASIBILITY_H
