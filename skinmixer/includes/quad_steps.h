#ifndef QUAD_STEPS_H
#define QUAD_STEPS_H

#include <vector>

#include <Eigen/Core>

#include <unordered_set>

#include "quad_charts.h"
#include "quad_ilp.h"

namespace QuadBoolean {

namespace internal {

template<class PolyMeshType, class TriangleMeshType>
bool makeILPFeasible(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        const bool polychordSolver,
        const bool splitSolver);

template<class TriangleMeshType, class PolyMeshType>
std::vector<int> getPatchDecomposition(
        TriangleMeshType& newSurface,
        PolyMeshType& preservedSurface,
        std::vector<std::vector<size_t>>& partitions,
        std::vector<std::vector<size_t>>& corners,
        const bool initialRemeshing,
        const double edgeFactor,
        const bool reproject,
        const bool splitConcaves,
        const bool finalSmoothing);

template<class TriangleMeshType>
std::vector<int> findSubdivisions(
        TriangleMeshType& newSurface,
        ChartData& chartData,
        const double alpha,
        const ILPMethod& method);

template<class TriangleMeshType, class PolyMeshType>
void quadrangulate(
        TriangleMeshType& newSurface,
        const ChartData& chartData,
        const std::vector<int>& ilpResult,
        const int chartSmoothingIterations,
        const int quadrangulationSmoothingIterations,
        PolyMeshType& quadrangulatedNewSurface,
        std::vector<int>& quadrangulatedNewSurfaceLabel);


template<class PolyMeshType, class TriangleMeshType>
void getResult(
        PolyMeshType& preservedSurface,
        PolyMeshType& quadrangulatedNewSurface,
        PolyMeshType& result,
        TriangleMeshType& targetBoolean,
        const int resultSmoothingIterations,
        const double resultSmoothingNRing,
        const int resultSmoothingLaplacianIterations,
        const double resultSmoothingLaplacianRing,
        const std::unordered_map<size_t, size_t>& preservedFacesMap,
        const std::unordered_map<size_t, size_t>& preservedVerticesMap);

}
}

#include "quad_steps.cpp"

#endif // QUAD_STEPS_H
