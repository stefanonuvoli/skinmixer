#ifndef QUAD_CONVERT_H
#define QUAD_CONVERT_H

#include <Eigen/Core>
#include <vector>

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
void VCGToEigen(
        PolyMeshType& vcgMesh,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        std::vector<int>& vMap,
        std::vector<int>& fMap,
        bool selectedOnly = false,
        int numVerticesPerFace = 3,
        int dim = 3);

template<class PolyMeshType>
void eigenToVCG(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        PolyMeshType& vcgMesh,
        int numVertices = 3,
        int dim = 3);

}
}

#include "quad_convert.cpp"

#endif // QUAD_CONVERT_H
