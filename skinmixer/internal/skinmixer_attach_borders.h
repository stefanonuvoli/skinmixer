#ifndef SKINMIXER_ATTACH_BORDERS_H
#define SKINMIXER_ATTACH_BORDERS_H

#include <nvl/nuvolib.h>

#include <unordered_set>
#include <vector>
#include <utility>

namespace skinmixer {
namespace internal {

template<class Mesh>
void attachMeshesByBorders(
        Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& meshNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices,
        std::unordered_set<typename Mesh::VertexId>& newSnappedVertices,
        std::unordered_set<typename Mesh::VertexId>& preSnappedVertices);

template<class Mesh>
std::vector<typename Mesh::FaceId> getPreUsedFacesAfterAttaching(
        const Mesh& preMesh,
        const std::unordered_set<typename Mesh::VertexId>& preNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& preSnappedVertices);

}
}

#include "skinmixer_attach_borders.cpp"

#endif // SKINMIXER_ATTACH_BORDERS_H
