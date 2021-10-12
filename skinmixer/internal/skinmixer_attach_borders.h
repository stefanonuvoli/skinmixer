#ifndef SKINMIXER_ATTACH_BORDERS_H
#define SKINMIXER_ATTACH_BORDERS_H

#include <nvl/nuvolib.h>

#include <unordered_set>
#include <vector>
#include <utility>

namespace skinmixer {
namespace internal {

template<class Mesh>
Mesh attachMeshesByBorders(
        const Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices,
        const std::unordered_set<typename Mesh::FaceId>& newSurfaceFaces,
        std::unordered_set<typename Mesh::VertexId>& newSnappedVertices,
        std::unordered_set<typename Mesh::VertexId>& preSnappedVertices);
}
}

#include "skinmixer_attach_borders.cpp"

#endif // SKINMIXER_ATTACH_BORDERS_H
