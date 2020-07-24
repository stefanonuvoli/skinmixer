#ifndef SKINMIXER_BLEND_H
#define SKINMIXER_BLEND_H

#include <nvl/nuvolib.h>

#include <vector>

namespace skinmixer {

template<class Mesh>
void blendMeshes(
        const std::vector<Mesh*>& meshes,
        const std::vector<std::vector<float>>& vertexFuzzyValue,
        Mesh& preservedMesh,
        Mesh& newSurfaceMesh,
        std::vector<typename Mesh::VertexId>& preservedBirthVertices,
        std::vector<typename Mesh::FaceId>& preservedBirthFaces,
        std::vector<nvl::Index>& preservedVerticesBirthModel,
        std::vector<nvl::Index>& preservedFacesBirthModel);

}

#include "skinmixer_blend.cpp"

#endif // SKINMIXER_BLEND_H
