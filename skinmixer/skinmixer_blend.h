#ifndef SKINMIXER_BLEND_H
#define SKINMIXER_BLEND_H

#include <nvl/nuvolib.h>

#include <vector>

namespace skinmixer {

template<class Model>
void blendModels(
        const std::vector<Model*>& models,
        const std::vector<std::vector<float>>& vertexSelectValue,
        Model* resultModel,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::VertexId>>& birthVertex,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::FaceId>>& birthFace);

}

#include "skinmixer_blend.cpp"

#endif // SKINMIXER_BLEND_H
