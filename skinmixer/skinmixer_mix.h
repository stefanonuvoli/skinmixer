#ifndef SKINMIXER_MIX_H
#define SKINMIXER_MIX_H

#include <vector>

#include "skinmixer/skinmixer_utilities.h"

namespace skinmixer {

template<class Mesh>
Mesh mixMeshes(
        const std::vector<Mesh*>& meshes,
        const std::vector<std::vector<float>>& vertexFuzzyValue);

}

#include "skinmixer_mix.cpp"

#endif // SKINMIXER_MIX_H
