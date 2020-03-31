#ifndef BLEND_WEIGHTS_H
#define BLEND_WEIGHTS_H

#include <nvl/math/segment.h>

#include <vector>

#include "skinmixer/skinmixer_graph.h"

namespace skinmixer {

template<class Model>
void blendSkinningWeights(
        SkinMixerGraph<Model>& skinMixerGraph,
        const nvl::Index& nodeId);

}

#include "blend_weights.cpp"

#endif // BLEND_WEIGHTS_H
