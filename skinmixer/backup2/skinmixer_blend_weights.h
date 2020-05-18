#ifndef SKINMIXER_BLEND_WEIGHTS_H
#define SKINMIXER_BLEND_WEIGHTS_H

#include <nvl/math/segment.h>

#include <vector>

#include "skinmixer/skinmixer_operation_graph.h"

namespace skinmixer {

template<class Model>
void blendSkinningWeights(
        OperationGraph<Model>& operationGraph,
        const nvl::Index& nodeId);

}

#include "skinmixer_blend_weights.cpp"

#endif // SKINMIXER_BLEND_WEIGHTS_H
