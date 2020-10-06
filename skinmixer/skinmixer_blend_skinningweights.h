#ifndef SKINMIXER_BLEND_SKINNINGWEIGHTS_H
#define SKINMIXER_BLEND_SKINNINGWEIGHTS_H

#include <nvl/nuvolib.h>

#include <vector>
#include <set>

#include "skinmixer_data.h"

namespace skinmixer {

template<class Model>
void blendSkinningWeights(
        const SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry);

}

#include "skinmixer_blend_skinningweights.cpp"

#endif // SKINMIXER_BLEND_SKINNINGWEIGHTS_H
