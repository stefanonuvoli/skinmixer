#ifndef SKINMIXER_BLEND_ANIMATIONS_H
#define SKINMIXER_BLEND_ANIMATIONS_H

#include <nvl/nuvolib.h>

#include <vector>

#include "skinmixer_data.h"

namespace skinmixer {

template<class Model>
void initializeAnimationWeights(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry);

template<class Model>
nvl::Index blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        const std::vector<nvl::Index>& animationIds);

}

#include "skinmixer_blend_animations.cpp"

#endif // SKINMIXER_BLEND_ANIMATIONS_H
