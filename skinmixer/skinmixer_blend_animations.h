#ifndef SKINMIXER_BLEND_ANIMATIONS_H
#define SKINMIXER_BLEND_ANIMATIONS_H

#include <nvl/nuvolib.h>

#include <vector>

#include "skinmixer_data.h"

namespace skinmixer {

template<class Model>
void initializeAnimationWeights(
        SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& newEntries);

template<class Model>
void blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        nvl::Index& targetAnimationId);

//template<class Model>
//nvl::Index findBestAnimation(
//        SkinMixerData<Model>& data,
//        typename SkinMixerData<Model>::Entry& entry,
//        const nvl::Index& index);
}

#include "skinmixer_blend_animations.cpp"

#endif // SKINMIXER_BLEND_ANIMATIONS_H
