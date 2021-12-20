#ifndef SKINMIXER_BLEND_ANIMATIONS_H
#define SKINMIXER_BLEND_ANIMATIONS_H

#include <nvl/nuvolib.h>

#include <vector>

#include "skinmixer_data.h"

#define BLEND_ANIMATION_FIXED 0
#define BLEND_ANIMATION_KEYFRAME 1
#define BLEND_ANIMATION_LOOP 2
#define BLEND_ANIMATION_NONE nvl::NULL_ID

namespace skinmixer {

template<class Model>
void initializeAnimationWeights(
        SkinMixerData<Model>& data,
        std::vector<nvl::Index> cluster,
        typename SkinMixerData<Model>::Entry& resultEntry);

template<class Model>
void blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        std::vector<std::pair<nvl::Index, nvl::Index>>& resultAnimations,
        const double& samplingFPS,
        const double& rotationWeight,
        const double& globalWeight,
        const double& localWeight,
        const double& globalDerivativeWeight,
        const double& localDerivativeWeight,
        const unsigned int& windowSize,
        const double& mainFrameWeight,
        const unsigned int& smoothingIterations,
        const double& smoothingThreshold);

//template<class Model>
//nvl::Index findBestAnimation(
//        SkinMixerData<Model>& data,
//        typename SkinMixerData<Model>::Entry& entry,
//        const nvl::Index& index);
}

#include "skinmixer_blend_animations.cpp"

#endif // SKINMIXER_BLEND_ANIMATIONS_H
