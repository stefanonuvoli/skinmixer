#ifndef SKINMIXER_BLEND_SKELETONS_H
#define SKINMIXER_BLEND_SKELETONS_H

#include <nvl/nuvolib.h>

#include <vector>
#include <set>

#include "skinmixer_data.h"

namespace skinmixer {

template<class Model>
void blendSkeletons(
        SkinMixerData<Model>& data,
        std::vector<nvl::Index> cluster,
        typename SkinMixerData<Model>::Entry& resultEntry);

}

#include "skinmixer_blend_skeletons.cpp"

#endif // SKINMIXER_BLEND_SKELETONS_H
