#ifndef SKINMIXER_BLEND_SURFACES_H
#define SKINMIXER_BLEND_SURFACES_H

#include <nvl/nuvolib.h>

#include <vector>

#include "skinmixer_data.h"

namespace skinmixer {

template<class Model>
void blendSurfaces(
        SkinMixerData<Model>& data,
        std::vector<nvl::Index> cluster,
        typename SkinMixerData<Model>::Entry& resultEntry,
        const MixMode mixMode,
        const bool blendColorFromTextures,
        const unsigned int smoothingBorderIterations,
        const double smoothingBorderThreshold,
        const unsigned int smoothingInnerIterations,
        const double smoothingInnerAlpha,
        const double voxelSizeFactor,
        const double voxelDistanceFactor);

}

#include "skinmixer_blend_surfaces.cpp"

#endif // SKINMIXER_BLEND_SURFACES_H
