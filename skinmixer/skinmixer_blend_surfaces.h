#ifndef SKINMIXER_BLEND_SURFACES_H
#define SKINMIXER_BLEND_SURFACES_H

#include <nvl/nuvolib.h>

#include <vector>

#include "skinmixer_data.h"

namespace skinmixer {

template<class Model>
void blendSurfaces(
        SkinMixerData<Model>& data,
        std::vector<nvl::Index>& newEntries);

}

#include "skinmixer_blend_surfaces.cpp"

#endif // SKINMIXER_BLEND_SURFACES_H
