#ifndef SKINMIXER_H
#define SKINMIXER_H

#include <vector>

#include "skinmixer/skinmixer_data.h"

namespace skinmixer {

struct MixParameters {
    MixParameters() :
        mixMode(MixMode::MESHING),
        blendColorsFromTextures(true),
        smoothingBorderIterations(30),
        smoothingBorderThreshold(0.8),
        smoothingInnerIterations(15),
        smoothingInnerAlpha(0.8),
        smoothingResultIterations(3),
        voxelSize(0.8),
        voxelDistance(30.0)
    {

    }

    MixMode mixMode;
    bool blendColorsFromTextures;
    unsigned int smoothingBorderIterations;
    double smoothingBorderThreshold;
    unsigned int smoothingInnerIterations;
    double smoothingInnerAlpha;
    unsigned int smoothingResultIterations;
    double voxelSize;
    double voxelDistance;
};

struct MixAnimationParameters {
    MixAnimationParameters() :
        samplingFPS(30.0),
        rotationWeight(0.8),
        globalWeight(0.9),
        localWeight(0.0),
        globalDerivativeWeight(0.1),
        localDerivativeWeight(0.0),
        windowSize(1),
        windowMainWeight(0.6),
        smoothingIterations(10),
        smoothingThreshold(0.95)
    {

    }

    double samplingFPS;
    double rotationWeight;
    double globalWeight;
    double localWeight;
    double globalDerivativeWeight;
    double localDerivativeWeight;
    unsigned int windowSize;
    double windowMainWeight;
    unsigned int smoothingIterations;
    double smoothingThreshold;
};

template<class Model>
std::vector<nvl::Index> mix(
        SkinMixerData<Model>& data,
        const MixParameters& par);


template<class Model>
void mixAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        std::vector<std::pair<nvl::Index, nvl::Index>>& resultAnimations,
        const MixAnimationParameters& parameters);

//template<class Model>
//nvl::Index chooseAnimation(
//        SkinMixerData<Model>& data,
//        typename SkinMixerData<Model>::Entry& entry,
//        const nvl::Index& index);

template<class Model>
nvl::Index replace(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const ReplaceMode& replaceMode,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness1,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2);

template<class Model>
nvl::Index attach(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2);

template<class Model>
nvl::Index remove(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent);

template<class Model>
nvl::Index detach(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int weightSmoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent);


}

#include "skinmixer.cpp"

#endif // SKINMIXER_H
