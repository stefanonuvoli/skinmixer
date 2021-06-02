#ifndef SKINMIXER_H
#define SKINMIXER_H

#include <vector>

#include "skinmixer/skinmixer_data.h"

namespace skinmixer {

template<class Model>
std::vector<nvl::Index> mix(
        SkinMixerData<Model>& data);

template<class Model>
void mixAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        nvl::Index& targetAnimationId);

//template<class Model>
//nvl::Index chooseAnimation(
//        SkinMixerData<Model>& data,
//        typename SkinMixerData<Model>::Entry& entry,
//        const nvl::Index& index);

template<class Model>
void replace(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int functionSmoothingIterations,
        const double rigidity,
        const double hardness1,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2);

template<class Model>
void attach(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int functionSmoothingIterations,
        const double rigidity,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2);

template<class Model>
void remove(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent);

template<class Model>
void detach(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIteration,
        const double rigidity,
        const double hardness,
        const bool includeParent);


}

#include "skinmixer.cpp"

#endif // SKINMIXER_H
