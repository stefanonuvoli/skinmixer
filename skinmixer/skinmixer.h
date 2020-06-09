#ifndef SKINMIXER_H
#define SKINMIXER_H

#include <vector>

#include "skinmixer/skinmixer_data.h"

namespace skinmixer {

template<class Model>
std::vector<Model*> meshing(
        SkinMixerData<Model>& data);

template<class Model>
void attach(
        SkinMixerData<Model>& data,
        Model* model1,
        Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& transformation1,
        const nvl::Affine3d& transformation2);

template<class Model>
void remove(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity);

template<class Model>
void detach(
        SkinMixerData<Model>& data,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity);


}

#include "skinmixer.cpp"

#endif // SKINMIXER_H
