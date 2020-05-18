#ifndef SKINMIXER_H
#define SKINMIXER_H

#include <nvl/math/segment.h>

#include <nvl/utilities/dummy_variables.h>

#include <vector>

#include "skinmixer/skinmixer_operation.h"

namespace skinmixer {

template<class Model>
void remove(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        std::vector<float>& vertexFuzzyValue,
        std::vector<float>& jointFuzzyValue);
template<class Model>
void detach(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        std::vector<float>& vertexFuzzyValue,
        std::vector<float>& jointFuzzyValue);

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint);
template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const std::vector<typename Model::Skeleton::JointId>& descendandJoints);

}

#include "skinmixer.cpp"

#endif // SKINMIXER_H
