#ifndef SKINMIXER_UTILITIES_H
#define SKINMIXER_UTILITIES_H

#include <vector>

#include <nvl/math/affine_transformations.h>

namespace skinmixer {

template<class Model>
nvl::Affine3d findAttachMoveTransformation(
        const Model* model1,
        const Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& currentTransformation1,
        const nvl::Affine3d& currentTransformation2);

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionImplicitFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const std::vector<typename Model::Skeleton::JointId>& descendandJoints);

}

#include "skinmixer_utilities.cpp"

#endif // SKINMIXER_UTILITIES_H
