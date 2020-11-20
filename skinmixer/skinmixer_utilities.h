#ifndef SKINMIXER_UTILITIES_H
#define SKINMIXER_UTILITIES_H

#include <vector>

#include <nvl/math/affine.h>

namespace skinmixer {

template<class Model>
nvl::Affine3d findReplaceMoveTransformation(
        const Model* model1,
        const Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& currentTransformation1,
        const nvl::Affine3d& currentTransformation2);

}

#include "skinmixer_utilities.cpp"

#endif // SKINMIXER_UTILITIES_H
