#include "skinmixer.h"

#include <nvl/models/algorithms/skeleton_adjacencies.h>
#include <nvl/models/algorithms/mesh_adjacencies.h>

#include <nvl/math/smoothing.h>
#include <nvl/math/numeric_limits.h>

namespace skinmixer {

template<class Model>
nvl::Affine3d findReplaceMoveTransformation(
        const Model* model1,
        const Model* model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& currentTransformation1,
        const nvl::Affine3d& currentTransformation2)
{
    typedef typename Model::Mesh::Point Point;
    typedef typename Model::Skeleton Skeleton;

    const Skeleton& skeleton1 = model1->skeleton;
    const Skeleton& skeleton2 = model2->skeleton;

    Point v1 = currentTransformation1 * (skeleton1.joint(targetJoint1).bindPose() * Point(0,0,0));
    Point v2 = currentTransformation2 * (skeleton2.joint(targetJoint2).bindPose() * Point(0,0,0));
    Point t = v1 - v2;

    return nvl::Affine3d(nvl::Translation3d(t));
}

}
