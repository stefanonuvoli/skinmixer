#ifndef ATTACH_H
#define ATTACH_H

#include <nvl/nuvolib.h>

#include <vector>

#include "skinmixer/skinmixer_graph.h"

namespace skinmixer {

template<class Model>
nvl::Affine3d attachFindTransformation(
        SkinMixerGraph<Model>& skinMixerGraph,
        const nvl::Index& nodeId1,
        const nvl::Index& nodeId2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& currentTransformation1,
        const nvl::Affine3d& currentTransformation2);

template<class Model>
std::vector<nvl::Index> attach(
        SkinMixerGraph<Model>& skinMixerGraph,
        const nvl::Index& nodeId1,
        const nvl::Index& nodeId2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const nvl::Affine3d& currentTransformation1,
        const nvl::Affine3d& currentTransformation2);

}

#include "attach.cpp"

#endif // ATTACH_H
