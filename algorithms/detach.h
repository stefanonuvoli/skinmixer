#ifndef DETACH_H
#define DETACH_H

#include <nvl/math/segment.h>

#include <vector>

namespace skinmixer {

template<class Model>
std::vector<Model> detachBySkeletonSegmentation(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const float compactness,
        const bool keepEntireSkeleton,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps);

template<class Model>
std::vector<Model> detachBySkinningWeightFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth,
        std::vector<nvl::Segment<typename Model::Mesh::Point>>& functionSegments,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps);

}

#include "detach.cpp"

#endif // DETACH_H
