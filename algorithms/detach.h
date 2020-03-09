#ifndef DETACH_H
#define DETACH_H

#include <vector>

namespace skinmixer {

template<class Model>
std::vector<Model> detachBySkeletonSegmentation(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        float compactness,
        bool keepEntireSkeleton,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps);

template<class Model>
std::vector<Model> detachByImplicitFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        bool keepEntireSkeleton,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps);

}

#include "detach.cpp"

#endif // DETACH_H
