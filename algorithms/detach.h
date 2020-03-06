#ifndef DETACH_H
#define DETACH_H

#include <vector>

namespace skinmixer {

template<class Model>
Model detachModelPart(
        const Model& model,
        const std::vector<typename Model::Mesh::VertexId>& faces,
        const std::vector<typename Model::Skeleton::JointId>& joints,
        std::vector<typename Model::Mesh::VertexId>& vertexMaps,
        std::vector<typename Model::Mesh::FaceId>& faceMaps,
        std::vector<typename Model::Skeleton::JointId>& jointMaps);

template<class Model>
std::vector<Model> detachFromSkeletonSegmentation(
        const Model& model,
        const typename Model::Skeleton::JointId targetJoint,
        float compactness,
        bool keepEntireSkeleton,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps);

}

#include "detach.cpp"

#endif // DETACH_H
