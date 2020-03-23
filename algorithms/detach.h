#ifndef DETACH_H
#define DETACH_H

#include <nvl/math/segment.h>

#include <nvl/utilities/dummy_variables.h>

#include <vector>

namespace skinmixer {

template<class Model>
std::vector<Model> detachBySkinningWeightFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth,
        std::vector<nvl::Segment<typename Model::Mesh::Point>>& curveCoordinates,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& birthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& birthFace,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& birthJoint);


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
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps,
        typename Model::Mesh& resultMesh,
        std::vector<int>& faceSegmentation = nvl::internal::dummyVectorInt,
        std::vector<int>& jointSegmentation = nvl::internal::dummyVectorInt2);

}

#include "detach.cpp"

#endif // DETACH_H
