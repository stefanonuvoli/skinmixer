#ifndef DETACH_H
#define DETACH_H

#include <nvl/math/segment.h>

#include <nvl/utilities/dummy_variables.h>

#include <vector>

#include "skinmixer/skinmixer_graph.h"

namespace skinmixer {

template<class Model>
std::vector<nvl::Index> detach(
        SkinMixerGraph<Model>& skinMixerGraph,
        const nvl::Index& nodeId,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth);

template<class Model>
std::vector<nvl::Index> detach(
        SkinMixerGraph<Model>& skinMixerGraph,
        Model* model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth);

template<class Model>
std::vector<Model> detachModel(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth,
        std::vector<nvl::Segment<typename Model::Mesh::Point>>& curveCoordinates,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps,
        typename Model::Mesh& resultMesh,
        std::vector<int>& faceSegmentation,
        std::vector<int>& jointSegmentation);

template<class Model>
std::vector<nvl::Segment<typename Model::Mesh::Point>> detachPreview(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool smooth);

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> getDetachingVertexFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const std::vector<typename Model::Skeleton::JointId>& descendandJoints,
        const double offset);
}

#include "detach.cpp"

#endif // DETACH_H
