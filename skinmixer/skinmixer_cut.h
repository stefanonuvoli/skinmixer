#ifndef SKINMIXER_CUT_H
#define SKINMIXER_CUT_H

#include <nvl/math/segment.h>

#include <vector>

#include "skinmixer/skinmixer_operation_graph.h"

#include <nvl/utilities/dummy_variables.h>

namespace skinmixer {

template<class Model>
std::vector<nvl::Index> cutOperation(
        OperationGraph<Model>& operationGraph,
        const nvl::Index& nodeId,
        const Operation& operation,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        const bool keepEntireSkeleton);

template<class Model>
std::vector<std::vector<nvl::Segment<typename Model::Mesh::Point>>> cutOperationPreview(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        std::vector<typename Model::SkinningWeights::Scalar>& cutFunction);

template<class Model>
std::vector<Model> cutAlongJointJunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        const bool keepEntireSkeleton,
        typename Model::Mesh& resultMesh,
        std::vector<std::vector<std::pair<typename Model::Mesh::VertexId, typename Model::Mesh::VertexId>>>& cutLines,
        std::vector<typename Model::SkinningWeights::Scalar>& cutFunction,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& birthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& birthFace,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& birthJoint,
        std::vector<int>& faceSegmentation,
        std::vector<int>& jointSegmentation);


namespace internal {

template<class Model>
void refineAlongJointJunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        typename Model::Mesh& resultMesh,
        std::vector<std::vector<std::pair<typename Model::Mesh::VertexId, typename Model::Mesh::VertexId>>>& cutLines,
        std::vector<typename Model::SkinningWeights::Scalar>& cutFunction,
        std::vector<typename Model::Mesh::VertexId>& resultBirthVertex,
        std::vector<typename Model::Mesh::FaceId>& resultBirthFace);

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionImplicitFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const std::vector<typename Model::Skeleton::JointId>& descendandJoints);

}

}

#include "skinmixer_cut.cpp"

#endif // SKINMIXER_CUT_H
