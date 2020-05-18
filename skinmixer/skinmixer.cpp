#include "skinmixer.h"

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/skeleton_transfer.h>
#include <nvl/models/model_transfer.h>
#include <nvl/models/skeleton_adjacencies.h>
#include <nvl/models/mesh_normals.h>
#include <nvl/models/mesh_implicit_function.h>
#include <nvl/models/mesh_geometric_information.h>

#include <nvl/vcglib/vcg_curve_on_manifold.h>
#include <nvl/vcglib/vcg_mesh_refine.h>

#include <nvl/libigl/igl_geodesics.h>

#include <nvl/math/comparisons.h>
#include <nvl/math/laplacian.h>
#include <nvl/math/numeric_limits.h>

namespace skinmixer {

template<class Model>
void remove(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        std::vector<float>& vertexFuzzyValue,
        std::vector<float>& jointFuzzyValue)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Skeleton::JointId JointId;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;

    //TODO RIGIDITY

    std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction =
            skinmixer::jointJunctionFunction(
                model,
                targetJoint);

    for (VertexId vId = 0; vId < mesh.nextVertexId(); vId++) {
        if (mesh.isVertexDeleted(vId))
            continue;

        float value = 1.0 - jointJunctionFunction[vId];

        vertexFuzzyValue[vId] = std::min(vertexFuzzyValue[vId], value);
    }

    std::vector<JointId> removedJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);
    for (JointId jId : removedJoints) {
        jointFuzzyValue[jId] = 0.0;
    }
}

template<class Model>
void detach(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        std::vector<float>& vertexFuzzyValue,
        std::vector<float>& jointFuzzyValue)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Skeleton::JointId JointId;

    //TODO RIGIDITY

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;

    std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction =
            skinmixer::jointJunctionFunction(
                model,
                targetJoint);

    for (VertexId vId = 0; vId < mesh.nextVertexId(); vId++) {
        if (mesh.isVertexDeleted(vId))
            continue;

        float value = jointJunctionFunction[vId];

        vertexFuzzyValue[vId] = std::min(vertexFuzzyValue[vId], value);
    }

    std::vector<JointId> removedJoints = nvl::skeletonJointNonDescendants(skeleton, targetJoint);
    for (JointId jId : removedJoints) {
        jointFuzzyValue[jId] = 0.0;
    }
}


template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint)
{
    typedef typename Model::Skeleton Skeleton;

    const Skeleton& skeleton = model.skeleton;
    return jointJunctionFunction(model, targetJoint, nvl::skeletonJointDescendants(skeleton, targetJoint));
}

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const std::vector<typename Model::Skeleton::JointId>& descendandJoints)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    const Mesh& mesh = model.mesh;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    std::vector<SkinningWeightsScalar> function(mesh.vertexNumber(), nvl::maxLimitValue<SkinningWeightsScalar>());

    //Create function
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        function[vId] = skinningWeights.weight(vId, targetJoint);

        for (JointId jId : descendandJoints) {
            function[vId] += skinningWeights.weight(vId, jId);
        }
    }

    //Smooth function
    std::vector<std::vector<VertexId>> vvAdj = nvl::meshVertexVertexAdjacencies(mesh);
    nvl::laplacianSmoothing(function, vvAdj, 10, 0.8);

    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        function[vId] = std::min(std::max(function[vId], -1.0), 1.0);
    }

    return function;
}

}
