#include "skinmixer.h"

#include <nvl/models/skeleton_adjacencies.h>
#include <nvl/models/mesh_adjacencies.h>

#include <nvl/math/smoothing.h>
#include <nvl/math/numeric_limits.h>

namespace skinmixer {

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIteration)
{
    typedef typename Model::Skeleton Skeleton;

    const Skeleton& skeleton = model.skeleton;
    return jointJunctionFunction(model, targetJoint, nvl::skeletonJointDescendants(skeleton, targetJoint), smoothingIteration);
}

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const std::vector<typename Model::Skeleton::JointId>& descendandJoints,
        const unsigned int smoothingIteration)
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
    if (smoothingIteration > 0) {
        std::vector<std::vector<VertexId>> vvAdj = nvl::meshVertexVertexAdjacencies(mesh);
        nvl::laplacianSmoothing(function, vvAdj, smoothingIteration, 0.8);
    }

    //Limit values between 0 and 1
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        function[vId] = std::min(std::max(function[vId], 0.0), 1.0);
    }

    return function;
}

template<class Model>
nvl::Affine3d findAttachMoveTransformation(
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

    Point v1 = currentTransformation1 * (skeleton1.joint(targetJoint1).restTransform() * Point(0,0,0));
    Point v2 = currentTransformation2 * (skeleton2.joint(targetJoint2).restTransform() * Point(0,0,0));
    Point t = v1 - v2;

    return nvl::Affine3d(nvl::Translation3d(t));
}

}
