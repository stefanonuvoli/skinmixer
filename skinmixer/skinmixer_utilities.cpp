#include "skinmixer.h"

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/skeleton_transfer.h>
#include <nvl/models/model_transfer.h>
#include <nvl/models/skeleton_adjacencies.h>
#include <nvl/models/mesh_normals.h>
#include <nvl/models/mesh_implicit_function.h>
#include <nvl/models/mesh_geometric_information.h>

#include <nvl/math/comparisons.h>
#include <nvl/math/laplacian.h>
#include <nvl/math/numeric_limits.h>

namespace skinmixer {

template<class Model>
std::vector<typename Model::SkinningWeights::Scalar> jointJunctionImplicitFunction(
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

    std::vector<SkinningWeightsScalar> cutFunction(mesh.vertexNumber(), nvl::maxLimitValue<SkinningWeightsScalar>());

    //Create function
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        cutFunction[vId] = skinningWeights.weight(vId, targetJoint);

        for (JointId jId : descendandJoints) {
            cutFunction[vId] += skinningWeights.weight(vId, jId);
        }

        cutFunction[vId] = std::min(std::max(cutFunction[vId], 0.0), 1.0);
    }

    return cutFunction;
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

    return nvl::getTranslationAffine3(t);
}

}
