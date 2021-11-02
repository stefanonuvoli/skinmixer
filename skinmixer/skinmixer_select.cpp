#include "skinmixer_select.h"

#include <nvl/math/smoothing.h>

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_adjacencies.h>

#include <nvl/math/common_functions.h>

#include <math.h>

#define SELECT_LOWER_THRESHOLD 0.02
#define SELECT_UPPER_THRESHOLD 0.98

namespace skinmixer {

namespace internal {
double computeHardness(double x, double h);

}

template<class Model>
void computeSelectValues(
        const Model& model,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const std::vector<double>& jointSelectValue,
        std::vector<double>& vertexSelectValue)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    //Create function
    std::vector<double> alphas(mesh.nextVertexId());
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        vertexSelectValue[vId] = 0.0;

        for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
            if (jointSelectValue[jId] > 0.0 && !nvl::epsEqual(jointSelectValue[jId], 0.0)) {
                const SkinningWeightsScalar& sw = skinningWeights.weight(vId, jId);
                vertexSelectValue[vId] += static_cast<double>(sw);
            }
        }

        alphas[vId] = 1.0;
        if (vertexSelectValue[vId] < SELECT_LOWER_THRESHOLD) {
            vertexSelectValue[vId] = 0.0;
        }
        else if (vertexSelectValue[vId] > SELECT_UPPER_THRESHOLD) {
            vertexSelectValue[vId] = 1.0;
        }
        else {
            alphas[vId] = 0.8;
        }
    }

    //Smooth function
    if (smoothingIterations > 0) {
        std::vector<std::vector<VertexId>> vvAdj = nvl::meshVertexVertexAdjacencies(mesh);
        nvl::laplacianSmoothing(vertexSelectValue, vvAdj, smoothingIterations, alphas);
    }

    //Get connected components
    std::vector<FaceId> faceComponentMap;
    std::vector<std::vector<FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, faceComponentMap);

    //For each component, we check rigidity
    for (const std::vector<FaceId>& componentFaces : connectedComponents) {
        //Create connected component mesh
        Mesh componentMesh;
        std::vector<VertexId> componentBirthVertex;
        std::vector<FaceId> componentBirthFace;
        nvl::meshTransferFaces(mesh, componentFaces, componentMesh, componentBirthVertex, componentBirthFace);

        //Check rigidity
        double minValue = nvl::maxLimitValue<double>();
        double maxValue = nvl::minLimitValue<double>();
        double avgValue = 0.0;
        for (VertexId cId = 0; cId < componentMesh.nextVertexId(); ++cId) {
            if (componentMesh.isVertexDeleted(cId))
                continue;

            VertexId vId = componentBirthVertex[cId];

            assert(vId != nvl::MAX_INDEX);
            minValue = nvl::min(minValue, vertexSelectValue[vId]);
            maxValue = nvl::max(maxValue, vertexSelectValue[vId]);

            //Offset
            vertexSelectValue[vId] = internal::computeHardness(vertexSelectValue[vId], hardness);

            avgValue += vertexSelectValue[vId];
        }
        avgValue /= componentMesh.vertexNumber();

        bool isRigid = minValue >= 1.0 - rigidity || maxValue <= rigidity;

        //1.0 or 0.0 depending on avg value
        if (isRigid) {
            for (VertexId cId = 0; cId < componentMesh.nextVertexId(); ++cId) {
                if (componentMesh.isVertexDeleted(cId))
                    continue;

                const VertexId& vId = componentBirthVertex[cId];

                vertexSelectValue[vId] = avgValue >= 0.5 ? 1.0 : 0.0;
            }
        }
    }
}

template<class Model>
void computeReplaceSelectValues(
        const Model& model1,
        const Model& model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness1,
        const double hardness2,
        const bool includeParent1,
        const bool includeParent2,
        std::vector<double>& vertexSelectValue1,
        std::vector<double>& jointSelectValue1,
        std::vector<double>& vertexSelectValue2,
        std::vector<double>& jointSelectValue2)
{
    skinmixer::computeRemoveSelectValues(model1, targetJoint1, smoothingIterations, rigidity, hardness1, includeParent1, 0.0, vertexSelectValue1, jointSelectValue1);
    skinmixer::computeDetachSelectValues(model2, targetJoint2, smoothingIterations, rigidity, hardness2, includeParent2, 0.0, vertexSelectValue2, jointSelectValue2);
}

template<class Model>
void computeAttachSelectValues(
        const Model& model1,
        const Model& model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness2,
        const bool includeParent2,
        std::vector<double>& vertexSelectValue1,
        std::vector<double>& jointSelectValue1,
        std::vector<double>& vertexSelectValue2,
        std::vector<double>& jointSelectValue2)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Model::SkinningWeights SkinningWeights;

    const Mesh& mesh1 = model1.mesh;
    const Skeleton& skeleton1 = model1.skeleton;
    const SkinningWeights& skinningWeights1 = model1.skinningWeights;

    vertexSelectValue1.resize(mesh1.nextVertexId(), 1.0);
    jointSelectValue1.resize(skeleton1.jointNumber(), 1.0);

    for (VertexId vId = 0; vId < mesh1.nextVertexId(); ++vId) {
        if (mesh1.isVertexDeleted(vId))
            continue;

        if ((skinningWeights1.weight(vId, targetJoint1) > 0.0 + nvl::EPSILON && skinningWeights1.weight(vId, targetJoint1) < 1.0 - nvl::EPSILON) ||
            (!skeleton1.isRoot(targetJoint1) && skinningWeights1.weight(vId, skeleton1.parentId(targetJoint1)) > 0.0 + nvl::EPSILON && skinningWeights1.weight(vId, skeleton1.parentId(targetJoint1)) < 1.0 - nvl::EPSILON)) {
            vertexSelectValue1[vId] = 0.999;
        }
    }

    skinmixer::computeDetachSelectValues(model2, targetJoint2, smoothingIterations, rigidity, hardness2, includeParent2, 0.5, vertexSelectValue2, jointSelectValue2);
}

template<class Model>
void computeRemoveSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent,
        const double minThreshold,
        std::vector<double>& vertexSelectValue,
        std::vector<double>& jointSelectValue)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    jointSelectValue.resize(skeleton.jointNumber(), 1.0);
    std::vector<JointId> descendantJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);
    for (JointId jId : descendantJoints) {
        jointSelectValue[jId] = 0.0;
    }
    jointSelectValue[targetJoint] = 0.0;
    if (includeParent) {
        jointSelectValue[skeleton.parentId(targetJoint)] = 0.0;
    }

    vertexSelectValue.resize(mesh.nextVertexId(), 1.0);
    computeSelectValues(
        model,
        smoothingIterations,
        rigidity,
        hardness,
        jointSelectValue,
        vertexSelectValue);

    //We reset them: this value will be used in blending skeletons
    jointSelectValue[targetJoint] = 1.0;
    if (includeParent) {
        jointSelectValue[skeleton.parentId(targetJoint)] = 1.0;
    }

    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        if (vertexSelectValue[vId] < minThreshold) {
            vertexSelectValue[vId] = 0.0;
        }
    }
}

template<class Model>
void computeDetachSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double hardness,
        const bool includeParent,
        const double minThreshold,
        std::vector<double>& vertexSelectValue,
        std::vector<double>& jointSelectValue)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    jointSelectValue.resize(skeleton.jointNumber(), 0.0);
    std::vector<JointId> descendantJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);
    for (JointId jId : descendantJoints) {
        jointSelectValue[jId] = 1.0;
    }
    jointSelectValue[targetJoint] = 1.0;
    if (includeParent) {
        jointSelectValue[skeleton.parentId(targetJoint)] = 1.0;
    }

    vertexSelectValue.resize(mesh.nextVertexId(), 1.0);
    computeSelectValues(
        model,
        smoothingIterations,
        rigidity,
        hardness,
        jointSelectValue,
        vertexSelectValue);    

    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        if (vertexSelectValue[vId] < minThreshold) {
            vertexSelectValue[vId] = 0.0;
        }
    }
}

namespace internal {

inline double computeHardness(double x, double h)
{
    if (nvl::epsEqual(h, 0.0) || x <= SELECT_LOWER_THRESHOLD || x >= SELECT_UPPER_THRESHOLD) {
        return x;
    }

    h = (h + 1) / 2.0;
    h = (1.0 - h) / h;
    x = nvl::pow(x, h);

    x = std::max(std::min(x, 1.0), 0.0);

    return x;
}

}

}
