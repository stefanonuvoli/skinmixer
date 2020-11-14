#include "skinmixer_select.h"

#include <nvl/math/smoothing.h>

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_adjacencies.h>

namespace skinmixer {

template<class Model>
void computeAttachSelectValues(
        const Model& model1,
        const Model& model2,
        const typename Model::Skeleton::JointId& targetJoint1,
        const typename Model::Skeleton::JointId& targetJoint2,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double offset1,
        const double offset2,
        std::vector<double>& vertexSelectValue1,
        std::vector<double>& jointSelectValue1,
        std::vector<double>& vertexSelectValue2,
        std::vector<double>& jointSelectValue2)
{
    skinmixer::computeRemoveSelectValues(model1, targetJoint1, smoothingIterations, rigidity, offset1, vertexSelectValue1, jointSelectValue1);
    skinmixer::computeDetachSelectValues(model2, targetJoint2, smoothingIterations, rigidity, offset2, vertexSelectValue2, jointSelectValue2);
}

template<class Model>
void computeRemoveSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double offset,
        std::vector<double>& vertexSelectValue,
        std::vector<double>& jointSelectValue)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    vertexSelectValue.resize(mesh.nextVertexId(), 1.0);
    jointSelectValue.resize(skeleton.jointNumber(), 1.0);

    //Get connected components
    std::vector<FaceId> faceComponentMap;
    std::vector<std::vector<FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, faceComponentMap);

    //Get function
    std::vector<SkinningWeightsScalar> jointFunction =
            skinmixer::jointJunctionFunction(
                model,
                targetJoint,
                smoothingIterations);

    //Fill vertex values
    const double offsetFactor = offset - 0.5;
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        vertexSelectValue[vId] = 1.0 - jointFunction[vId];
        if (vertexSelectValue[vId] >= 0.01 && vertexSelectValue[vId] <= 0.99) {
            vertexSelectValue[vId] += offsetFactor;
            vertexSelectValue[vId] = (vertexSelectValue[vId] - offsetFactor) / (1.0 - offsetFactor);
        }

        if (vertexSelectValue[vId] <= 0.02) {
            vertexSelectValue[vId] = 0.0;
        }
        if (vertexSelectValue[vId] >= 0.98) {
            vertexSelectValue[vId] = 1.0;
        }
    }

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

    std::vector<JointId> descendantJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);
    for (JointId jId : descendantJoints) {
        jointSelectValue[jId] = 0.0;
    }
}

template<class Model>
void computeDetachSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int smoothingIterations,
        const double rigidity,
        const double offset,
        std::vector<double>& vertexSelectValue,
        std::vector<double>& jointSelectValue)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    vertexSelectValue.resize(mesh.nextVertexId(), 1.0);
    jointSelectValue.resize(skeleton.jointNumber(), 1.0);

    //Get connected components
    std::vector<FaceId> faceComponentMap;
    std::vector<std::vector<FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, faceComponentMap);

    //Get function
    std::vector<SkinningWeightsScalar> jointFunction =
            skinmixer::jointJunctionFunction(
                model,
                targetJoint,
                smoothingIterations);

    //Fill vertex values
    const double offsetFactor = offset - 0.5;
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        vertexSelectValue[vId] = jointFunction[vId];
        if (vertexSelectValue[vId] >= 0.01 && vertexSelectValue[vId] <= 0.99) {
            vertexSelectValue[vId] += offsetFactor;
            vertexSelectValue[vId] = (vertexSelectValue[vId] - offsetFactor) / (1.0 - offsetFactor);
        }

        if (vertexSelectValue[vId] <= 0.02) {
            vertexSelectValue[vId] = 0.0;
        }
        if (vertexSelectValue[vId] >= 0.98) {
            vertexSelectValue[vId] = 1.0;
        }
    }

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

    std::vector<JointId> nonDescendantJoints = nvl::skeletonJointNonDescendants(skeleton, targetJoint);
    for (JointId jId : nonDescendantJoints) {
        jointSelectValue[jId] = 0.0;
    }
}

}
