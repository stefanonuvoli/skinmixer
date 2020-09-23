#include "skinmixer_select.h"

#include <nvl/math/smoothing.h>

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_adjacencies.h>

namespace skinmixer {

template<class Model>
void computeRemoveSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity,
        std::vector<float>& vertexSelectValue,
        std::vector<bool>& jointSelectValue)
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

    vertexSelectValue.resize(mesh.nextVertexId(), 1.0);
    jointSelectValue.resize(skeleton.jointNumber(), true);

    //Get connected components
    std::vector<FaceId> faceComponentMap;
    std::vector<std::vector<FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, faceComponentMap);

    //Get function
    std::vector<SkinningWeightsScalar> jointFunction =
            skinmixer::jointJunctionFunction(
                model,
                targetJoint,
                functionSmoothingIterations);

    for (const std::vector<FaceId>& componentFaces : connectedComponents) {
        //Create connected component mesh
        Mesh componentMesh;
        std::vector<VertexId> componentBirthVertex;
        std::vector<FaceId> componentBirthFace;
        nvl::meshTransferFaces(mesh, componentFaces, componentMesh, componentBirthVertex, componentBirthFace);

        float minFunction = nvl::maxLimitValue<float>();
        float maxFunction = nvl::minLimitValue<float>();
        float avgFunction = 0.0;
        std::vector<float> componentFunction(componentMesh.vertexNumber(), nvl::maxLimitValue<float>());

        for (VertexId vId = 0; vId < componentMesh.nextVertexId(); ++vId) {
            if (componentMesh.isVertexDeleted(vId))
                continue;

            assert(componentBirthVertex[vId] != nvl::MAX_ID);
            componentFunction[vId] = std::max(std::min(1.0f - jointFunction[componentBirthVertex[vId]], 1.0), 0.0);
            minFunction = nvl::min(minFunction, componentFunction[vId]);
            maxFunction = nvl::max(maxFunction, componentFunction[vId]);
            avgFunction += componentFunction[vId];
        }
        avgFunction /= componentMesh.vertexNumber();

        const double minRigidityLimit = 1.0 - rigidity;
        const double maxRigidityLimit = 1.0 - minRigidityLimit;

        bool isRigid = minFunction >= minRigidityLimit || maxFunction <= maxRigidityLimit;

        for (VertexId vId = 0; vId < componentMesh.nextVertexId(); vId++) {
            if (componentMesh.isVertexDeleted(vId))
                continue;

            if (!isRigid) {
                vertexSelectValue[componentBirthVertex[vId]] = std::min(vertexSelectValue[componentBirthVertex[vId]], componentFunction[vId]);
            }
            else {
                vertexSelectValue[componentBirthVertex[vId]] = std::min(vertexSelectValue[componentBirthVertex[vId]], (avgFunction >= 0.5f ? 1.0f : -1.0f));
            }
        }
    }

    std::vector<JointId> removedJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);
    for (JointId jId : removedJoints) {
        jointSelectValue[jId] = false;
    }
}

template<class Model>
void computeDetachSelectValues(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const unsigned int functionSmoothingIterations,
        const double offset,
        const double rigidity,
        std::vector<float>& vertexSelectValue,
        std::vector<bool>& jointSelectValue)
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

    vertexSelectValue.resize(mesh.nextVertexId(), 1.0);
    jointSelectValue.resize(skeleton.jointNumber(), true);

    //Get connected components
    std::vector<FaceId> faceComponentMap;
    std::vector<std::vector<FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, faceComponentMap);

    //Get function
    std::vector<SkinningWeightsScalar> jointFunction =
            skinmixer::jointJunctionFunction(
                model,
                targetJoint,
                functionSmoothingIterations);

    for (const std::vector<FaceId>& componentFaces : connectedComponents) {
        //Create connected component mesh
        Mesh componentMesh;
        std::vector<VertexId> componentBirthVertex;
        std::vector<FaceId> componentBirthFace;
        nvl::meshTransferFaces(mesh, componentFaces, componentMesh, componentBirthVertex, componentBirthFace);

        float minFunction = nvl::maxLimitValue<float>();
        float maxFunction = nvl::minLimitValue<float>();
        float avgFunction = 0.0;
        std::vector<float> componentFunction(componentMesh.vertexNumber(), nvl::maxLimitValue<float>());

        for (VertexId vId = 0; vId < componentMesh.nextVertexId(); ++vId) {
            if (componentMesh.isVertexDeleted(vId))
                continue;

            assert(componentBirthVertex[vId] != nvl::MAX_ID);
            componentFunction[vId] = std::max(std::min(jointFunction[componentBirthVertex[vId]], 1.0), 0.0);
            minFunction = nvl::min(minFunction, componentFunction[vId]);
            maxFunction = nvl::max(maxFunction, componentFunction[vId]);
            avgFunction += componentFunction[vId];
        }
        avgFunction /= componentMesh.vertexNumber();

        const double minRigidityLimit = 1.0 - rigidity;
        const double maxRigidityLimit = 1.0 - minRigidityLimit;

        bool isRigid = minFunction >= minRigidityLimit || maxFunction <= maxRigidityLimit;

        for (VertexId vId = 0; vId < componentMesh.nextVertexId(); vId++) {
            if (componentMesh.isVertexDeleted(vId))
                continue;

            if (!isRigid) {
                vertexSelectValue[componentBirthVertex[vId]] = std::min(vertexSelectValue[componentBirthVertex[vId]], componentFunction[vId]);
            }
            else {
                vertexSelectValue[componentBirthVertex[vId]] = std::min(vertexSelectValue[componentBirthVertex[vId]], (avgFunction >= 0.5f ? 1.0f : -1.0f));
            }
        }
    }

    std::vector<JointId> removedJoints = nvl::skeletonJointNonDescendants(skeleton, targetJoint);
    for (JointId jId : removedJoints) {
        jointSelectValue[jId] = false;
    }
}

}
