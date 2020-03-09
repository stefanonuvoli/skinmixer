#include "detach.h"

#include <limits>

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/skeleton_transfer.h>

#include <nvl/models/model_transfer.h>

#include "algorithms/skeleton_segmentation.h"

namespace skinmixer {

template<class Model>
std::vector<Model> detachBySkeletonSegmentation(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        float compactness,
        bool keepEntireSkeleton,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;

    std::vector<Model> result;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;

    //Get segmentation
    std::vector<int> jointSegmentation;
    std::vector<int> faceSegmentation = skinmixer::skeletonBinarySegmentationGraphcut(model, compactness, targetJoint, jointSegmentation);

    int maxLabel = -std::numeric_limits<int>::max();
    for (const int& label : faceSegmentation) {
        maxLabel = std::max(maxLabel, label);
    }

    for (int l = 0; l <= maxLabel; ++l) {
        std::vector<FaceId> faces;
        for (FaceId fId = 0; fId < mesh.faceNumber(); ++fId) {
            if (mesh.isFaceDeleted(fId))
                continue;

            if (faceSegmentation[fId] == l) {
                faces.push_back(fId);
            }
        }

        std::vector<JointId> joints;
        for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
            if (keepEntireSkeleton || jointSegmentation[jId] == l) {
                joints.push_back(jId);
            }
        }

        if (std::find(joints.begin(), joints.end(), targetJoint) == joints.end()) {
            joints.push_back(targetJoint);
        }

        if (!faces.empty()) {
            std::vector<VertexId> vertexMap;
            std::vector<FaceId> faceMap;
            std::vector<JointId> jointMap;

            Model newModel;
            nvl::modelTransfer(model, faces, joints, newModel, vertexMap, faceMap, jointMap);

            result.push_back(newModel);
            vertexMaps.push_back(vertexMap);
            faceMaps.push_back(faceMap);
            jointMaps.push_back(jointMap);
        }
    }

    return result;
}

template<class Model>
std::vector<Model> detachByImplicitFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        bool keepEntireSkeleton,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps)
{

}


}
