#include "detach.h"

#include <limits>

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/skeleton_transfer.h>
#include <nvl/models/model_transfer.h>
#include <nvl/models/skeleton_adjacencies.h>
#include <nvl/models/mesh_normals.h>
#include <nvl/models/mesh_implicit_function.h>

#include <nvl/vcglib/curve_on_manifold.h>
#include <nvl/vcglib/mesh_refine.h>

#include <nvl/math/comparisons.h>
#include <nvl/math/laplacian.h>

namespace skinmixer {

//template<class Model>
//std::vector<Model> detachBySkeletonSegmentation(
//        const Model& model,
//        const typename Model::Skeleton::JointId& targetJoint,
//        const float compactness,
//        const bool keepEntireSkeleton,
//        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
//        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
//        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps)
//{
//    typedef typename Model::Mesh Mesh;
//    typedef typename Mesh::VertexId VertexId;
//    typedef typename Mesh::FaceId FaceId;
//    typedef typename Model::Skeleton Skeleton;
//    typedef typename Skeleton::JointId JointId;

//    std::vector<Model> result;

//    const Mesh& mesh = model.mesh;
//    const Skeleton& skeleton = model.skeleton;

//    //Get segmentation
//    std::vector<int> jointSegmentation;
//    std::vector<int> faceSegmentation = skinmixer::skeletonBinarySegmentationGraphcut(model, compactness, targetJoint, jointSegmentation);

//    int maxLabel = -std::numeric_limits<int>::max();
//    for (const int& label : faceSegmentation) {
//        maxLabel = std::max(maxLabel, label);
//    }

//    for (int l = 0; l <= maxLabel; ++l) {
//        std::vector<FaceId> faces;
//        for (FaceId fId = 0; fId < mesh.faceNumber(); ++fId) {
//            if (mesh.isFaceDeleted(fId))
//                continue;

//            if (faceSegmentation[fId] == l) {
//                faces.push_back(fId);
//            }
//        }

//        std::vector<JointId> joints;
//        for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
//            if (keepEntireSkeleton || jointSegmentation[jId] == l) {
//                joints.push_back(jId);
//            }
//        }

//        if (std::find(joints.begin(), joints.end(), targetJoint) == joints.end()) {
//            joints.push_back(targetJoint);
//        }

//        if (!faces.empty()) {
//            std::vector<VertexId> vertexMap;
//            std::vector<FaceId> faceMap;
//            std::vector<JointId> jointMap;

//            Model newModel;
//            nvl::modelTransfer(model, faces, joints, newModel, vertexMap, faceMap, jointMap);

//            result.push_back(newModel);
//            vertexMaps.push_back(vertexMap);
//            faceMaps.push_back(faceMap);
//            jointMaps.push_back(jointMap);
//        }
//    }

//    return result;
//}

template<class Model>
std::vector<Model> detachBySkinningWeightFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth,
        std::vector<nvl::Segment<typename Model::Mesh::Point>>& curveCoordinates,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& vertexMaps,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& faceMaps,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& jointMaps)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    std::vector<Model> result;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    std::vector<JointId> descendandJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);

    std::vector<SkinningWeightsScalar> vertexFunction(mesh.vertexNumber(), 0.0);

    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        vertexFunction[vId] = skinningWeights.weight(vId, targetJoint) - 0.5 + offset;
    }
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        for (JointId jId : descendandJoints) {
            vertexFunction[vId] += skinningWeights.weight(vId, jId);
        }
    }
//    std::vector<std::vector<nvl::Index>> vvAdj = nvl::meshVertexVertexAdjacencies(mesh);
//    nvl::laplacianSmoothing(vertexFunction, vvAdj, 10, 0.8);

    Mesh refinedMesh;

    std::vector<std::pair<typename Model::Mesh::VertexId, typename Model::Mesh::VertexId>> curveVertices;

    std::vector<nvl::Index> refineBirthVertex;
    std::vector<nvl::Index> refineBirthFace;
    if (smooth) {
        std::vector<nvl::Segment<typename Model::Mesh::Point>> functionCoordinates;
        nvl::meshImplicitFunction(mesh, vertexFunction, functionCoordinates);
        nvl::curveOnManifold(mesh, functionCoordinates, refinedMesh, curveCoordinates, curveVertices, refineBirthVertex, refineBirthFace, 40, 20, 0.05, false, false);
    }
    else {
        nvl::refineByImplicitFunction(mesh, vertexFunction, refinedMesh, curveCoordinates, curveVertices, refineBirthVertex, refineBirthFace);
    }

    if (curveVertices.size() >= 3) {
        std::vector<int> jointSegmentation(skeleton.jointNumber(), 0);
        jointSegmentation[targetJoint] = 1;
        for (JointId jId : descendandJoints) {
            jointSegmentation[jId] = 1;
        }

        std::vector<std::vector<nvl::Index>> ffAdj = nvl::meshFaceFaceAdjacencies(refinedMesh);
        std::set<std::pair<VertexId, VertexId>> detachingVerticesSet(curveVertices.begin(), curveVertices.end());
        std::vector<int> faceSegmentation(refinedMesh.faceNumber(), -1);

        bool segmentationDone;
        do {
            segmentationDone = true;

            size_t maxFId = nvl::MAX_ID;
            size_t minFId = nvl::MAX_ID;
            double maxFunction = -std::numeric_limits<double>::max();
            for (FaceId i = 0; i < refinedMesh.faceNumber(); ++i) {
                if (refinedMesh.isFaceDeleted(i))
                    continue;

                if (faceSegmentation[i] == -1) {
                    segmentationDone = false;

                    const Face& face = refinedMesh.face(i);

                    double functionSum = 0;
                    for (FaceId j = 0; j < face.vertexNumber(); ++j) {
                        VertexId birthVId = refineBirthVertex[face.vertexId(j)];

                        if (birthVId != nvl::MAX_ID) {
                            functionSum += vertexFunction[birthVId];
                        }
                    }

                    if (functionSum < 0 && -functionSum >= maxFunction) {
                        maxFunction = -functionSum;
                        minFId = i;
                        maxFId = nvl::MAX_ID;
                    }
                    else if (functionSum > 0 && functionSum >= maxFunction) {
                        maxFunction = functionSum;
                        maxFId = i;
                        minFId = nvl::MAX_ID;
                    }
                }
            }

            if (!segmentationDone) {
                FaceId currentFaceId = nvl::MAX_ID;
                if (maxFId != nvl::MAX_ID) {
                    assert(minFId == nvl::MAX_ID);
                    faceSegmentation[maxFId] = 1;
                    currentFaceId = maxFId;
                }
                else if (minFId != nvl::MAX_ID) {
                    faceSegmentation[minFId] = 0;
                    currentFaceId = minFId;
                }
                assert(currentFaceId != nvl::MAX_ID);

                int label = faceSegmentation[currentFaceId];
                assert(label != -1);

                std::stack<FaceId> stack;
                stack.push(currentFaceId);

                std::vector<bool> visited(refinedMesh.faceNumber(), false);
                while (!stack.empty()) {
                    FaceId fId = stack.top();
                    stack.pop();

                    if (visited[fId])
                        continue;

                    visited[fId] = true;

                    assert(faceSegmentation[fId] == label || faceSegmentation[fId] == -1);
                    faceSegmentation[fId] = label;

                    const Face& face = refinedMesh.face(fId);

                    std::vector<VertexId> faceVertices = face.vertexIds();
                    std::sort(faceVertices.begin(), faceVertices.end());

                    std::vector<FaceId>& adjacentFaces = ffAdj[fId];
                    for (FaceId adjId : adjacentFaces) {
                        const Face& adjFace = refinedMesh.face(adjId);

                        std::vector<VertexId> adjVertices = adjFace.vertexIds();
                        std::sort(adjVertices.begin(), adjVertices.end());

                        std::vector<VertexId> intersection;
                        std::set_intersection(faceVertices.begin(), faceVertices.end(), adjVertices.begin(), adjVertices.end(), std::back_inserter(intersection));

                        if (intersection.size() == 2) {
                            std::pair<VertexId, VertexId> vertices(std::min(intersection[0], intersection[1]), std::max(intersection[0], intersection[1]));
                            if (detachingVerticesSet.find(vertices) == detachingVerticesSet.end()) {
                                stack.push(adjId);
                            }
                        }
                        else {
                            std::cout << "Warning: intersection between two adjacent faces is not composed of two vertices (duplicate face?)." << std::endl;
                        }
                    }
                }

            }
        } while (!segmentationDone);

        for (int l = 0; l <= 1; ++l) {
            std::vector<FaceId> faces;
            for (FaceId fId = 0; fId < refinedMesh.faceNumber(); ++fId) {
                if (refinedMesh.isFaceDeleted(fId))
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
                nvl::meshTransferFaces(refinedMesh, faces, newModel.mesh, vertexMap, faceMap);

                for (VertexId& vId : vertexMap) {
                    if (vId != nvl::MAX_ID) {
                        vId = refineBirthVertex[vId];
                    }
                }
                for (FaceId& fId : faceMap) {
                    if (fId != nvl::MAX_ID) {
                        fId = refineBirthFace[fId];
                    }
                }

                nvl::skeletonTransferJoints(model.skeleton, joints, newModel.skeleton, jointMap);
                nvl::modelSkinningWeightsTransfer(model, vertexMap, jointMap, newModel);
                nvl::modelAnimationTransfer(model, jointMap, newModel);

                result.push_back(newModel);
                vertexMaps.push_back(vertexMap);
                faceMaps.push_back(faceMap);
                jointMaps.push_back(jointMap);
            }
        }
    }

    return result;
}


}
