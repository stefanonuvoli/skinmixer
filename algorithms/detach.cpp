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
#include <nvl/math/numeric_limits.h>

namespace skinmixer {

template<class Model>
std::vector<Model> detachBySkinningWeightFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth,
        std::vector<nvl::Segment<typename Model::Mesh::Point>>& curveCoordinates,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& birthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& birthFace,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& birthJoint)
{
    typedef typename Model::Mesh Mesh;

    Mesh resultMesh;
    std::vector<int> faceSegmentation;
    std::vector<int> jointSegmentation;

    return detachBySkinningWeightFunction(
                model,
                targetJoint,
                offset,
                keepEntireSkeleton,
                smooth,
                curveCoordinates,
                birthVertex,
                birthFace,
                birthJoint,
                resultMesh,
                faceSegmentation,
                jointSegmentation);
}

template<class Model>
std::vector<Model> detachBySkinningWeightFunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const bool keepEntireSkeleton,
        const bool smooth,
        std::vector<nvl::Segment<typename Model::Mesh::Point>>& curveCoordinates,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& birthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& birthFace,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& birthJoint,
        typename Model::Mesh& newMesh,
        std::vector<int>& faceSegmentation,
        std::vector<int>& jointSegmentation)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    const SkinningWeightsScalar rigidityLimit = 0.7;

    std::vector<Model> result;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    std::vector<JointId> descendandJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);

    std::vector<SkinningWeightsScalar> vertexFunction(mesh.vertexNumber(), nvl::maxLimitValue<SkinningWeightsScalar>());

    //Create function
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        vertexFunction[vId] = skinningWeights.weight(vId, targetJoint) - 0.5 + offset;
    }
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        for (JointId jId : descendandJoints) {
            vertexFunction[vId] += skinningWeights.weight(vId, jId);
        }
    }
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        vertexFunction[vId] = std::min(std::max(vertexFunction[vId] * 2.0, -1.0), 1.0);
    }

    //Smooth function? Not useful!
//    std::vector<std::vector<VertexId>> vvAdj = nvl::meshVertexVertexAdjacencies(mesh);
//    nvl::laplacianSmoothing(vertexFunction, vvAdj, 10, 0.8);

    std::vector<std::vector<FaceId>> ffAdj = nvl::meshFaceFaceAdjacencies(mesh);

    std::vector<FaceId> faceComponentMap;
    std::vector<std::vector<FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, ffAdj, faceComponentMap);

    bool meshHasBeenSplitted = false;

    //Resulting mesh
    std::vector<VertexId> resultBirthVertex;
    std::vector<FaceId> resultBirthFace;

    //Copy vertices
    nvl::meshTransferVertices(mesh, newMesh, resultBirthVertex);
    std::vector<VertexId> resultVertexMap = nvl::getInverseMap(resultBirthVertex);

    std::vector<std::pair<VertexId, VertexId>> curveVertices;
    for (const std::vector<FaceId>& componentFaces : connectedComponents) {
        //Create connected component mesh
        Mesh componentMesh;
        std::vector<VertexId> transferBirthVertex;
        std::vector<FaceId> transferBirthFace;
        nvl::meshTransferFaces(mesh, componentFaces, componentMesh, transferBirthVertex, transferBirthFace);

        //Transfer the function
        SkinningWeightsScalar minFunction = nvl::maxLimitValue<SkinningWeightsScalar>();
        SkinningWeightsScalar maxFunction = nvl::minLimitValue<SkinningWeightsScalar>();
        std::vector<SkinningWeightsScalar> componentFunction(componentMesh.vertexNumber(), nvl::maxLimitValue<SkinningWeightsScalar>());
        for (VertexId vId = 0; vId < componentMesh.nextVertexId(); ++vId) {
            if (componentMesh.isVertexDeleted(vId))
                continue;

            assert(transferBirthVertex[vId] != nvl::MAX_ID);
            componentFunction[vId] = vertexFunction[transferBirthVertex[vId]];
            minFunction = nvl::min(minFunction, componentFunction[vId]);
            maxFunction = nvl::max(maxFunction, componentFunction[vId]);
        }

        //Refine using function
        Mesh refinedMesh;
        std::vector<VertexId> refineBirthVertex;
        std::vector<FaceId> refineBirthFace;
        std::vector<std::pair<VertexId, VertexId>> refineCurveVertices;

        //Refine if rigid component
        if (minFunction <= -rigidityLimit && maxFunction >= rigidityLimit) {
            if (smooth) {
                std::vector<nvl::Segment<typename Model::Mesh::Point>> functionCoordinates;
                nvl::meshImplicitFunction(componentMesh, componentFunction, functionCoordinates);
                nvl::curveOnManifold(componentMesh, functionCoordinates, refinedMesh, curveCoordinates, refineCurveVertices, refineBirthVertex, refineBirthFace, 40, 20, 0.05, false, false);
            }
            else {
                nvl::refineByImplicitFunction(componentMesh, componentFunction, refinedMesh, curveCoordinates, refineCurveVertices, refineBirthVertex, refineBirthFace);
            }
        }
        //Non rigid function
        else {
            nvl::meshTransferFaces(componentMesh, refinedMesh, refineBirthVertex, refineBirthFace);
        }

        //Get birth vertices and faces related to mesh
        std::vector<VertexId> currentBirthVertex(refineBirthVertex.size(), nvl::MAX_ID);
        for (VertexId vId = 0; vId < refineBirthVertex.size(); vId++) {
            if (refineBirthVertex[vId] != nvl::MAX_ID) {
                currentBirthVertex[vId] = transferBirthVertex[refineBirthVertex[vId]];
            }
        }
        std::vector<FaceId> currentBirthFace(refineBirthFace.size(), nvl::MAX_ID);
        for (FaceId fId = 0; fId < refineBirthFace.size(); fId++) {
            if (refineBirthFace[fId] != nvl::MAX_ID) {
                currentBirthFace[fId] = transferBirthFace[refineBirthFace[fId]];
            }
        }

        //Get vertex map
        std::vector<VertexId> vertexMap(refinedMesh.nextVertexId(), nvl::MAX_ID);

        //Add new vertices to the result mesh
        for (VertexId vId = 0; vId < refinedMesh.nextVertexId(); ++vId) {
            if (refinedMesh.isVertexDeleted(vId))
                continue;

            const Vertex& vertex = refinedMesh.vertex(vId);

            if (currentBirthVertex[vId] == nvl::MAX_ID) {
                VertexId newVId = newMesh.addVertex(vertex);

                vertexMap[vId] = newVId;

                assert(resultBirthVertex.size() == newVId);
                resultBirthVertex.push_back(nvl::MAX_ID);
            }
            else {
                assert(currentBirthVertex[vId] != nvl::MAX_ID);
                assert(resultVertexMap[currentBirthVertex[vId]] != nvl::MAX_ID);
                vertexMap[vId] = resultVertexMap[currentBirthVertex[vId]];
            }
        }

        //Add faces
        for (FaceId fId = 0; fId < refinedMesh.nextFaceId(); ++fId) {
            if (refinedMesh.isFaceDeleted(fId))
                continue;

            const Face& face = refinedMesh.face(fId);

            std::vector<VertexId> newVertices(face.vertexNumber());

            for (VertexId j = 0; j < face.vertexNumber(); ++j) {
                const VertexId vId = vertexMap[face.vertexId(j)];
                assert(vId != nvl::MAX_ID);
                newVertices[j] = vId;
            }

            FaceId newFId = newMesh.addFace(newVertices);

            assert(resultBirthFace.size() == newFId);
            resultBirthFace.push_back(currentBirthFace[fId]);
        }

        if (refineCurveVertices.size() >= 3) {
            meshHasBeenSplitted = true;
            for (std::pair<VertexId, VertexId>& vertices : refineCurveVertices) {
                curveVertices.push_back(
                            std::make_pair(
                                vertexMap[vertices.first],
                                vertexMap[vertices.second]));
            }
        }
    }

    if (meshHasBeenSplitted) {
        jointSegmentation.resize(skeleton.jointNumber(), 0);
        jointSegmentation[targetJoint] = 1;
        for (JointId jId : descendandJoints) {
            jointSegmentation[jId] = 1;
        }

        std::vector<std::vector<FaceId>> ffAdj = nvl::meshFaceFaceAdjacencies(newMesh);

        faceSegmentation.resize(newMesh.nextFaceId(), -1);

        std::set<std::pair<VertexId, VertexId>> detachingVerticesSet(curveVertices.begin(), curveVertices.end());
        bool segmentationDone;
        do {
            segmentationDone = true;

            size_t maxFId = nvl::MAX_ID;
            size_t minFId = nvl::MAX_ID;
            double maxFunction = nvl::minLimitValue<SkinningWeightsScalar>();
            for (FaceId i = 0; i < newMesh.nextFaceId(); ++i) {
                if (newMesh.isFaceDeleted(i))
                    continue;

                if (faceSegmentation[i] == -1) {
                    segmentationDone = false;

                    const Face& face = newMesh.face(i);

                    double functionSum = 0;
                    for (FaceId j = 0; j < face.vertexNumber(); ++j) {
                        VertexId birthVId = resultBirthVertex[face.vertexId(j)];

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

                std::vector<bool> visited(newMesh.nextFaceId(), false);
                while (!stack.empty()) {
                    FaceId fId = stack.top();
                    stack.pop();

                    if (visited[fId])
                        continue;

                    visited[fId] = true;

                    assert(faceSegmentation[fId] == label || faceSegmentation[fId] == -1);
                    faceSegmentation[fId] = label;

                    const Face& face = newMesh.face(fId);

                    std::vector<VertexId> faceVertices = face.vertexIds();
                    std::sort(faceVertices.begin(), faceVertices.end());

                    std::vector<FaceId>& adjacentFaces = ffAdj[fId];
                    for (FaceId adjId : adjacentFaces) {
                        const Face& adjFace = newMesh.face(adjId);

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
            for (FaceId fId = 0; fId < newMesh.nextFaceId(); ++fId) {
                if (newMesh.isFaceDeleted(fId))
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
                std::vector<VertexId> currentBirthVertex;
                std::vector<FaceId> currentBirthFace;
                std::vector<JointId> currentBirthJoint;

                Model newModel;
                nvl::meshTransferFaces(newMesh, faces, newModel.mesh, currentBirthVertex, currentBirthFace);

                for (VertexId& vId : currentBirthVertex) {
                    if (vId != nvl::MAX_ID) {
                        vId = resultBirthVertex[vId];
                    }
                }
                for (FaceId& fId : currentBirthFace) {
                    if (fId != nvl::MAX_ID) {
                        fId = resultBirthFace[fId];
                    }
                }

                nvl::skeletonTransferJoints(model.skeleton, joints, newModel.skeleton, currentBirthJoint);
                nvl::modelSkinningWeightsTransfer(model, currentBirthVertex, currentBirthJoint, newModel);
                nvl::modelAnimationTransfer(model, currentBirthJoint, newModel);

                result.push_back(newModel);
                birthVertex.push_back(currentBirthVertex);
                birthFace.push_back(currentBirthFace);
                birthJoint.push_back(currentBirthJoint);
            }
        }
    }

    return result;
}

}
