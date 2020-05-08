#include "skinmixer_blend_weights.h"

#include <nvl/math/numeric_limits.h>

#include <nvl/vcglib/vcg_grid.h>

#include <nvl/models/model_normalization.h>

#include "skinmixer/skinmixer_graph_algorithms.h"

#include <unordered_map>

namespace skinmixer {

template<class Model>
void blendSkinningWeights(
        OperationGraph<Model>& operationGraph,
        const nvl::Index& nodeId)
{
    typedef nvl::Index Index;
    typedef OperationGraph<Model> Graph;
    typedef typename Graph::Node Node;
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::Point Point;
    typedef typename Skeleton::JointId JointId;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;
    typedef nvl::VCGGrid<Mesh> Grid;

    Node& node = operationGraph.node(nodeId);

    if (operationGraph.node(nodeId).parents.empty()) {
        return;
    }

    Model* model = node.model;

    const Mesh& mesh = model->mesh;
    const Skeleton& skeleton = model->skeleton;
    SkinningWeights& skinningWeights = model->skinningWeights;

    std::vector<Grid*> grids;
    std::unordered_map<Index, Grid*> nodeGridMap;

    std::vector<Index> vertexBirthNode(mesh.nextVertexId(), nvl::MAX_ID);
    std::vector<VertexId> birthVertex(mesh.nextVertexId(), nvl::MAX_ID);
    std::vector<bool> isNewVertex(mesh.nextVertexId(), false);
    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        isNewVertex[vId] = findVertexBirthNode(operationGraph, nodeId, vId, vertexBirthNode[vId], birthVertex[vId]);

        if (!isNewVertex[vId]) {
            assert(vertexBirthNode[vId] != nvl::MAX_ID);

            if (nodeGridMap.find(vertexBirthNode[vId]) == nodeGridMap.end()) {
                Model* nodeModel = operationGraph.node(vertexBirthNode[vId]).model;
                Grid* grid = new Grid(nodeModel->mesh);

                grids.push_back(grid);

                nodeGridMap.insert(std::make_pair(vertexBirthNode[vId], grid));
            }
        }
    }

    std::vector<Index> jointBirthNode(skeleton.jointNumber(), nvl::MAX_ID);
    std::vector<JointId> birthJoint(skeleton.jointNumber(), nvl::MAX_ID);
    std::vector<bool> isNewJoint(skeleton.jointNumber(), false);
    for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
        isNewJoint[jId] = findJointBirthNode(operationGraph, nodeId, jId, jointBirthNode[jId], birthJoint[jId]);
    }

    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        if (!isNewVertex[vId])
            continue;

        assert(vertexBirthNode[vId] == nvl::MAX_ID);
        assert(birthVertex[vId] == nvl::MAX_ID);

        const Point point = mesh.vertex(vId).point();

        Index bestRootNode = nvl::MAX_ID;
        FaceId bestTriangleId = nvl::MAX_ID;
        Point bestBarycentricCoordinates(nvl::maxLimitValue<Scalar>(), nvl::maxLimitValue<Scalar>(), nvl::maxLimitValue<Scalar>());
        Scalar bestDistance = nvl::maxLimitValue<Scalar>();

        for (std::pair<Index, Grid*> entry : nodeGridMap) {
            Index currentRootNode = entry.first;
            Grid* currentGrid = entry.second;

            FaceId faceId;
            Point closestPoint;
            FaceId triangleFaceId;
            Point barycentricCoordinates;

            Point originalPosition;
            bool foundOriginalPosition = findOriginalPosition(operationGraph, nodeId, currentRootNode, point, originalPosition);
            assert(foundOriginalPosition);

            faceId = currentGrid->getClosestFace(originalPosition, closestPoint, triangleFaceId, barycentricCoordinates);

            Scalar distance = (originalPosition - closestPoint).norm();

            if (distance < bestDistance) {
                bestRootNode = currentRootNode;
                bestTriangleId = faceId;

                bestBarycentricCoordinates = barycentricCoordinates;
            }
        }

        assert(bestTriangleId != nvl::MAX_ID);
        assert(bestRootNode != nvl::MAX_ID);

        for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
            if (jointBirthNode[jId] == bestRootNode) {
                const Mesh& triangulatedMesh = nodeGridMap.at(bestRootNode)->triangulatedMesh();
                const Face& triangleFace = triangulatedMesh.face(bestTriangleId);

                assert(triangleFace.vertexNumber() == 3);
                VertexId v1 = triangleFace.vertexId(0);
                VertexId v2 = triangleFace.vertexId(1);
                VertexId v3 = triangleFace.vertexId(2);

                const SkinningWeights& rootSkinningWeights = operationGraph.node(bestRootNode).model->skinningWeights;

                SkinningWeightsScalar blendedSkinningWeight =
                        rootSkinningWeights.weight(v1, birthJoint[jId]) * bestBarycentricCoordinates(0) +
                        rootSkinningWeights.weight(v2, birthJoint[jId]) * bestBarycentricCoordinates(1) +
                        rootSkinningWeights.weight(v3, birthJoint[jId]) * bestBarycentricCoordinates(2);

                if (blendedSkinningWeight >= +nvl::EPSILON) {
                    skinningWeights.setWeight(vId, jId, blendedSkinningWeight);
                }
            }
        }
    }

    skinningWeights.updateNonZeros();

    for (Grid*& grid : grids) {
        delete grid;
        grid = nullptr;
    }

    nvl::modelNormalizeSkinningWeights(*model);
}

}
