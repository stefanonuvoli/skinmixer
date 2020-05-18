#include "skinmixer_cut.h"

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

#include "skinmixer/skinmixer_blend_weights.h"

namespace skinmixer {

template<class Model>
std::vector<nvl::Index> cutOperation(
        OperationGraph<Model>& operationGraph,
        const nvl::Index& nodeId,
        const Operation& operation,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        const bool keepEntireSkeleton)
{
    typedef OperationGraphNode<Model> Node;
    typedef nvl::Index Index;
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    assert(operation == Operation::DETACH || operation == Operation::REMOVE || operation == Operation::SPLIT);

    //Input
    const Model& model = *(operationGraph.node(nodeId).model);

    //Output
    std::vector<nvl::Index> newNodes;

    //Data for cut model along joint junction
    Mesh resultMesh;
    std::vector<int> faceSegmentation;
    std::vector<int> jointSegmentation;
    std::vector<std::vector<VertexId>> birthVertex;
    std::vector<std::vector<FaceId>> birthFace;
    std::vector<std::vector<JointId>> birthJoint;
    std::vector<std::vector<std::pair<VertexId, VertexId>>> cutLines;
    std::vector<SkinningWeightsScalar> cutFunction;

    //Cut model along joint junction
    std::vector<Model> newModels = cutAlongJointJunction(
                model,
                targetJoint,
                offset,
                rigidity,
                smooth,
                keepEntireSkeleton,
                resultMesh,
                cutLines,
                cutFunction,
                birthVertex,
                birthFace,
                birthJoint,
                faceSegmentation,
                jointSegmentation);

    assert(newModels.size() == 0 || newModels.size() == 2);

    if (!newModels.empty()) {
        //Indices depending on operator
        Index iStart = 0, iEnd = newModels.size();
        if (operation == Operation::DETACH) {
            iStart = 1;
        }
        if (operation == Operation::REMOVE) {
            iEnd = 1;
        }

        //Update graph
        for (Index i = iStart; i < iEnd; ++i) {
            const Model& model = newModels[i];
            const Skeleton& skeleton = model.skeleton;
            const Mesh& mesh = model.mesh;

            Node node;
            node.model = new Model(model);
            node.operation = operation;

            node.parents.push_back(nodeId);

            node.birthVertex = birthVertex[i];
            node.birthFace = birthFace[i];
            node.birthJoint = birthJoint[i];

            node.birthVertexParentNodeId = std::vector<Index>(node.birthVertex.size(), 0);
            node.birthFaceParentNodeId = std::vector<Index>(node.birthFace.size(), 0);
            node.birthJointParentNodeId = std::vector<Index>(node.birthJoint.size(), 0);

            for (JointId jId = 0; jId < skeleton.jointNumber(); ++jId) {
                if (node.birthJoint[jId] == targetJoint) {
                    node.vCutJointId = jId;
                    break;
                }
            }
            for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
                if (node.birthVertex[vId] == nvl::MAX_ID) {
                    node.vCutVertices.push_back(vId);
                }
            }

            size_t newNodeId = operationGraph.addNode(node);
            newNodes.push_back(newNodeId);
        }

        Node& parentNode = operationGraph.node(nodeId);
        for (Index newNodeId : newNodes) {
            blendSkinningWeights(operationGraph, newNodeId);
            parentNode.children.push_back(newNodeId);
        }
    }

    return newNodes;
}

template<class Model>
std::vector<std::vector<nvl::Segment<typename Model::Mesh::Point>>> cutOperationPreview(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        std::vector<typename Model::SkinningWeights::Scalar>& cutFunction)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename nvl::Segment<typename Mesh::Point> Segment;

    //Output
    std::vector<std::vector<Segment>> cutSegments;

    //Data for joint junction cut
    Mesh resultMesh;
    std::vector<VertexId> resultBirthVertex;
    std::vector<FaceId> resultBirthFace;
    std::vector<std::vector<std::pair<VertexId, VertexId>>> cutLines;

    //Refine along joint junction
    internal::refineAlongJointJunction(
                model,
                targetJoint,
                offset,
                rigidity,
                smooth,
                resultMesh,
                cutLines,
                cutFunction,
                resultBirthVertex,
                resultBirthFace);

    //Add cut segments
    for (const std::vector<std::pair<VertexId, VertexId>>& line : cutLines) {
        std::vector<Segment> segments;
        for (const std::pair<VertexId, VertexId>& edge : line) {
            segments.push_back(Segment(
                 resultMesh.vertex(edge.first).point(),
                 resultMesh.vertex(edge.second).point()
            ));
        }

        cutSegments.push_back(segments);
    }

    return cutSegments;
}

template<class Model>
std::vector<Model> cutAlongJointJunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        const bool keepEntireSkeleton,
        typename Model::Mesh& resultMesh,
        std::vector<std::vector<std::pair<typename Model::Mesh::VertexId, typename Model::Mesh::VertexId>>>& cutLines,
        std::vector<typename Model::SkinningWeights::Scalar>& cutFunction,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& birthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& birthFace,
        std::vector<std::vector<typename Model::Skeleton::JointId>>& birthJoint,
        std::vector<int>& faceSegmentation,
        std::vector<int>& jointSegmentation)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    //Output
    std::vector<Model> result;

    //Input
    const Skeleton& skeleton = model.skeleton;

    //Descendant joints

    //Data for joint junction cut
    std::vector<VertexId> resultBirthVertex;
    std::vector<FaceId> resultBirthFace;

    //Cut along joint junction
    internal::refineAlongJointJunction(model, targetJoint, offset, rigidity, smooth, resultMesh, cutLines, cutFunction, resultBirthVertex, resultBirthFace);

    if (!cutLines.empty()) {
        std::vector<JointId> descendandJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);
        std::vector<std::vector<FaceId>> ffAdj = nvl::meshFaceFaceAdjacencies(resultMesh);

        //Descendant labeled as 1, other ones as 0
        jointSegmentation.resize(skeleton.jointNumber(), 0);
        jointSegmentation[targetJoint] = 1;
        for (JointId jId : descendandJoints) {
            jointSegmentation[jId] = 1;
        }

        //Initial face segmentation setted to -1
        faceSegmentation.resize(resultMesh.nextFaceId(), -1);

        std::set<std::pair<VertexId, VertexId>> detachingVerticesSet;
        for (std::vector<std::pair<VertexId, VertexId>>& line : cutLines) {
            detachingVerticesSet.insert(line.begin(), line.end());
        }

        bool segmentationDone;
        do {
            segmentationDone = true;

            size_t maxFId = nvl::MAX_ID;
            size_t minFId = nvl::MAX_ID;
            double maxFunction = nvl::minLimitValue<SkinningWeightsScalar>();
            for (FaceId i = 0; i < resultMesh.nextFaceId(); ++i) {
                if (resultMesh.isFaceDeleted(i))
                    continue;

                if (faceSegmentation[i] == -1) {
                    segmentationDone = false;

                    const Face& face = resultMesh.face(i);

                    double functionSum = 0;
                    for (FaceId j = 0; j < face.vertexNumber(); ++j) {
                        VertexId birthVId = resultBirthVertex[face.vertexId(j)];

                        if (birthVId != nvl::MAX_ID) {
                            functionSum += cutFunction[birthVId];
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

                std::vector<bool> visited(resultMesh.nextFaceId(), false);
                while (!stack.empty()) {
                    FaceId fId = stack.top();
                    stack.pop();

                    if (visited[fId])
                        continue;

                    visited[fId] = true;

                    assert(faceSegmentation[fId] == label || faceSegmentation[fId] == -1);
                    faceSegmentation[fId] = label;

                    const Face& face = resultMesh.face(fId);

                    std::vector<VertexId> faceVertices = face.vertexIds();
                    std::sort(faceVertices.begin(), faceVertices.end());

                    std::vector<FaceId>& adjacentFaces = ffAdj[fId];
                    for (FaceId adjId : adjacentFaces) {
                        const Face& adjFace = resultMesh.face(adjId);

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
            for (FaceId fId = 0; fId < resultMesh.nextFaceId(); ++fId) {
                if (resultMesh.isFaceDeleted(fId))
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
                nvl::meshTransferFaces(resultMesh, faces, newModel.mesh, currentBirthVertex, currentBirthFace);

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


namespace internal {

template<class Model>
void refineAlongJointJunction(
        const Model& model,
        const typename Model::Skeleton::JointId& targetJoint,
        const double offset,
        const double rigidity,
        const bool smooth,
        typename Model::Mesh& resultMesh,
        std::vector<std::vector<std::pair<typename Model::Mesh::VertexId, typename Model::Mesh::VertexId>>>& cutLines,
        std::vector<typename Model::SkinningWeights::Scalar>& cutFunction,
        std::vector<typename Model::Mesh::VertexId>& resultBirthVertex,
        std::vector<typename Model::Mesh::FaceId>& resultBirthFace)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Model::Mesh::Scalar Scalar;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    //Output
    std::vector<Model> result;

    //Input
    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;

    //Clear data
    resultBirthVertex.clear();
    resultBirthFace.clear();
    resultMesh.clear();
    cutLines.clear();

    //Descendant joints
    std::vector<JointId> descendandJoints = nvl::skeletonJointDescendants(skeleton, targetJoint);

    //Compute cut function
    cutFunction = jointJunctionImplicitFunction(model, targetJoint, descendandJoints);

    if (descendandJoints.empty()) {
        return;
    }

    //Face-face adjancies
    std::vector<std::vector<FaceId>> ffAdj = nvl::meshFaceFaceAdjacencies(mesh);

    //Get connected components
    std::vector<FaceId> faceComponentMap;
    std::vector<std::vector<FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, ffAdj, faceComponentMap);

    //Copy mesh vertices
    nvl::meshTransferVertices(mesh, resultMesh, resultBirthVertex);
    std::vector<VertexId> resultVertexMap = nvl::getInverseMap(resultBirthVertex);


    //Copy vertices
    std::vector<std::pair<VertexId, VertexId>> curveVertices;
    for (const std::vector<FaceId>& componentFaces : connectedComponents) {
        //Create connected component mesh
        Mesh componentMesh;
        std::vector<VertexId> componentBirthVertex;
        std::vector<FaceId> componentBirthFace;
        nvl::meshTransferFaces(mesh, componentFaces, componentMesh, componentBirthVertex, componentBirthFace);

        SkinningWeightsScalar minFunction = nvl::maxLimitValue<SkinningWeightsScalar>();
        SkinningWeightsScalar maxFunction = nvl::minLimitValue<SkinningWeightsScalar>();
        SkinningWeightsScalar avgFunction = 0.0;
        std::vector<SkinningWeightsScalar> componentFunction(componentMesh.vertexNumber(), nvl::maxLimitValue<SkinningWeightsScalar>());

        for (VertexId vId = 0; vId < componentMesh.nextVertexId(); ++vId) {
            if (componentMesh.isVertexDeleted(vId))
                continue;

            assert(componentBirthVertex[vId] != nvl::MAX_ID);
            componentFunction[vId] = cutFunction[componentBirthVertex[vId]];
            minFunction = nvl::min(minFunction, componentFunction[vId]);
            maxFunction = nvl::max(maxFunction, componentFunction[vId]);
            avgFunction += componentFunction[vId];
        }
        avgFunction /= componentMesh.vertexNumber();

        const double minRigidityLimit = -rigidity + nvl::EPSILON;
        const double maxRigidityLimit = rigidity - nvl::EPSILON;

        bool isRigid = minFunction > minRigidityLimit || maxFunction < maxRigidityLimit;

        Mesh componentRefinedMesh;
        std::vector<VertexId> componentRefineBirthVertex;
        std::vector<FaceId> componentRefineBirthFace;
        std::vector<std::pair<VertexId, VertexId>> componentRefineCurveLines;

        //If it is not rigid
        if (!isRigid) {
            //Perform refining over the cut function zeros
            if (smooth) {
                std::vector<nvl::Segment<typename Model::Mesh::Point>> functionCoordinates;
                functionCoordinates = nvl::meshImplicitFunction(componentMesh, componentFunction);
                componentRefinedMesh = nvl::curveOnManifold(componentMesh, functionCoordinates, componentRefineCurveLines, componentRefineBirthVertex, componentRefineBirthFace, 15, 10, 0.05, false, false);
            }
            else {
                componentRefinedMesh = nvl::refineByImplicitFunction(componentMesh, componentFunction, componentRefineCurveLines, componentRefineBirthVertex, componentRefineBirthFace);
            }

            //If there is an offset, perform refining using geodesics
            if (!componentRefineCurveLines.empty() && !nvl::epsEqual(offset, 0.0)) {
                //We use 20% of the bounding box diagonal as maximum movement
                nvl::AlignedBox3d bbox = nvl::meshBoundingBox(mesh);
                const Scalar bboxOffset = offset * bbox.diagonal().norm() * 0.2;

                //Create source vertices (where the mesh has been refined)
                std::vector<VertexId> sourceVertices;
                for (std::pair<VertexId, VertexId> pair : componentRefineCurveLines) {
                    sourceVertices.push_back(pair.first);
                    sourceVertices.push_back(pair.second);
                }
                std::sort(sourceVertices.begin(), sourceVertices.end());
                sourceVertices.erase(unique(sourceVertices.begin(), sourceVertices.end()), sourceVertices.end());

                //Get geodesic function for the component mesh
                std::vector<Scalar> geodesics = nvl::heatGeodesics(componentRefinedMesh, sourceVertices);
                std::vector<Scalar> geodesicsCutFunction(componentMesh.nextVertexId(), nvl::maxLimitValue<double>());
                nvl::Index currentVId = 0;
                for (VertexId vId = 0; vId < componentRefinedMesh.nextVertexId(); ++vId) {
                    if (mesh.isVertexDeleted(vId))
                        continue;

                    //Set offset with sign depending on the cut function
                    if (componentRefineBirthVertex[vId] != nvl::MAX_ID) {
                        assert(currentVId < geodesics.size());
                        if (componentFunction[componentRefineBirthVertex[vId]] >= 0) {
                            geodesicsCutFunction[componentRefineBirthVertex[vId]] = geodesics[currentVId];
                        }
                        else {
                            geodesicsCutFunction[componentRefineBirthVertex[vId]] = -geodesics[currentVId];
                        }

                        geodesicsCutFunction[componentRefineBirthVertex[vId]] += bboxOffset;
                    }

                    currentVId++;
                }

                //Perform geodesics refining
                if (smooth) {
                    std::vector<nvl::Segment<typename Model::Mesh::Point>> geodesicFunctionCoordinates;
                    geodesicFunctionCoordinates = nvl::meshImplicitFunction(componentMesh, geodesicsCutFunction);
                    componentRefinedMesh = nvl::curveOnManifold(componentMesh, geodesicFunctionCoordinates, componentRefineCurveLines, componentRefineBirthVertex, componentRefineBirthFace, 15, 10, 0.05, false, false);
                }
                else {
                    componentRefinedMesh = nvl::refineByImplicitFunction(componentMesh, geodesicsCutFunction, componentRefineCurveLines, componentRefineBirthVertex, componentRefineBirthFace);
                }
            }
        }
        //It is rigid
        else {
            nvl::meshTransferFaces(componentMesh, componentRefinedMesh, componentRefineBirthVertex, componentRefineBirthFace);
        }

        //Get birth vertices and faces related to result mesh
        std::vector<VertexId> currentBirthVertex(componentRefineBirthVertex.size(), nvl::MAX_ID);
        std::vector<FaceId> currentBirthFace(componentRefineBirthFace.size(), nvl::MAX_ID);
        for (VertexId vId = 0; vId < componentRefineBirthVertex.size(); ++vId) {
            if (componentRefineBirthVertex[vId] != nvl::MAX_ID) {
                assert(componentBirthVertex[componentRefineBirthVertex[vId]] != nvl::MAX_ID);
                currentBirthVertex[vId] = componentBirthVertex[componentRefineBirthVertex[vId]];
            }
        }
        for (FaceId fId = 0; fId < componentRefineBirthFace.size(); ++fId) {
            if (componentRefineBirthFace[fId] != nvl::MAX_ID) {
                assert(componentBirthFace[componentRefineBirthFace[fId]] != nvl::MAX_ID);
                currentBirthFace[fId] = componentBirthFace[componentRefineBirthFace[fId]];
            }
        }

        //Vertex map between the mesh and the result mesh
        std::vector<VertexId> currentVertexMap(mesh.nextVertexId(), nvl::MAX_ID);

        //Add new vertices to the result mesh and obtain the vertex map
        for (VertexId vId = 0; vId < componentRefinedMesh.nextVertexId(); ++vId) {
            if (componentRefinedMesh.isVertexDeleted(vId))
                continue;

            const Vertex& vertex = componentRefinedMesh.vertex(vId);

            if (currentBirthVertex[vId] == nvl::MAX_ID) {
                VertexId newVId = resultMesh.addVertex(vertex);

                currentVertexMap[vId] = newVId;

                assert(resultBirthVertex.size() == newVId);
                resultBirthVertex.push_back(nvl::MAX_ID);
            }
            else {
                assert(currentBirthVertex[vId] != nvl::MAX_ID);
                assert(resultVertexMap[currentBirthVertex[vId]] != nvl::MAX_ID);
                currentVertexMap[vId] = resultVertexMap[currentBirthVertex[vId]];
            }
        }

        //Add faces
        for (FaceId fId = 0; fId < componentRefinedMesh.nextFaceId(); ++fId) {
            if (componentRefinedMesh.isFaceDeleted(fId))
                continue;

            const Face& face = componentRefinedMesh.face(fId);

            std::vector<VertexId> newVertices(face.vertexNumber());

            for (VertexId j = 0; j < face.vertexNumber(); ++j) {
                const VertexId vId = currentVertexMap[face.vertexId(j)];
                assert(vId != nvl::MAX_ID);
                newVertices[j] = vId;
            }

            FaceId newFId = resultMesh.addFace(newVertices);

            assert(resultBirthFace.size() == newFId);
            resultBirthFace.push_back(currentBirthFace[fId]);
        }

        if (componentRefineCurveLines.size() >= 3) {
            std::vector<std::pair<VertexId, VertexId>> line;
            for (std::pair<VertexId, VertexId>& vertices : componentRefineCurveLines) {
                line.push_back(
                            std::make_pair(
                                currentVertexMap[vertices.first],
                            currentVertexMap[vertices.second]));
            }
            cutLines.push_back(line);
        }
    }
}

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
    }

    //Smooth function
    std::vector<std::vector<VertexId>> vvAdj = nvl::meshVertexVertexAdjacencies(mesh);
    nvl::laplacianSmoothing(cutFunction, vvAdj, 10, 0.8);

    for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
        if (mesh.isVertexDeleted(vId))
            continue;

        cutFunction[vId] = std::min(std::max((cutFunction[vId] - 0.5) * 2.0, -1.0), 1.0);
    }

    return cutFunction;
}

}

}
