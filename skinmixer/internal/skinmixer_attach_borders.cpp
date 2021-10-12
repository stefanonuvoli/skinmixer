#include "skinmixer_attach_borders.h"

#include <nvl/vcglib/vcg_collapse_borders.h>
#include <nvl/vcglib/vcg_convert.h>
#include <nvl/vcglib/vcg_triangle_mesh.h>
#include <nvl/vcglib/vcg_polygon_mesh.h>

#include <nvl/models/mesh_adjacencies.h>
#include <nvl/models/mesh_borders.h>
#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_split.h>
#include <nvl/models/mesh_graph.h>

#include <nvl/math/numeric_limits.h>
#include <nvl/math/closest_point.h>

#include <nvl/structures/graph.h>
#include <nvl/structures/dijkstra.h>

#include <numeric>
#include <gurobi_c++.h>

#ifdef SAVE_MESHES_FOR_DEBUG
#include <nvl/models/mesh_io.h>
#endif

namespace skinmixer {
namespace internal {

template<class Mesh>
Mesh attachMeshesByBorders(
        const Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices,
        const std::unordered_set<typename Mesh::FaceId>& newSurfaceFaces,
        std::unordered_set<typename Mesh::VertexId>& newSnappedVertices,
        std::unordered_set<typename Mesh::VertexId>& preSnappedVertices)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename nvl::Index Index;

    Mesh newMesh; //Result

    //Mesh graph, connected components and face-face adjacencies
    std::vector<std::vector<FaceId>> mVVAdj = nvl::meshVertexVertexAdjacencies(mesh);
    nvl::Graph<VertexId> meshGraph = nvl::meshGraph(mesh, mVVAdj);
    std::vector<std::vector<FaceId>> mFFAdj = nvl::meshFaceFaceAdjacencies(mesh);
    std::vector<std::vector<FaceId>> mConnectedComponents = nvl::meshConnectedComponents(mesh, mFFAdj);

    //Calculate chains for destination mesh
    std::vector<std::vector<VertexId>> dChains;

    //Chains selected for the target mesh
    std::vector<std::vector<VertexId>> mChains;
    std::unordered_set<Index> mUsedComponents;

    //Find snappable chains
    std::vector<std::vector<VertexId>> dBorderVertexChains = nvl::meshBorderVertexChains(destMesh);
    for (Index chainId = 0; chainId < dBorderVertexChains.size(); ++chainId) {
        const std::vector<VertexId>& vertexChain = dBorderVertexChains[chainId];

        bool snappable = true;;

        for (const VertexId& vId : vertexChain) {
            if (destNonSnappableVertices.find(vId) != destNonSnappableVertices.end()) {
                snappable = false;
            }
        }

        if (snappable) {
            dChains.push_back(vertexChain);
        }
    }

    //Find faces to be deleted
    std::unordered_set<FaceId> facesToDelete;

    //Find best chains that matches the destination mesh borders
    mChains.resize(dChains.size());
    std::unordered_set<VertexId> verticesInChains;
    for (Index chainId = 0; chainId < dChains.size(); ++chainId) {
        const std::vector<VertexId>& dChain = dChains[chainId];
        std::vector<VertexId>& mChain = mChains[chainId];

        double mChainScore = nvl::maxLimitValue<double>();
        Index mComponentId = nvl::MAX_INDEX;

        //For each component, find the candidate chain
        for (Index cId = 0; cId < mConnectedComponents.size(); cId++) {
            nvl::Graph<VertexId> tmpMeshGraph = meshGraph;

            const std::vector<FaceId>& component = mConnectedComponents[cId];
            std::vector<VertexId> componentVertices;

            for (const FaceId& fId : component) {
                if (facesToDelete.find(fId) == facesToDelete.end()) {
                    for (const VertexId& vId : mesh.face(fId).vertexIds()) {
                        componentVertices.push_back(vId);
                    }
                }
            }

            //Find candidate chain
            std::vector<VertexId> candidateChain;

            //Vertices used in the candidate chain
            std::unordered_set<VertexId> chainVerticesSet;

            bool validPath = true;
            double chainScore = 0.0;
            for (Index j = 0; j < dChain.size() && validPath; ++j) {
                const Point& dPoint = destMesh.vertex(dChain[j]).point();

                double bestDistance = nvl::maxLimitValue<double>();
                VertexId bestVId = nvl::MAX_INDEX;

                for (const VertexId& vId : componentVertices) {
                    if (chainVerticesSet.find(vId) != chainVerticesSet.end() ||
                        verticesInChains.find(vId) != verticesInChains.end())
                    {
                        continue;
                    }

                    const Point& mPoint = mesh.vertex(vId).point();

                    double distance = (mPoint - dPoint).norm();

                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestVId = vId;
                    }
                }

                if (bestVId < nvl::MAX_INDEX) {
                    chainScore += bestDistance;

                    if (j > 0) {
                        nvl::GraphPath<VertexId> sp = nvl::dijkstra(tmpMeshGraph, candidateChain[candidateChain.size()-1], bestVId);
                        if (!sp.path.empty()) {
                            std::vector<VertexId> path(sp.path.begin(), sp.path.end());

                            tmpMeshGraph.deleteNode(path[0]);
                            for (Index k = 1; k < path.size()-1; ++k) {
                                tmpMeshGraph.deleteNode(path[k]);
                                chainVerticesSet.insert(path[k]);
                                candidateChain.push_back(path[k]);
                            }
                        }
                        else {
                            validPath = false;
                        }
                    }

                    if (validPath) {
                        chainVerticesSet.insert(bestVId);
                        candidateChain.push_back(bestVId);
                    }
                }
                else {
                    validPath = false;
                }
            }

            if (validPath) {
                //Readd first node and its incoming edges
                tmpMeshGraph.addNode(candidateChain[0]);
                for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
                    if (mesh.isVertexDeleted(vId))
                        continue;

                    for (const VertexId& adjVId : mVVAdj[vId]) {
                        if (adjVId == candidateChain[0]) {
                            const Point& p1 = mesh.vertex(vId).point();
                            const Point& p2 = mesh.vertex(adjVId).point();

                            const double distance = (p1 - p2).norm();
                            tmpMeshGraph.addEdge(vId, adjVId, distance);
                        }
                    }
                }

                //Close the loop
                nvl::GraphPath<VertexId> sp = nvl::dijkstra(tmpMeshGraph, candidateChain[candidateChain.size()-1], candidateChain[0]);
                if (!sp.path.empty()) {
                    std::vector<VertexId> path(sp.path.begin(), sp.path.end());

                    tmpMeshGraph.deleteNode(path[0]);
                    for (Index k = 1; k < path.size()-1; ++k) {
                        tmpMeshGraph.deleteNode(path[k]);
                        chainVerticesSet.insert(path[k]);
                        candidateChain.push_back(path[k]);
                    }
                    tmpMeshGraph.deleteNode(path[path.size()-1]);

                    chainScore /= dChain.size();
                }
                else {
                    validPath = false;
                }
            }

            if (validPath && chainScore < mChainScore) {
                mChain = candidateChain;
                mChainScore = chainScore;
                mComponentId = cId;
            }
        }

        assert(mComponentId < nvl::MAX_INDEX);
        mUsedComponents.insert(mComponentId);

        //Delete nodes from graph
        for (const VertexId& vId : mChain) {
            meshGraph.deleteNode(vId);
        }

        verticesInChains.insert(mChain.begin(), mChain.end());

        //Get chain edges
        std::set<std::pair<VertexId, VertexId>> chainEdges;
        for (Index i = 0; i < mChain.size(); ++i) {
            const Index nextI = (i + 1) % mChain.size();
            VertexId v1 = mChain[i];
            VertexId v2 = mChain[nextI];

            assert(v1 != v2);
            if (v1 > v2) {
                std::swap(v1, v2);
            }

            chainEdges.insert(std::make_pair(v1, v2));
        }

        //Find components splitted by the chain
        std::vector<std::vector<typename Mesh::FaceId>> chainComponents;
        std::stack<FaceId> stack;
        std::vector<bool> visited(mesh.nextFaceId(), false);
        for (const FaceId& fId : mConnectedComponents[mComponentId]) {
            if (visited[fId] || facesToDelete.find(fId) != facesToDelete.end()) {
                continue;
            }

            stack.push(fId);

            std::vector<FaceId> currentComponent;

            while (!stack.empty()) {
                FaceId currentFId = stack.top();
                stack.pop();

                if (!visited[currentFId]) {
                    assert(facesToDelete.find(currentFId) == facesToDelete.end());
                    currentComponent.push_back(currentFId);

                    const Face& face = mesh.face(currentFId);

                    for (Index k = 0; k < face.vertexNumber(); ++k) {
                        VertexId v1 = face.vertexId(k);
                        VertexId v2 = face.nextVertexId(k);

                        assert(v1 != v2);
                        if (v1 > v2) {
                            std::swap(v1, v2);
                        }

                        if (chainEdges.find(std::make_pair(v1, v2)) == chainEdges.end()) {
                            const FaceId& adjId = mFFAdj[currentFId][k];

                            if (adjId != nvl::MAX_INDEX && !visited[adjId] && facesToDelete.find(adjId) == facesToDelete.end()) {
                                stack.push(adjId);
                            }
                        }
                    }

                    visited[currentFId] = true;
                }
            }

            chainComponents.push_back(currentComponent);
        }

        assert(chainComponents.size() == 2);

        //Delete component with less new surface faces
        Index bestChainComponentToDelete = nvl::MAX_INDEX;
        double bestChainComponentToDeleteScore = nvl::maxLimitValue<double>();
        for (Index i = 0; i < chainComponents.size(); ++i) {
            std::vector<FaceId>& faces = chainComponents[i];

            double score = 0.0;
            for (const FaceId& fId : faces) {
                if (newSurfaceFaces.find(fId) != newSurfaceFaces.end()) {
                    score += 1.0;
                }
                const Face& face = mesh.face(fId);
            }

            if (score < bestChainComponentToDeleteScore) {
                bestChainComponentToDeleteScore = score;
                bestChainComponentToDelete = i;
            }
        }

        assert(bestChainComponentToDelete < nvl::MAX_INDEX);

        for (const FaceId& fId : chainComponents[bestChainComponentToDelete]) {
            facesToDelete.insert(fId);
        }


#ifdef SAVE_MESHES_FOR_DEBUG
        std::vector<FaceId> facesToKeep;
        for (const Index& cId : mUsedComponents) {
            const std::vector<FaceId>& connectedComponent = mConnectedComponents[cId];
            for (const FaceId& fId : connectedComponent) {
                if (facesToDelete.find(fId) == facesToDelete.end()) {
                    facesToKeep.push_back(fId);
                }
            }
        }

        Mesh tmpMesh;
        nvl::meshTransferFaces(mesh, facesToKeep, tmpMesh);

        nvl::meshSaveToFile("results/attaching_1_component_" + std::to_string(chainId) + ".obj", tmpMesh);
#endif
    }





    //Data to be used for the attaching
    std::vector<Index> chainStartJ(dChains.size());
    std::vector<Index> chainStartI(dChains.size());
    std::vector<std::vector<Index>> chainDClosestV(dChains.size());
    std::vector<std::vector<double>> chainDClosestT(dChains.size());

    //Compute data used for attaching
    for (Index chainId = 0; chainId < dChains.size(); ++chainId) {        
        const std::vector<VertexId>& dChain = dChains[chainId];
        const std::vector<VertexId>& mChain = mChains[chainId];

        VertexId bestStartI = nvl::MAX_INDEX;
        VertexId bestStartJ = nvl::MAX_INDEX;
        Scalar bestStartDist = nvl::maxLimitValue<Scalar>();

        //Compute the start vertices (closest projected point)
        std::vector<VertexId> minDClosestVertex(dChain.size(), nvl::MAX_INDEX);
        std::vector<double> minDClosestT(dChain.size(), nvl::maxLimitValue<double>());
        for (Index j = 0; j < dChain.size(); ++j) {
            const Point& dPoint = destMesh.vertex(dChain[j]).point();

            Scalar minDist = nvl::maxLimitValue<Scalar>();

            for (Index i = 0; i < mChain.size(); ++i) {
                Index nextI = (i + 1) % mChain.size();

                const Point& mPoint = mesh.vertex(mChain[i]).point();
                const Point& mNextPoint = mesh.vertex(mChain[nextI]).point();

                double t;
                Point projectionPoint = nvl::closestPointOnSegment(mPoint, mNextPoint, dPoint, t);
                Scalar dist = (projectionPoint - dPoint).norm();

                if (dist <= minDist) {
                    minDist = dist;
                    minDClosestVertex[j] = i;
                    minDClosestT[j] = t;

                    if (dist <= bestStartDist) {
                        bestStartDist = dist;
                        bestStartI = i;
                        bestStartJ = j;
                    }
                }
            }
        }

        chainStartI[chainId] = bestStartI;
        chainStartJ[chainId] = bestStartJ;
        chainDClosestV[chainId] = minDClosestVertex;
        chainDClosestT[chainId] = minDClosestT;
    }


    //Final parametrization for the candidates
    std::vector<std::vector<double>> dFinalParametrization(dChains.size(), std::vector<double>());
    std::vector<std::vector<double>> mFinalParametrization(dChains.size(), std::vector<double>());


    //Compute final parameterization
    for (Index chainId = 0; chainId < dChains.size(); ++chainId) {
        const std::vector<VertexId>& dChain = dChains[chainId];
        const std::vector<VertexId>& mChain = mChains[chainId];

        const Index& startI = chainStartI[chainId];
        const Index& startJ = chainStartJ[chainId];
        const std::vector<Index>& dClosestIVertex = chainDClosestV[chainId];
        const std::vector<double>& dClosestIT = chainDClosestT[chainId];

        //Smooth vertices in the loops
        std::vector<Point> mSmoothPoint(mChain.size());
        std::vector<Point> dSmoothPoint(dChain.size());
        for (Index i = 0; i < mChain.size(); ++i) {
            mSmoothPoint[i] = mesh.vertex(mChain[i]).point();
        }
        for (Index j = 0; j < dChain.size(); ++j) {
            dSmoothPoint[j] = destMesh.vertex(dChain[j]).point();
        }
        for (Index t = 0; t < 5; ++t) {
            std::vector<Point> mChainPointsCopy = mSmoothPoint;
            for (Index i = 0; i < mSmoothPoint.size(); ++i) {
                Index nextI = (i + 1) % mSmoothPoint.size();
                Index prevI = i > 0 ? i - 1 : mSmoothPoint.size() - 1;
                mSmoothPoint[i] =
                        0.5 * mChainPointsCopy[i] +
                        0.5 * (mChainPointsCopy[prevI] + mChainPointsCopy[nextI]);
            }

            std::vector<Point> dChainPointsCopy = dSmoothPoint;
            for (Index j = 0; j < dSmoothPoint.size(); ++j) {
                Index nextJ = (j + 1) % dSmoothPoint.size();
                Index prevJ = j > 0 ? j - 1 : dSmoothPoint.size() - 1;
                dSmoothPoint[j] =
                        0.5 * dChainPointsCopy[j] +
                        0.5 * (dChainPointsCopy[prevJ] + dChainPointsCopy[nextJ]);
            }
        }

        //Parametrization functions on length
        std::vector<double>& mParametrization = mFinalParametrization[chainId];
        std::vector<double>& dParametrization = dFinalParametrization[chainId];
        mParametrization.resize(mChain.size());
        dParametrization.resize(dChain.size());

        //Total distances
        Scalar dChainTotalDist = 0.0;
        Scalar mChainSmoothedTotalDist = 0.0;
        for (Index j = 0; j < dChain.size() - 1; ++j) {
            Index nextJ = (j + 1) % dChain.size();

            nvl::Vector3d vec = dSmoothPoint[nextJ] - dSmoothPoint[j];
            dChainTotalDist += vec.norm();
        }
        for (Index i = 0; i < mChain.size() - 1; ++i) {
            Index nextI = (i + 1) % mChain.size();

            nvl::Vector3d vec = mSmoothPoint[nextI] - mSmoothPoint[i];
            mChainSmoothedTotalDist += vec.norm();
        }

        //Compute parametrization for destination mesh
        Scalar dChainCurrentDist = 0.0;
        dParametrization[startJ] = 0.0;
        for (Index j = 0; j < dChain.size(); ++j) {
            Index currentJ = (startJ + j) % dChain.size();
            Index nextJ = (currentJ + 1) % dChain.size();

            nvl::Vector3d vec = dSmoothPoint[nextJ] - dSmoothPoint[currentJ];

            double value = dChainCurrentDist / dChainTotalDist;
            dParametrization[currentJ] = std::max(std::min(value, 1.0), 0.0);

            dChainCurrentDist += vec.norm();
        }

        //Compute offset of the parametrization
        const double offsetT = (
                    dClosestIT[startJ] * (
                        mSmoothPoint[(startI + 1) % mChain.size()] - mSmoothPoint[startI]
                    ).norm()
                ) / mChainSmoothedTotalDist;

        //Compute parametrization for destination mesh
        Scalar mChainCurrentDist = 0.0;
        for (Index i = 0; i < mChain.size(); ++i) {
            Index currentI = (startI + i) % mChain.size();
            Index nextI = (currentI + 1) % mChain.size();

            nvl::Vector3d vec = mSmoothPoint[nextI] - mSmoothPoint[currentI];
            mParametrization[currentI] = (mChainCurrentDist / mChainSmoothedTotalDist) - offsetT;

            mChainCurrentDist += vec.norm();
        }

        //Optimization parameters
        const double alpha = 0.9;
        const double distanceCost = alpha;
        const double lengthCost = 1 - alpha;

        //Final parametrization
        try {
            GRBEnv env = GRBEnv();

            GRBModel model = GRBModel(env);

#ifdef GUROBI_NON_VERBOSE
            model.set(GRB_IntParam_OutputFlag, 0);
#endif
            //Variables
            std::vector<GRBVar> vars(mChain.size());
            for (size_t i = 0; i < mChain.size(); i++) {
                vars[i] = model.addVar(-1.0, 2.0, 0.0, GRB_CONTINUOUS, "t" + std::to_string(i));
            }

            //Length parametrization cost
            GRBQuadExpr lengthObj = 0;
            for (size_t i = 0; i < mChain.size(); i++) {
                Index currentI = (startI + i) % mChain.size();

                double value = mParametrization[currentI];

                lengthObj += (vars[currentI] - value) * (vars[currentI] - value);
            }

            lengthObj /= mChain.size();

            //Distance cost
            GRBQuadExpr distanceObj = 0;
            int numDistanceTerms = 0;
            double lastParam1 = nvl::minLimitValue<double>();
            double lastParam2 = nvl::minLimitValue<double>();
            for (size_t j = 0; j < dChain.size(); j++) {
                Index currentJ = (j + startJ) % dChain.size();

                Index currentI = dClosestIVertex[currentJ];
                Index nextI = (currentI + 1) % mChain.size();

                //Avoid if parametrization has decreased
                if (mParametrization[currentI] < lastParam1 || mParametrization[nextI] < lastParam2) {
                    continue;
                }
                lastParam1 = mParametrization[currentI];
                lastParam2 = mParametrization[nextI];

                const double& currentT = dClosestIT[currentJ];

                Scalar parametrizedDistance = (mSmoothPoint[nextI] - mSmoothPoint[currentI]).norm() / mChainSmoothedTotalDist;

                double value1 = dParametrization[currentJ] - (currentT * parametrizedDistance);
                double value2 = dParametrization[currentJ] + ((1.0 - currentT) * parametrizedDistance);

                distanceObj +=
                        0.5 * ((vars[currentI] - value1) * (vars[currentI] - value1)) +
                        0.5 * ((vars[nextI] - value2) * (vars[nextI] - value2));

                numDistanceTerms++;
            }

            distanceObj /= numDistanceTerms;

            //Objective function
            GRBQuadExpr obj = distanceCost * distanceObj + lengthCost * lengthObj;

            //Add constraints
            model.addConstr(vars[startI] == mParametrization[startI]);
            for (size_t i = 0; i < mChain.size() - 1; i++) {
                Index currentI = (startI + i) % mChain.size();
                Index nextI = (currentI + 1) % mChain.size();
                model.addConstr(vars[nextI] >= vars[currentI] + 0.0001);
            }

            //Set objective function
            model.setObjective(obj, GRB_MINIMIZE);

            //Optimize model
            model.optimize();

            //Retrieve result
            for (size_t i = 0; i < mChain.size(); i++) {
                mParametrization[i] = vars[i].get(GRB_DoubleAttr_X);
            }

#ifndef GUROBI_NON_VERBOSE
            std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
#endif
        }
        catch (GRBException e) {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        }
    }

    //Clean mesh from components that are non-splitted in borders
    std::vector<FaceId> facesToKeep;
    for (const Index& cId : mUsedComponents) {
        const std::vector<FaceId>& connectedComponent = mConnectedComponents[cId];
        for (const FaceId& fId : connectedComponent) {
            if (facesToDelete.find(fId) == facesToDelete.end()) {
                facesToKeep.push_back(fId);
            }
        }
    }

    //Save in the final mesh data-structure
    std::vector<VertexId> cleaningBirthVertex;
    std::vector<FaceId> cleaningBirthFace;
    nvl::meshTransferFaces(mesh, facesToKeep, newMesh, cleaningBirthVertex, cleaningBirthFace);

    std::vector<VertexId> cleaningVertexMap = nvl::inverseMap(cleaningBirthVertex, mesh.nextVertexId());

    for (Index chainId = 0; chainId < mChains.size(); ++chainId) {
        std::vector<VertexId>& mChain = mChains[chainId];
        for (Index i = 0; i < mChain.size(); ++i) {
            mChain[i] = cleaningVertexMap[mChain[i]];
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/attaching_2_cleaned.obj", newMesh);
#endif

    std::vector<std::vector<FaceId>> meshVFAdj = nvl::meshVertexFaceAdjacencies(newMesh);

    std::vector<VertexId> fixedBorderVertices;

    for (Index chainId = 0; chainId < dChains.size(); ++chainId) {
        const std::vector<VertexId>& dChain = dChains[chainId];
        const std::vector<VertexId>& mChain = mChains[chainId];
        const std::vector<double>& mParametrization = mFinalParametrization[chainId];
        const std::vector<double>& dParametrization = dFinalParametrization[chainId];

        const Index& startI = chainStartI[chainId];
        const Index& startJ = chainStartJ[chainId];
        Index i = 0;

        VertexId currentMVertexId = mChain[startI];
        for (Index j = 0; j < dChain.size(); ++j) {
            Index currentJ = (startJ + j) % dChain.size();
            Index currentI = (startI + i) % mChain.size();
            Index nextI = (currentI + 1) % mChain.size();

            const VertexId& destVertexId = dChain[currentJ];
            const Point& jPoint = destMesh.vertex(destVertexId).point();

            const Scalar& pT = dParametrization[currentJ];

            while (i < mChain.size() - 1 && mParametrization[nextI] < pT) {
                i++;

                currentI = (startI + i) % mChain.size();
                nextI = (currentI + 1) % mChain.size();

                currentMVertexId = mChain[currentI];
            }

            VertexId newNVertexId = nvl::meshSplitEdge(newMesh, currentMVertexId, mChain[nextI], jPoint, meshVFAdj);

            newSnappedVertices.insert(newNVertexId);
            preSnappedVertices.insert(destVertexId);

            fixedBorderVertices.push_back(newNVertexId);

            currentMVertexId = newNVertexId;
        }
    }


#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/attaching_3_splitted.obj", newMesh);
#endif


    std::vector<VertexId> nonCollapsed = nvl::collapseBorders(newMesh, fixedBorderVertices);
    std::cout << nonCollapsed.size() << " vertices non collapsed." << std::endl;

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/attaching_4_collapsed.obj", newMesh);
#endif

    if (!nonCollapsed.empty()) {
        //TODO!!
    }

    return newMesh;
}

}
}
