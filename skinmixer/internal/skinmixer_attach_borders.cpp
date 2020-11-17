#include "skinmixer_attach_borders.h"

#include <nvl/vcglib/vcg_collapse_borders.h>
#include <nvl/vcglib/vcg_convert.h>
#include <nvl/vcglib/vcg_triangle_mesh.h>
#include <nvl/vcglib/vcg_polygon_mesh.h>

#include <nvl/models/mesh_adjacencies.h>
#include <nvl/models/mesh_borders.h>
#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_split.h>

#include <nvl/math/numeric_limits.h>
#include <nvl/math/closest_point.h>

#include <numeric>
#include <gurobi_c++.h>

#ifdef SAVE_MESHES_FOR_DEBUG
#include <nvl/models/mesh_io.h>
#endif

namespace skinmixer {
namespace internal {

template<class Mesh>
void attachMeshesByBorders(
        Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& meshNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices,
        std::unordered_set<typename Mesh::VertexId>& newSnappedVertices,
        std::unordered_set<typename Mesh::VertexId>& preSnappedVertices)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename nvl::Index Index;

    Mesh tmpMesh = mesh;

    //Get adjacencies of the mesh and connected components
    std::vector<std::vector<FaceId>> mFFAdj = nvl::meshFaceFaceAdjacencies(tmpMesh);
    std::vector<std::vector<FaceId>> mConnectedComponents = nvl::meshConnectedComponents(tmpMesh, mFFAdj);

    //Calculate chains for mesh, taking into account the connected component
    std::vector<std::vector<VertexId>> mChains;
    std::vector<Index> mChainsComponent;
    for (Index cId = 0; cId < mConnectedComponents.size(); cId++) {
        const std::vector<FaceId>& component = mConnectedComponents[cId];

        std::vector<std::vector<VertexId>> componentVertexChains = nvl::meshSubsetBorderVertexChains(
                    tmpMesh, std::unordered_set<FaceId>(component.begin(), component.end()), mFFAdj);

        for (std::vector<VertexId> componentVertexChain : componentVertexChains) {
            //We reverse to match the order (clockwise)
            std::reverse(componentVertexChain.begin(), componentVertexChain.end());

            mChains.push_back(componentVertexChain);
            mChainsComponent.push_back(cId);
        }
    }

    //Used components in the mesh
    std::unordered_set<Index> mUsedComponents;

    //Calculate chains for destination mesh
    std::vector<std::vector<VertexId>> dChains = nvl::meshBorderVertexChains(destMesh);

    //Define snappable flags
    std::vector<bool> mSnappable(mChains.size(), true);
    std::vector<bool> dSnappable(dChains.size(), true);

    for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
        const std::vector<VertexId>& mChain = mChains[mChainId];
        for (const VertexId& vId : mChain) {
            if (meshNonSnappableVertices.find(vId) != meshNonSnappableVertices.end()) {
                mSnappable[mChainId] = false;
                mUsedComponents.insert(mChainsComponent[mChainId]);
                break;
            }
        }
    }

    for (Index dChainId = 0; dChainId < dChains.size(); ++dChainId) {
        const std::vector<VertexId>& dChain = dChains[dChainId];
        for (const VertexId& vId : dChain) {
            if (destNonSnappableVertices.find(vId) != destNonSnappableVertices.end()) {
                dSnappable[dChainId] = false;
                break;
            }
        }
    }

    //Data to be used for the attaching
    std::vector<double> candidateScore;
    std::vector<Index> candidateMChains;
    std::vector<Index> candidateDChains;
    std::vector<Index> candidateMStartIndex;
    std::vector<Index> candidateDStartIndex;
    std::vector<std::vector<Index>> candidateDClosestVertex;
    std::vector<std::vector<double>> candidateDClosestT;

    for (Index dChainId = 0; dChainId < dChains.size(); ++dChainId) {
        const std::vector<VertexId>& dChain = dChains[dChainId];

        for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
            const std::vector<VertexId>& mChain = mChains[mChainId];

            VertexId startI = nvl::MAX_INDEX;
            VertexId startJ = nvl::MAX_INDEX;
            Scalar startDist = nvl::maxLimitValue<Scalar>();

            //Compute the start vertices (closest projected point)
            std::vector<VertexId> minDClosestVertex(dChain.size(), nvl::MAX_INDEX);
            std::vector<double> minDClosestT(dChain.size(), nvl::maxLimitValue<double>());
            for (Index j = 0; j < dChain.size(); ++j) {
                const Point& pPoint = destMesh.vertex(dChain[j]).point();

                Scalar minDist = nvl::maxLimitValue<Scalar>();

                for (Index i = 0; i < mChain.size(); ++i) {
                    Index nextI = (i + 1) % mChain.size();

                    const Point& nPoint = tmpMesh.vertex(mChain[i]).point();
                    const Point& nNextPoint = tmpMesh.vertex(mChain[nextI]).point();

                    double t;
                    Point projectionPoint = nvl::closestPointOnSegment(nPoint, nNextPoint, pPoint, t);
                    Scalar dist = (projectionPoint - pPoint).norm();

                    if (dist <= minDist) {
                        minDist = dist;
                        minDClosestVertex[j] = i;
                        minDClosestT[j] = t;

                        if (dist <= startDist) {
                            startDist = dist;
                            startI = i;
                            startJ = j;
                        }
                    }
                }
            }

            //Compute score by computing the total distortion
            Scalar score = 0.0;
            for (Index j = 0; j < dChain.size(); ++j) {
                Index currentI = minDClosestVertex[j];
                Index nextI = (currentI + 1) % mChain.size();

                const Point& pPoint = destMesh.vertex(dChain[j]).point();
                const Scalar& targetT = minDClosestT[j];

                Point currentIPoint = tmpMesh.vertex(mChain[currentI]).point();
                Point nextIPoint = tmpMesh.vertex(mChain[nextI]).point();

                Point nPoint = currentIPoint + (nextIPoint - currentIPoint) * targetT;

                score += (nPoint - pPoint).norm();
            }

            score /= dChain.size();

            candidateScore.push_back(score);
            candidateMChains.push_back(mChainId);
            candidateDChains.push_back(dChainId);
            candidateMStartIndex.push_back(startI);
            candidateDStartIndex.push_back(startJ);
            candidateDClosestVertex.push_back(minDClosestVertex);
            candidateDClosestT.push_back(minDClosestT);
        }
    }

    //Order by best distortion
    std::vector<Index> orderedCandidateId(candidateScore.size());
    std::iota(orderedCandidateId.begin(), orderedCandidateId.end(), 0);
    std::sort(orderedCandidateId.begin(), orderedCandidateId.end(), [&candidateScore] (const Index& a, const Index& b){
        return candidateScore[a] < candidateScore[b];
    });

    //Final parametrization for the candidates
    std::vector<std::vector<double>> dFinalParametrization(candidateScore.size(), std::vector<double>());
    std::vector<std::vector<double>> mFinalParametrization(candidateScore.size(), std::vector<double>());
    std::vector<bool> candidateComputed(candidateScore.size(), false);

    for (int it = 0; it < 2; ++it) {
        for (Index candidateId : orderedCandidateId) {
            Index dChainId = candidateDChains[candidateId];
            Index mChainId = candidateMChains[candidateId];

            if (!dSnappable[dChainId] || !mSnappable[mChainId]) {
                continue;
            }

            const Index& startJ = candidateDStartIndex[candidateId];
            const Index& startI = candidateMStartIndex[candidateId];
            const std::vector<Index>& dClosestIVertex = candidateDClosestVertex[candidateId];
            const std::vector<double>& dClosestIT = candidateDClosestT[candidateId];

            const std::vector<VertexId>& dChain = dChains[dChainId];
            const std::vector<VertexId>& mChain = mChains[mChainId];

            //Smooth vertices in the loops
            std::vector<Point> mSmoothPoint(mChain.size());
            std::vector<Point> dSmoothPoint(dChain.size());
            for (Index i = 0; i < mChain.size(); ++i) {
                mSmoothPoint[i] = tmpMesh.vertex(mChain[i]).point();
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
            std::vector<double>& mParametrization = mFinalParametrization[candidateId];
            std::vector<double>& dParametrization = dFinalParametrization[candidateId];
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

            mUsedComponents.insert(mChainsComponent[mChainId]);
            dSnappable[dChainId] = false;
            mSnappable[mChainId] = false;
            candidateComputed[candidateId] = true;
        }

        //We set as non snappable all the chains in components not used
        for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
            if (mSnappable[mChainId] && mUsedComponents.find(mChainsComponent[mChainId]) == mUsedComponents.end()) {
                mSnappable[mChainId] = false;
            }
        }

        //We set as snappable the border chains on preserved mesh
        for (Index dChainId = 0; dChainId < dChains.size(); ++dChainId) {
            const std::vector<VertexId>& dChain = dChains[dChainId];
            for (const VertexId& vId : dChain) {
                if (destNonSnappableVertices.find(vId) != destNonSnappableVertices.end()) {
                    dSnappable[dChainId] = true;
                    break;
                }
            }
        }
    }

    //Clean mesh from components that are non-splitted in borders
    std::vector<FaceId> facesToKeep;
    for (const Index& cId : mUsedComponents) {
        const std::vector<FaceId>& connectedComponent = mConnectedComponents[cId];
        facesToKeep.insert(facesToKeep.end(), connectedComponent.begin(), connectedComponent.end());
    }

    //Save in the final mesh data-structure
    mesh.clear();
    std::vector<VertexId> cleaningBirthVertex;
    std::vector<FaceId> cleaningBirthFace;
    nvl::meshTransferFaces(tmpMesh, facesToKeep, mesh, cleaningBirthVertex, cleaningBirthFace);

    std::vector<VertexId> cleaningVertexMap = nvl::inverseMap(cleaningBirthVertex, tmpMesh.nextVertexId());

    for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
        std::vector<VertexId>& mChain = mChains[mChainId];
        for (Index i = 0; i < mChain.size(); ++i) {
            mChain[i] = cleaningVertexMap[mChain[i]];
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/attaching_1_cleaned.obj", mesh);
#endif

    std::vector<std::vector<FaceId>> meshVFAdj = nvl::meshVertexFaceAdjacencies(mesh);

    std::vector<VertexId> fixedBorderVertices;
    std::unordered_set<Index> computedMChain;

    for (Index candidateId : orderedCandidateId) {
        //Parametrization computed
        if (candidateComputed[candidateId]) {
            const std::vector<double>& mParametrization = mFinalParametrization[candidateId];
            const std::vector<double>& dParametrization = dFinalParametrization[candidateId];

            const Index& mChainId = candidateMChains[candidateId];
            const Index& dChainId = candidateDChains[candidateId];
            const Index& startI = candidateMStartIndex[candidateId];
            const Index& startJ = candidateDStartIndex[candidateId];

            const std::vector<VertexId>& mChain = mChains[mChainId];
            const std::vector<VertexId>& dChain = dChains[dChainId];

            Index i = 0;

            computedMChain.insert(mChainId);

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

                VertexId newNVertexId = nvl::meshSplitEdge(mesh, currentMVertexId, mChain[nextI], jPoint, meshVFAdj);

                newSnappedVertices.insert(newNVertexId);
                preSnappedVertices.insert(destVertexId);

                fixedBorderVertices.push_back(newNVertexId);

                currentMVertexId = newNVertexId;
            }
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/attaching_2_splitted.obj", mesh);
#endif

    for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
        if (computedMChain.find(mChainId) != computedMChain.end())
            continue;

        const std::vector<VertexId>& mChain = mChains[mChainId];
        for (Index i = 0; i < mChain.size(); ++i) {
            if (mChain[i] != nvl::MAX_INDEX) {
                fixedBorderVertices.push_back(mChain[i]);
            }
        }
    }


    std::vector<VertexId> nonCollapsed = nvl::collapseBorders(mesh, fixedBorderVertices);
    std::cout << nonCollapsed.size() << " vertices non collapsed." << std::endl;

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/attaching_3_collapsed.obj", mesh);
#endif

    if (!nonCollapsed.empty()) {
        //TODO!!
    }
}

template<class Mesh>
std::vector<typename Mesh::FaceId> getPreNotUsedFacesAfterAttaching(
        const Mesh& preMesh,
        const std::unordered_set<typename Mesh::VertexId>& preNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& preSnappedVertices)
{
    typedef typename nvl::Index Index;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;

    //Get adjacencies of the mesh and connected components
    std::vector<std::vector<FaceId>> preFFAdj = nvl::meshFaceFaceAdjacencies(preMesh);
    std::vector<std::vector<FaceId>> preConnectedComponents = nvl::meshConnectedComponents(preMesh, preFFAdj);

    //Used components in the mesh
    std::unordered_set<Index> preUsedComponents;

    //Calculate chains for mesh, taking into account the connected component
    std::vector<std::vector<VertexId>> preChains;
    std::vector<Index> preChainsComponent;
    for (Index cId = 0; cId < preConnectedComponents.size(); cId++) {
        const std::vector<FaceId>& component = preConnectedComponents[cId];

        std::vector<std::vector<VertexId>> componentVertexChains = nvl::meshSubsetBorderVertexChains(
                    preMesh, std::unordered_set<FaceId>(component.begin(), component.end()), preFFAdj);

        if (componentVertexChains.empty()) {
            preUsedComponents.insert(cId);
        }
        for (std::vector<VertexId> componentVertexChain : componentVertexChains) {
            preChains.push_back(componentVertexChain);
            preChainsComponent.push_back(cId);
        }
    }

    //Add components snapped or that have already border vertices
    for (Index preChainId = 0; preChainId < preChains.size(); ++preChainId) {
        const std::vector<VertexId>& pChain = preChains[preChainId];
        for (const VertexId& vId : pChain) {
            if (preNonSnappableVertices.find(vId) != preNonSnappableVertices.end() || preSnappedVertices.find(vId) != preSnappedVertices.end()) {
                preUsedComponents.insert(preChainsComponent[preChainId]);
            }
        }
    }

    //Get faces that are not used
    std::vector<typename Mesh::FaceId> preErasedFaces;
    for (Index cId = 0; cId < preConnectedComponents.size(); cId++) {
        if (preUsedComponents.find(cId) == preUsedComponents.end()) {
            for (const FaceId& fId : preConnectedComponents[cId]) {
                preErasedFaces.push_back(fId);
            }
        }
    }

    return preErasedFaces;
}

}
}
