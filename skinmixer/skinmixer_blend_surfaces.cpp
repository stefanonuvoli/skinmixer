#include "skinmixer_blend_surfaces.h"

#include "internal/skinmixer_attach_borders.h"
#include "internal/skinmixer_openvdb_blending.h"

#include <nvl/math/numeric_limits.h>
#include <nvl/math/transformations.h>

#include <nvl/models/mesh_geometric_information.h>
#include <nvl/models/mesh_morphological_operations.h>
#include <nvl/models/mesh_transformations.h>
#include <nvl/models/mesh_adjacencies.h>
#include <nvl/models/mesh_borders.h>
#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_smoothing.h>

#include <nvl/vcglib/vcg_convert.h>
#include <nvl/vcglib/vcg_triangle_mesh.h>
#include <nvl/vcglib/vcg_polygon_mesh.h>

#ifdef SAVE_MESHES_FOR_DEBUG
#include <nvl/models/mesh_io.h>
#endif

#include <quadretopology/quadretopology.h>

#include <vector>

#define FACE_KEEP_THRESHOLD 0.999
#define MAX_DISTANCE_BLENDED_MESH 2.0
#define VOXEL_SIZE_FACTOR 0.7
#define ATTACH_DISTANCE 0.7
#define MAX_VOXEL_DISTANCE 100.0
#define SMOOTHING_THRESHOLD 0.9

namespace skinmixer {

namespace internal {

template<class Mesh>
Mesh quadrangulateMesh(
        const Mesh& newMesh,
        const Mesh& preMesh,
        const Mesh& blendedMesh,
        Mesh& quadrangulation,
        const std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& preBirthVertex,
        const std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& preBirthFace,
        std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& birthVertex,
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& birthFace);

}

template<class Model>
void blendSurfaces(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        typename SkinMixerData<Model>::Entry& entry)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename nvl::Index Index;
    typedef typename SkinMixerData<Model>::BirthInfo::VertexInfo VertexInfo;
    typedef typename SkinMixerData<Model>::SelectInfo SelectInfo;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename SkinMixerData<Model>::Entry Entry;

    //New model
    Model* model = entry.model;
    Mesh& resultMesh = model->mesh;

    //Max distance of the distance field
    double maxDistance = MAX_VOXEL_DISTANCE;

    //Calculate voxel size
    double voxelSize = nvl::maxLimitValue<Scalar>();

    //Get actions
    std::vector<Index> actions;
    for (const Index eId : cluster) {
        const Entry& entry = data.entry(eId);

        actions.insert(actions.end(), entry.relatedActions.begin(), entry.relatedActions.end());

        const Mesh& mesh = entry.model->mesh;

        Scalar avgLength = nvl::meshAverageEdgeLength(mesh);
        voxelSize = std::min(voxelSize, avgLength * VOXEL_SIZE_FACTOR);
    }

    //Remove duplicate actions
    std::sort(actions.begin(), actions.end());
    actions.erase(std::unique(actions.begin(), actions.end()), actions.end());

    //Get models of each action
    std::vector<std::vector<const Model*>> models(actions.size());
    std::vector<std::vector<const std::vector<double>*>> vertexSelectValue(actions.size());

    //Get models and select values of the actions
    for (Index aId = 0; aId < actions.size(); ++aId) {
        const Index& actionId = actions[aId];

        const Action& action = data.action(actionId);

        if (action.entry1 != nvl::MAX_INDEX) {
            models[aId].push_back(data.entry(action.entry1).model);
            vertexSelectValue[aId].push_back(&action.select1.vertex);
        }

        if (action.entry2 != nvl::MAX_INDEX) {
            models[aId].push_back(data.entry(action.entry2).model);
            vertexSelectValue[aId].push_back(&action.select2.vertex);
        }
    }

    //Scale transform
    double scaleFactor = 1.0 / voxelSize;
    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);


    //Grid data
    std::vector<std::vector<internal::FloatGridPtr>> closedGrids(actions.size());
    std::vector<std::vector<internal::IntGridPtr>> polygonGrids(actions.size());
    std::vector<std::vector<std::vector<VertexId>>> gridBirthVertex(actions.size());
    std::vector<std::vector<std::vector<FaceId>>> gridBirthFace(actions.size());

    //Fill faces to keep with all faces for each model in the cluster
    std::unordered_map<const Model*, std::unordered_set<FaceId>> preFacesToKeepMap;
    for (const Index eId : cluster) {
        const Entry& entry = data.entry(eId);

        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        std::unordered_set<FaceId> facesToKeep;
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            facesToKeep.insert(fId);
        }

        preFacesToKeepMap.insert(std::make_pair(model, facesToKeep));
    }

    //Blended and new mesh
    Mesh blendedMesh;
    Mesh newMesh;

    //For each action get the blended and the new mesh, and determine faces to be preserved
    for (Index aId = 0; aId < actions.size(); ++aId) {
        const Index& actionId = actions[aId];
        const Action& action = data.action(actionId);
        const std::vector<const Model*>& actionModels = models[aId];
        const std::vector<const std::vector<double>*>& actionVertexSelectValues = vertexSelectValue[aId];

        //Fill faces to keep with all faces
        std::vector<std::unordered_set<FaceId>> actionFacesToKeep(actionModels.size());
        for (Index mId = 0; mId < actionModels.size(); ++mId) {
            const Mesh& mesh = actionModels[mId]->mesh;

            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                actionFacesToKeep[mId].insert(fId);
            }
        }

        //Erase faces to keep for the original meshes
        for (Index mId = 0; mId < actionModels.size(); ++mId) {
            const Mesh& mesh = actionModels[mId]->mesh;
            //Erase face under the threshold
            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                double selectValue = internal::averageFaceSelectValue(mesh, fId, *actionVertexSelectValues[mId]);

                if (selectValue < FACE_KEEP_THRESHOLD) {
                    actionFacesToKeep[mId].erase(fId);
                }
            }

            nvl::meshOpenFaceSelection(mesh, actionFacesToKeep[mId]);
            nvl::meshCloseFaceSelection(mesh, actionFacesToKeep[mId]);

            //Reinsert deleted face due to open/close
            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                const Face& face = mesh.face(fId);

                bool toKeep = true;
                for (Index j = 0; j < face.vertexNumber(); j++) {
                    double selectValue = actionVertexSelectValues[mId]->at(face.vertexId(j));
                    if (!nvl::epsEqual(selectValue, 1.0)) {
                        toKeep = false;
                    }
                }
                if (toKeep) {
                    actionFacesToKeep[mId].insert(fId);
                }
            }
        }

        std::vector<internal::FloatGridPtr>& actionClosedGrids = closedGrids[aId];
        std::vector<internal::IntGridPtr>& actionPolygonGrids = polygonGrids[aId];
        std::vector<std::vector<VertexId>>& actionGridBirthVertex = gridBirthVertex[aId];
        std::vector<std::vector<FaceId>>& actionGridBirthFace = gridBirthFace[aId];
        std::vector<internal::FloatGridPtr> actionUnsignedGrids;
        std::vector<internal::FloatGridPtr> actionSignedGrids;
        std::vector<openvdb::Vec3i> bbMin;
        std::vector<openvdb::Vec3i> bbMax;

        //Get grids
        internal::getClosedGrids(
            action.operation,
            actionModels,
            actionVertexSelectValues,
            scaleFactor,
            maxDistance,
            actionUnsignedGrids,
            actionSignedGrids,
            actionClosedGrids,
            actionPolygonGrids,
            actionGridBirthVertex,
            actionGridBirthFace,
            bbMin,
            bbMax);

        //Destroy the not used grids
        actionUnsignedGrids.clear();
        actionSignedGrids.clear();

        //Minimum and maximum coordinates in the scalar fields
        openvdb::Vec3i minCoord(
            std::numeric_limits<int>::max(),
            std::numeric_limits<int>::max(),
            std::numeric_limits<int>::max());
        openvdb::Vec3i maxCoord(
            std::numeric_limits<int>::min(),
            std::numeric_limits<int>::min(),
            std::numeric_limits<int>::min());

        for (Index mId = 0; mId < models.size(); ++mId) {
            minCoord = openvdb::Vec3i(
                std::min(minCoord.x(), bbMin[mId].x()),
                std::min(minCoord.y(), bbMin[mId].y()),
                std::min(minCoord.z(), bbMin[mId].z()));

            maxCoord = openvdb::Vec3i(
                std::max(maxCoord.x(), bbMax[mId].x()),
                std::max(maxCoord.y(), bbMax[mId].y()),
                std::max(maxCoord.z(), bbMax[mId].z()));
        }

        //Blend mesh
        Mesh actionBlendedMesh = internal::getBlendedMesh(
                action.operation,
                actionModels,
                actionVertexSelectValues,
                scaleFactor,
                maxDistance,
                actionClosedGrids,
                actionPolygonGrids,
                actionGridBirthFace,
                minCoord,
                maxCoord);


    #ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_blendedMesh.obj", actionBlendedMesh);
    #endif

        //Create meshes
        Mesh actionNewMesh;
        Mesh actionPreMesh;

        std::unordered_set<VertexId> preNonSnappableVertices;

        //Create preserved
        std::vector<std::pair<nvl::Index, VertexId>> actionPreBirthVertex;
        std::vector<std::pair<nvl::Index, FaceId>> actionPreBirthFace;
        for (Index mId = 0; mId < actionModels.size(); ++mId) {
            const Mesh& mesh = actionModels[mId]->mesh;

            //Find vertices that were already in the border of the original mesh
            std::vector<VertexId> borderVertices = nvl::meshBorderVertices(mesh);
            std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());

            nvl::Size lastVertexId = actionPreMesh.nextVertexId();
            nvl::Size lastFaceId = actionPreMesh.nextFaceId();

            std::vector<VertexId> tmpBirthVertex;
            std::vector<FaceId> tmpBirthFace;
            nvl::meshTransferFaces(mesh, std::vector<FaceId>(actionFacesToKeep[mId].begin(), actionFacesToKeep[mId].end()), actionPreMesh, tmpBirthVertex, tmpBirthFace);

            actionPreBirthVertex.resize(actionPreMesh.nextVertexId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
            actionPreBirthFace.resize(actionPreMesh.nextFaceId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
            for (Index vId = lastVertexId; vId < actionPreMesh.nextVertexId(); ++vId) {
                assert(tmpBirthVertex[vId] != nvl::MAX_INDEX);
                actionPreBirthVertex[vId] = std::make_pair(mId, tmpBirthVertex[vId]);
            }
            for (Index fId = lastFaceId; fId < actionPreMesh.nextFaceId(); ++fId) {
                assert(tmpBirthFace[fId] != nvl::MAX_INDEX);
                actionPreBirthFace[fId] = std::make_pair(mId, tmpBirthFace[fId]);
            }

            //Non snappable vertices in preserved mesh (border or select value equal to 1)
            for (VertexId vId = lastVertexId; vId < actionPreMesh.nextVertexId(); vId++) {
                if (actionPreMesh.isVertexDeleted(vId))
                    continue;

                if (borderVerticesSet.find(tmpBirthVertex[vId]) != borderVerticesSet.end()) {
                    preNonSnappableVertices.insert(vId);
                }
            }
        }

    #ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_premesh_1_initial.obj", actionPreMesh);
    #endif

        //Fill vertices to keep in the blended mesh
        std::unordered_set<FaceId> actionBlendedFacesToKeep;
        for (FaceId fId = 0; fId < actionBlendedMesh.nextFaceId(); ++fId) {
            if (actionBlendedMesh.isFaceDeleted(fId))
                continue;

            bool isNewSurface = true;

            for (VertexId vId : actionBlendedMesh.face(fId).vertexIds()) {
                const Point& point = actionBlendedMesh.vertex(vId).point();

                Point scaledPoint = scaleTransform * point;
                openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

                for (Index mId = 0; mId < actionModels.size(); ++mId) {
                    internal::IntGrid::ConstAccessor polygonAccessor = actionPolygonGrids[mId]->getConstAccessor();

                    internal::IntGrid::ValueType pId = polygonAccessor.getValue(coord);

                    if (pId >= 0) {
                        FaceId originFaceId = actionGridBirthFace[mId][pId];

                        if (actionFacesToKeep[mId].find(originFaceId) != actionFacesToKeep[mId].end()) {
                            isNewSurface = false;
                        }
                    }
                }
            }

            if (isNewSurface) {
                actionBlendedFacesToKeep.insert(fId);
            }
        }

        //Regularization
        nvl::meshOpenFaceSelection(actionBlendedMesh, actionBlendedFacesToKeep);

        //Transfer in the new surface the faces to be kept
        nvl::meshTransferFaces(actionBlendedMesh, std::vector<VertexId>(actionBlendedFacesToKeep.begin(), actionBlendedFacesToKeep.end()), actionNewMesh);

    #ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_newmesh_1_regularized.obj", actionNewMesh);
    #endif

        //Attach mesh borders to the preserved mesh
        double attachingMaxDistance = voxelSize * 10;
        std::unordered_set<VertexId> newSnappedVertices;
        std::unordered_set<VertexId> preSnappedVertices;
        internal::attachMeshesByBorders(actionNewMesh, actionPreMesh, attachingMaxDistance, std::unordered_set<VertexId>(), preNonSnappableVertices, newSnappedVertices, preSnappedVertices);

    #ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_newmesh_2_attached.obj", actionNewMesh);
    #endif

        std::vector<FaceId> preNotUsedFaces = internal::getPreNotUsedFacesAfterAttaching(actionPreMesh, preNonSnappableVertices, preSnappedVertices);
        for (FaceId& fId : preNotUsedFaces) {
            actionFacesToKeep[actionPreBirthFace[fId].first].erase(actionPreBirthFace[fId].second);
        }

    #ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_premesh.obj", actionPreMesh);
    #endif

        //Select vertices to smooth
        std::vector<VertexId> borderVerticesToSmooth;
        std::vector<double> borderVerticesToSmoothAlpha;
        std::vector<VertexId> innerVerticesToSmooth;
        std::vector<double> innerVerticesToSmoothAlpha;

        std::vector<std::vector<FaceId>> newFFAdj = nvl::meshFaceFaceAdjacencies(actionNewMesh);

        for (VertexId vId = 0; vId < actionNewMesh.nextVertexId(); ++vId) {
            if (actionNewMesh.isVertexDeleted(vId))
                continue;

            if (nvl::meshIsBorderVertex(actionNewMesh, vId, newFFAdj))
                continue;

            const Point& point = actionNewMesh.vertex(vId).point();

            Point scaledPoint = scaleTransform * point;
            internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

            double maxSelectValue = 0.0;
            for (Index mId = 0; mId < actionModels.size(); ++mId) {
                const Mesh& mesh = actionModels[mId]->mesh;

                internal::IntGrid::ConstAccessor polygonAccessor = actionPolygonGrids[mId]->getConstAccessor();
                internal::IntGrid::ValueType pId = polygonAccessor.getValue(coord);

                if (pId >= 0) {
                    FaceId originFaceId = actionGridBirthFace[mId][pId];
                    double selectValue = internal::interpolateFaceSelectValue(mesh, originFaceId, point, *actionVertexSelectValues[mId]);
                    maxSelectValue = std::max(maxSelectValue, selectValue);
                }
            }

            if (maxSelectValue >= SMOOTHING_THRESHOLD) {
                const double borderSmoothingAlpha = 1.0 - (maxSelectValue - SMOOTHING_THRESHOLD) / (1.0 - SMOOTHING_THRESHOLD);
                borderVerticesToSmooth.push_back(vId);
                borderVerticesToSmoothAlpha.push_back(borderSmoothingAlpha);
            }

            const double innerSmoothingAlpha = 1.0 - maxSelectValue;
            innerVerticesToSmooth.push_back(vId);
            innerVerticesToSmoothAlpha.push_back(innerSmoothingAlpha);
        }

        nvl::meshLaplacianSmoothing(actionNewMesh, borderVerticesToSmooth, 5, borderVerticesToSmoothAlpha);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/action_newmesh_3_smoothed_border.obj", actionNewMesh);
#endif

        nvl::meshLaplacianSmoothing(actionNewMesh, innerVerticesToSmooth, 5, 0.7);

    #ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_newmesh_4_smoothed.obj", actionNewMesh);
    #endif

        //Add new mesh and blended mesh to the global variables
        nvl::meshTransferFaces(actionNewMesh, newMesh);
        nvl::meshTransferFaces(actionBlendedMesh, blendedMesh);

        //Delete faces from pre mesh
        for (Index mId = 0; mId < actionModels.size(); ++mId) {
            const Model* model = actionModels[mId];
            const Mesh& mesh = model->mesh;

            std::unordered_set<FaceId>& preFacesToKeepSet = preFacesToKeepMap[model];

            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                if (actionFacesToKeep[mId].find(fId) == actionFacesToKeep[mId].end()) {
                    preFacesToKeepSet.erase(fId);
                }
            }
        }
    }

    Mesh preMesh;

    //Create preserved
    std::vector<std::pair<nvl::Index, VertexId>> preBirthVertex;
    std::vector<std::pair<nvl::Index, FaceId>> preBirthFace;
    for (const Index eId : cluster) {
        const Entry& entry = data.entry(eId);

        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        const std::unordered_set<FaceId>& preFacesToKeepSet = preFacesToKeepMap[model];

        //Find vertices that were already in the border of the original mesh
        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(mesh);
        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());

        nvl::Size lastVertexId = preMesh.nextVertexId();
        nvl::Size lastFaceId = preMesh.nextFaceId();

        std::vector<VertexId> tmpBirthVertex;
        std::vector<FaceId> tmpBirthFace;
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(preFacesToKeepSet.begin(), preFacesToKeepSet.end()), preMesh, tmpBirthVertex, tmpBirthFace);

        preBirthVertex.resize(preMesh.nextVertexId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
        preBirthFace.resize(preMesh.nextFaceId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
        for (Index vId = lastVertexId; vId < preMesh.nextVertexId(); ++vId) {
            assert(tmpBirthVertex[vId] != nvl::MAX_INDEX);
            preBirthVertex[vId] = std::make_pair(eId, tmpBirthVertex[vId]);
        }
        for (Index fId = lastFaceId; fId < preMesh.nextFaceId(); ++fId) {
            assert(tmpBirthFace[fId] != nvl::MAX_INDEX);
            preBirthFace[fId] = std::make_pair(eId, tmpBirthFace[fId]);
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/premesh.obj", preMesh);
#endif

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh.obj", newMesh);
#endif

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);
#endif


    //Get final mesh
    Mesh quadrangulation;
    std::vector<std::pair<nvl::Index, VertexId>> resultPreBirthVertex;
    std::vector<std::pair<nvl::Index, FaceId>> resultPreBirthFace;
    resultMesh = internal::quadrangulateMesh(newMesh, preMesh, blendedMesh, quadrangulation, preBirthVertex, preBirthFace, resultPreBirthVertex, resultPreBirthFace);

    entry.birth.vertex.resize(resultMesh.nextVertexId());
    for (VertexId vId = 0; vId < resultMesh.nextVertexId(); ++vId) {
        if (resultMesh.isVertexDeleted(vId))
            continue;

        std::vector<VertexInfo>& vertexInfo = entry.birth.vertex[vId];

        if (resultPreBirthVertex[vId].first != nvl::MAX_INDEX) {
            VertexInfo info;
            info.eId = resultPreBirthVertex[vId].first;
            info.vId = resultPreBirthVertex[vId].second;
            info.weight = 1.0;
            info.closestFaceId = nvl::MAX_INDEX;
            info.distance = 0.0;
            vertexInfo.push_back(info);
        }
        else {
            const Point& point = resultMesh.vertex(vId).point();

            Point scaledPoint = scaleTransform * point;
            internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

            double bestDistance = nvl::maxLimitValue<double>();
            Index bestActionId = nvl::maxLimitValue<double>();

            for (Index aId = 0; aId < actions.size(); ++aId) {
                std::vector<const Model*>& actionModels = models[aId];

                double currentDistance = 0.0;

                for (Index mId = 0; mId < actionModels.size(); ++mId) {
                    const Mesh& mesh = actionModels[mId]->mesh;

                    internal::FloatGrid::ConstAccessor closedAccessor = closedGrids[aId][mId]->getConstAccessor();
                    internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);

                    currentDistance += std::fabs(closedDistance);
                }

                currentDistance /= actionModels.size();

                if (currentDistance < bestDistance) {
                    bestDistance = currentDistance;
                    bestActionId = aId;
                }
            }

            const Index& actionId = actions[bestActionId];
            const Action& action = data.action(actionId);

            std::vector<const Model*>& actionModels = models[bestActionId];
            const std::vector<const std::vector<double>*>& actionVertexSelectValues = vertexSelectValue[bestActionId];

            internal::FloatGrid::ConstAccessor closedAccessor1 = closedGrids[bestActionId][0]->getConstAccessor();
            internal::IntGrid::ConstAccessor polygonAccessor1 = polygonGrids[bestActionId][0]->getConstAccessor();
            internal::FloatGrid::ValueType closedDistance1 = closedAccessor1.getValue(coord);
            internal::IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);
            const Mesh& mesh1 = actionModels[0]->mesh;
            Index eId1 = data.entryFromModel(actionModels[0]).id;

            if (action.operation == OperationType::REMOVE || action.operation == OperationType::DETACH) {
                if (pId1 >= 0) {
                    FaceId originFaceId1 = gridBirthFace[bestActionId][0][pId1];
                    VertexInfo info;
                    info.eId = eId1;
                    info.vId = nvl::MAX_INDEX;
                    info.weight = 1.0;
                    info.closestFaceId = originFaceId1;
                    info.distance = closedDistance1;
                    vertexInfo.push_back(info);
                }
            }
            else if (action.operation == OperationType::REPLACE) {
                internal::FloatGrid::ConstAccessor closedAccessor2 = closedGrids[bestActionId][1]->getConstAccessor();
                internal::IntGrid::ConstAccessor polygonAccessor2 = polygonGrids[bestActionId][1]->getConstAccessor();
                internal::FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);
                internal::IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);
                const Mesh& mesh2 = actionModels[1]->mesh;
                Index eId2 = data.entryFromModel(actionModels[1]).id;

                if (pId1 >= 0 && pId2 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                    FaceId originFaceId1 = gridBirthFace[bestActionId][0][pId1];
                    double selectValue1 = internal::interpolateFaceSelectValue(mesh1, originFaceId1, point, *actionVertexSelectValues[0]);

                    FaceId originFaceId2 = gridBirthFace[bestActionId][1][pId2];
                    double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, *actionVertexSelectValues[1]);

                    if (selectValue1 >= SELECT_VALUE_MAX_THRESHOLD && selectValue2 < SELECT_VALUE_MAX_THRESHOLD) {
                        VertexInfo info1;
                        info1.eId = eId1;
                        info1.vId = nvl::MAX_INDEX;
                        info1.weight = 1.0;
                        info1.closestFaceId = originFaceId1;
                        info1.distance = closedDistance1;
                        vertexInfo.push_back(info1);
                    }
                    else if (selectValue1 < SELECT_VALUE_MAX_THRESHOLD && selectValue2 >= SELECT_VALUE_MAX_THRESHOLD) {
                        VertexInfo info2;
                        info2.eId = eId2;
                        info2.vId = nvl::MAX_INDEX;
                        info2.weight = 1.0;
                        info2.closestFaceId = originFaceId2;
                        info2.distance = closedDistance2;
                        vertexInfo.push_back(info2);
                    }
                    else if (selectValue1 <= SELECT_VALUE_MIN_THRESHOLD && selectValue2 <= SELECT_VALUE_MIN_THRESHOLD) {
                        VertexInfo info1;
                        info1.eId = eId1;
                        info1.vId = nvl::MAX_INDEX;
                        info1.weight = 0.5;
                        info1.closestFaceId = originFaceId1;
                        info1.distance = closedDistance1;
                        vertexInfo.push_back(info1);

                        VertexInfo info2;
                        info2.eId = eId2;
                        info2.vId = nvl::MAX_INDEX;
                        info2.weight = 0.5;
                        info2.closestFaceId = originFaceId2;
                        info2.distance = closedDistance2;
                        vertexInfo.push_back(info2);
                    }
                    else {
                        VertexInfo info1;
                        info1.eId = eId1;
                        info1.vId = nvl::MAX_INDEX;
                        info1.weight = selectValue1 / (selectValue1 + selectValue2);
                        info1.closestFaceId = originFaceId1;
                        info1.distance = closedDistance1;
                        vertexInfo.push_back(info1);

                        VertexInfo info2;
                        info2.eId = eId2;
                        info2.vId = nvl::MAX_INDEX;
                        info2.weight = selectValue2 / (selectValue1 + selectValue2);
                        info2.closestFaceId = originFaceId2;
                        info2.distance = closedDistance2;
                        vertexInfo.push_back(info2);
                    }
                }
                else if (pId1 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance) {
                    FaceId originFaceId1 = gridBirthFace[bestActionId][0][pId1];
                    VertexInfo info1;
                    info1.eId = eId1;
                    info1.vId = nvl::MAX_INDEX;
                    info1.weight = 1.0;
                    info1.closestFaceId = originFaceId1;
                    info1.distance = closedDistance1;
                    vertexInfo.push_back(info1);
                }
                else if (pId2 >= 0 && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                    FaceId originFaceId2 = gridBirthFace[bestActionId][0][pId1];
                    VertexInfo info2;
                    info2.eId = eId2;
                    info2.vId = nvl::MAX_INDEX;
                    info2.weight = 1.0;
                    info2.closestFaceId = originFaceId2;
                    info2.distance = closedDistance2;
                    vertexInfo.push_back(info2);
                }
            }
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/quadrangulationmesh.obj", quadrangulation);
    nvl::meshSaveToFile("results/resultmesh.obj", resultMesh);
#endif
}

namespace internal {

template<class Mesh>
Mesh quadrangulateMesh(
        const Mesh& newMesh,
        const Mesh& preMesh,
        const Mesh& blendedMesh,
        Mesh& quadrangulation,
        const std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& preBirthVertex,
        const std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& preBirthFace,
        std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& birthVertex,
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& birthFace)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;

    Mesh result;

    nvl::VCGTriangleMesh vcgNewMesh;
    nvl::VCGPolygonMesh vcgPreMesh;
    nvl::VCGPolygonMesh vcgQuadrangulation;
    nvl::VCGPolygonMesh vcgResult;
    nvl::VCGPolygonMesh vcgBlendedMesh;

    nvl::convertMeshToVCGMesh(newMesh, vcgNewMesh);
    nvl::convertMeshToVCGMesh(blendedMesh, vcgBlendedMesh);

    std::vector<nvl::Index> vcgPreBirthVertex;
    std::vector<nvl::Index> vcgPreBirthFace;
    nvl::convertMeshToVCGMesh(preMesh, vcgPreMesh, vcgPreBirthVertex, vcgPreBirthFace);

    QuadRetopology::Parameters par;
    par.ilpMethod = QuadRetopology::LEASTSQUARES;
    par.alpha = 0.5;
    par.isometry = true;
    par.regularityQuadrilaterals = true;
    par.regularityNonQuadrilaterals = false;
    par.regularityNonQuadrilateralsWeight = 0.9;
    par.alignSingularities = false;
    par.alignSingularitiesWeight = 0.1;
    par.repeatLosingConstraintsIterations = 0;
    par.repeatLosingConstraintsQuads = false;
    par.repeatLosingConstraintsNonQuads = false;
    par.repeatLosingConstraintsAlign = false;
    par.feasibilityFix = true;
    par.hardParityConstraint = true;
    par.timeLimit = 200;
    par.gapLimit = 0.0;
    par.callbackTimeLimit = { 3.00, 5.000, 10.0, 20.0, 30.0, 60.0, 90.0, 120.0 };
    par.callbackGapLimit = { 0.001, 0.005, 0.01, 0.05, 0.10, 0.15, 0.20, 0.300 };
    par.minimumGap = 0.3;
    par.chartSmoothingIterations = 5;
    par.quadrangulationFixedSmoothingIterations = 5;
    par.quadrangulationNonFixedSmoothingIterations = 5;
    par.doubletRemoval = true;
    par.resultSmoothingIterations = 5;
    par.resultSmoothingNRing = 3;
    par.resultSmoothingLaplacianIterations = 5;
    par.resultSmoothingLaplacianNRing = 3;

    //Get patch decomposition of the new surface
    std::vector<std::vector<size_t>> newSurfacePartitions;
    std::vector<std::vector<size_t>> newSurfaceCorners;
    std::vector<int> newSurfaceLabel = QuadRetopology::patchDecomposition(vcgNewMesh, vcgPreMesh, newSurfacePartitions, newSurfaceCorners, true, 1.0, true, false, true);

    //Get chart data
    QuadRetopology::ChartData chartData = QuadRetopology::computeChartData(vcgNewMesh, newSurfaceLabel, newSurfaceCorners);

    //Select the subsides to fix
    std::vector<size_t> fixedSubsides;
    for (size_t subsideId = 0; subsideId < chartData.subsides.size(); ++subsideId) {
        QuadRetopology::ChartSubside& subside = chartData.subsides[subsideId];
        if (subside.isOnBorder) {
            fixedSubsides.push_back(subsideId);
        }
    }

    //Get chart length
    std::vector<double> chartEdgeLength = QuadRetopology::computeChartEdgeLength(chartData, fixedSubsides, 5, 0.7);

    //Solve ILP to find best side size
    double gap;
    std::vector<int> ilpResult = QuadRetopology::findSubdivisions(
            chartData,
            fixedSubsides,
            chartEdgeLength,
            par,
            gap);

    //Quadrangulate,
    std::vector<int> quadrangulationLabel;
    std::vector<std::vector<size_t>> quadrangulationPartitions;
    std::vector<std::vector<size_t>> quadrangulationCorners;
    QuadRetopology::quadrangulate(
                vcgNewMesh,
                chartData,
                fixedSubsides,
                ilpResult,
                par,
                vcgQuadrangulation,
                quadrangulationLabel,
                quadrangulationPartitions,
                quadrangulationCorners);

    //Get the result
    std::vector<int> resultPreservedVertexMap;
    std::vector<int> resultPreservedFaceMap;
    QuadRetopology::computeResult(
                vcgPreMesh, vcgQuadrangulation,
                vcgResult, vcgBlendedMesh,
                par,
                resultPreservedVertexMap, resultPreservedFaceMap);

    nvl::convertVCGMeshToMesh(vcgQuadrangulation, quadrangulation);
    std::vector<nvl::Index> vcgResultBirthVertex;
    std::vector<nvl::Index> vcgResultBirthFace;
    nvl::convertVCGMeshToMesh(vcgResult, result, vcgResultBirthVertex, vcgResultBirthFace);

    birthVertex.resize(result.nextVertexId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
    birthFace.resize(result.nextVertexId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
    for (VertexId vId = 0; vId < result.nextVertexId(); vId++) {
        if (result.isVertexDeleted(vId))
            continue;

        nvl::Index vcgResultVId = vcgResultBirthVertex[vId];
        int vcgPreMeshVId = resultPreservedVertexMap[vcgResultVId];
        if (vcgPreMeshVId >= 0) {
            birthVertex[vId] = preBirthVertex[vcgPreBirthVertex[vcgPreMeshVId]];
        }
    }
    for (FaceId fId = 0; fId < result.nextFaceId(); fId++) {
        if (result.isFaceDeleted(fId))
            continue;

        nvl::Index vcgResultFId = vcgResultBirthFace[fId];
        int vcgPreMeshFId = resultPreservedFaceMap[vcgResultFId];
        if (vcgPreMeshFId >= 0) {
            birthFace[fId] = preBirthFace[vcgPreBirthFace[vcgPreMeshFId]];
        }
    }

    return result;
}

}

}
