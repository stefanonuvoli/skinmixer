#include "skinmixer_blend_surfaces.h"

#include "internal/skinmixer_attach_borders.h"
#include "internal/skinmixer_field.h"
#include "internal/skinmixer_morphological_operations.h"

#include <nvl/math/numeric_limits.h>
#include <nvl/math/transformations.h>

#include <nvl/models/mesh_geometric_information.h>
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
#define SMOOTHING_BORDER_ITERATIONS 10
#define SMOOTHING_INNER_ITERATIONS 10
#define NEWSURFACE_DISTANCE_THRESHOLD 0.001
#define PRESERVE_GAP_THRESHOLD 0.01
#define PRESERVE_DISTANCE_THRESHOLD 1.0
#define PRESERVE_REGULARIZATION_ITERATIONS 1
//#define PRESERVE_MAX_DISTANCE 2.0
//#define PRESERVE_DISTANCE_THRESHOLD 0.5
//#define PRESERVE_SELECT_THRESHOLD 0.5
//#define PRESERVE_FLAG_UNKNOWN -1
//#define PRESERVE_FLAG_NOT_FOUND -2
//#define PRESERVE_FLAG_DISPUTED -3
//#define PRESERVE_EXPANSION 5

namespace skinmixer {

namespace internal {

template<class Model>
typename Model::Mesh getPreservedMesh(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        std::unordered_map<const Model*, std::unordered_set<typename Model::Mesh::FaceId>>& preservedFacesPerModel,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::VertexId>>& preBirthVertex,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::FaceId>>& preBirthFace);

template<class Model>
std::unordered_map<const Model*, std::unordered_set<typename Model::Mesh::FaceId>> getPreservedFacesPerModel(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        const std::vector<const Model*>& models,
        const std::vector<std::unordered_set<typename Model::Mesh::FaceId>>& preservedFaces);

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
        SkinMixerData<Model>& data,
        std::vector<nvl::Index>& newEntries)
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

    //Find cluster
    std::vector<nvl::Index> cluster;
    for (const Action& action : data.actions()) {
        cluster.push_back(action.entry1);
        if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
            cluster.push_back(action.entry2);
        }
    }
    std::sort(cluster.begin(), cluster.end());
    cluster.erase(std::unique(cluster.begin(), cluster.end()), cluster.end());

    //New model
    Model* resultModel = new Model();
    Mesh& resultMesh = resultModel->mesh;

    Index newEntryId = data.addEntry(resultModel);
    Entry& resultEntry = data.entry(newEntryId);
    newEntries.push_back(newEntryId);

    resultEntry.birth.entries = cluster;

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


    //Scale transform
    double scaleFactor = 1.0 / voxelSize;
    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);






    //Grid data
    std::vector<const Model*> models(cluster.size());
    std::vector<std::vector<double>> vertexSelectValues(cluster.size());
    std::vector<Mesh> inputMeshes(cluster.size());
    std::vector<Mesh> closedMeshes(cluster.size());
    std::vector<internal::FloatGridPtr> closedGrids(cluster.size());
    std::vector<internal::IntGridPtr> polygonGrids(cluster.size());
    std::vector<std::vector<VertexId>> fieldBirthVertex(cluster.size());
    std::vector<std::vector<FaceId>> fieldBirthFace(cluster.size());
    std::vector<openvdb::Vec3i> bbMin(cluster.size());
    std::vector<openvdb::Vec3i> bbMax(cluster.size());
    std::vector<std::unordered_set<FaceId>> facesInField(cluster.size());
    std::vector<std::unordered_set<FaceId>> preservedFaces(cluster.size());
    std::vector<std::vector<std::vector<FaceId>>> ffAdjs(cluster.size());

    //Create grid for each model
    std::unordered_map<Index, Index> clusterMap;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Index& eId = cluster[cId];

        clusterMap.insert(std::make_pair(eId, cId));

        const Entry& entry = data.entry(eId);
        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        //Model
        models[cId] = model;

        //Vertex select values
        SelectInfo globalSelectInfo = data.computeGlobalSelectInfo(cId);
        vertexSelectValues[cId] = globalSelectInfo.vertex;

        //Find faces in the field
        facesInField[cId] = internal::findFacesInField(mesh, vertexSelectValues[cId], scaleFactor);


        //Find faces in the field
        ffAdjs[cId] = nvl::meshFaceFaceAdjacencies(mesh);

        //Transfer vertices to keep in the current mesh
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(facesInField[cId].begin(), facesInField[cId].end()), inputMeshes[cId], fieldBirthVertex[cId], fieldBirthFace[cId]);

        //Scale mesh
        const nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);
        nvl::meshApplyTransformation(inputMeshes[cId], scaleTransform);

        internal::getClosedGrid(inputMeshes[cId], maxDistance, closedMeshes[cId], closedGrids[cId], polygonGrids[cId], bbMin[cId], bbMax[cId]);

#ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/field_input_" + std::to_string(cId) + ".obj", inputMeshes[cId]);
        nvl::meshSaveToFile("results/field_closed_" + std::to_string(cId) + ".obj", closedMeshes[cId]);
#endif
    }

    



    //Blended, preserved and new mesh
    Mesh blendedMesh;
    Mesh newMesh;
    Mesh preMesh;


    //Blend grids
    internal::FloatGridPtr blendedGrid;
    internal::IntGridPtr activeActionGrid;
    internal::getBlendedGrid(
        data,
        cluster,
        clusterMap,
        actions,
        models,
        vertexSelectValues,
        closedGrids,
        polygonGrids,
        fieldBirthFace,
        bbMin,
        bbMax,
        scaleFactor,
        maxDistance,
        blendedGrid,
        activeActionGrid);

    //Extract iso surface mesh
    blendedMesh = internal::convertGridToMesh<Mesh>(blendedGrid, true);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/blendedMesh_non_rescaled.obj", blendedMesh);
#endif

    //Rescale back
    nvl::meshApplyTransformation(blendedMesh, scaleTransform.inverse());

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);
#endif





//        //Add faces with select values over the threshold
//        for (Index mId = 0; mId < actionModels.size(); ++mId) {
//            const Mesh& mesh = actionModels[mId]->mesh;

//            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
//                if (mesh.isFaceDeleted(fId)) {
//                    continue;
//                }

//                double selectValue = internal::averageFaceSelectValue(mesh, fId, *actionVertexSelectValues[mId]);

//                if (selectValue >= FACE_KEEP_THRESHOLD) {
//                    actionPreservedFaces[mId].insert(fId);
//                }
//            }

//            //Replace
//            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
//                if (mesh.isFaceDeleted(fId)) {
//                    continue;
//                }

//                double selectValue = internal::averageFaceSelectValue(mesh, fId, *actionVertexSelectValues[mId]);

//                const Face& face = mesh.face(fId);

//                bool toKeep = false;

//                if (actionFacesInField[mId].find(fId) != actionFacesInField[mId].end()) {
//                    toKeep = true;
//                    for (Index j = 0; j < face.vertexNumber(); ++j) {
//                        const Point& point = mesh.vertex(face.vertexId(j)).point();

//                        Point scaledPoint = scaleTransform * point;
//                        internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

//                        internal::FloatGrid::ConstAccessor blendedAccessor = blendedGrid->getConstAccessor();
//                        internal::FloatGrid::ValueType blendedDistance = blendedAccessor.getValue(coord);

//                        internal::FloatGrid::ConstAccessor closedAccessor = closedGrids[aId][mId]->getConstAccessor();
//                        internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);

//                        if (std::fabs(closedDistance - blendedDistance) > PRESERVE_GAP_THRESHOLD || std::fabs(blendedDistance) > PRESERVE_DISTANCE_THRESHOLD) {
//                            toKeep = false;
//                        }
//                    }
//                }
//                else if (actionFacesInField[mId].find(fId) == actionFacesInField[mId].end() && selectValue >= 0.5) {
//                    toKeep = true;
//                }
//                else if (actionFacesInField[mId].find(fId) == actionFacesInField[mId].end() && selectValue < 0.5) {
//                    toKeep = false;
//                }

//                if (toKeep) {
//                    actionPreservedFaces[mId].insert(fId);
//                }
//            }

//            //Attach
//            if (action.operation == OperationType::ATTACH) {
//                for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
//                    if (mesh.isFaceDeleted(fId)) {
//                        continue;
//                    }

//                    double selectValue = internal::averageFaceSelectValue(mesh, fId, *actionVertexSelectValues[mId]);

//                    bool toDelete;
//                    if (actionFacesInField[mId].find(fId) != actionFacesInField[mId].end()) {
//                        const Face& face = mesh.face(fId);

//                        toDelete = false;
//                        for (Index j = 0; j < face.vertexNumber(); ++j) {
//                            const Point& point = mesh.vertex(face.vertexId(j)).point();

//                            Point scaledPoint = scaleTransform * point;
//                            internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

//                            internal::FloatGrid::ConstAccessor blendedAccessor = blendedGrid->getConstAccessor();
//                            internal::FloatGrid::ValueType blendedDistance = blendedAccessor.getValue(coord);

//                            internal::FloatGrid::ConstAccessor closedAccessor = closedGrids[aId][mId]->getConstAccessor();
//                            internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);

//                            if (std::fabs(closedDistance) <= 0.5 && std::fabs(closedDistance - blendedDistance) > 0.5) {
//                                toDelete = true;
//                            }
//                        }
//                    }
//                    else if (actionFacesInField[mId].find(fId) == actionFacesInField[mId].end() && selectValue > 0.5) {
//                        toDelete = false;
//                    }
//                    else if (actionFacesInField[mId].find(fId) == actionFacesInField[mId].end() && selectValue < 0.5) {
//                        toDelete = true;
//                    }
//                    if (toDelete) {
//                        actionPreservedFaces[mId].erase(fId);
//                    }
//                }
//            }


//    //Initialize faces to be preserved
//    for (Index aId = 0; aId < actions.size(); ++aId) {
//        const Index& actionId = actions[aId];
//        const Action& action = data.action(actionId);

//        const std::vector<const Model*>& actionModels = models[aId];
//        const std::vector<std::unordered_set<FaceId>>& actionFacesInField = facesInField[aId];
//        std::vector<std::unordered_set<FaceId>>& actionPreservedFaces = preservedFaces[aId];

//        actionPreservedFaces.resize(actionModels.size());

//        //Add all faces
//        for (Index mId = 0; mId < actionModels.size(); ++mId) {
//            const Mesh& mesh = actionModels[mId]->mesh;

//            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
//                if (mesh.isFaceDeleted(fId)) {
//                    continue;
//                }

//                actionPreservedFaces[mId].insert(fId);
//            }
//        }
//    }


    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Mesh& mesh = models[cId]->mesh;

        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValues[cId]);

            if (selectValue >= 0.5) {
                preservedFaces[cId].insert(fId);
            }
        }
    }

    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Mesh& mesh = models[cId]->mesh;

        //Add
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            const Face& face = mesh.face(fId);

            for (Index j = 0; j < face.vertexNumber(); ++j) {
                const Point& point = mesh.vertex(face.vertexId(j)).point();

                Point scaledPoint = scaleTransform * point;
                internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

                internal::IntGrid::ConstAccessor actionAccessor = activeActionGrid->getConstAccessor();
                internal::IntGrid::ValueType bestActionId = actionAccessor.getValue(coord);

                if (bestActionId < nvl::maxLimitValue<int>()) {
                    const Index& actionId = actions[bestActionId];
                    const Action& action = data.action(actionId);

                    //Entry 1
                    const Index eId1 = action.entry1;
                    assert(eId1 != nvl::MAX_INDEX);
                    const Index cId1 = clusterMap.at(eId1);

                    const internal::FloatGridPtr& closedGrid1 = closedGrids[cId1];
                    const internal::FloatGrid::ConstAccessor closedAccessor1 = closedGrid1->getConstAccessor();
                    const internal::FloatGrid::ValueType closedDistance1 = closedAccessor1.getValue(coord);

                    const internal::IntGridPtr& polygonGrid1 = polygonGrids[cId1];
                    const internal::IntGrid::ConstAccessor polygonAccessor1 = polygonGrid1->getConstAccessor();
                    const internal::IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);

                    if (pId1 >= 0 && std::fabs(closedDistance1) > PRESERVE_DISTANCE_THRESHOLD) {
                        preservedFaces[cId1].erase(fieldBirthFace[cId1][pId1]);
                    }


                    //Entry 2
                    const Index eId2 = action.entry2;
                    if (eId2 != nvl::MAX_INDEX) {
                        const Index cId2 = clusterMap.at(eId2);

                        const internal::FloatGridPtr& closedGrid2 = closedGrids[cId2];
                        const internal::FloatGrid::ConstAccessor closedAccessor2 = closedGrid2->getConstAccessor();
                        const internal::FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);

                        const internal::IntGridPtr& polygonGrid2 = polygonGrids[cId2];
                        const internal::IntGrid::ConstAccessor polygonAccessor2 = polygonGrid2->getConstAccessor();
                        const internal::IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);

                        if (pId2 >= 0 && std::fabs(closedDistance2) > PRESERVE_DISTANCE_THRESHOLD) {
                            preservedFaces[cId2].erase(fieldBirthFace[cId2][pId2]);
                        }
                    }

                }
            }
        }
    }

    for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
        if (blendedMesh.isFaceDeleted(fId))
            continue;

        for (VertexId vId : blendedMesh.face(fId).vertexIds()) {
            const Point& point = blendedMesh.vertex(vId).point();

            Point scaledPoint = scaleTransform * point;
            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

            internal::IntGrid::ConstAccessor actionAccessor = activeActionGrid->getConstAccessor();
            internal::IntGrid::ValueType bestActionId = actionAccessor.getValue(coord);

            if (bestActionId < nvl::maxLimitValue<int>()) {
                const Index& actionId = actions[bestActionId];
                const Action& action = data.action(actionId);

                internal::FloatGrid::ConstAccessor blendedAccessor = blendedGrid->getConstAccessor();
                internal::FloatGrid::ValueType blendedDistance = blendedAccessor.getValue(coord);

                //Entry 1
                const Index eId1 = action.entry1;
                assert(eId1 != nvl::MAX_INDEX);
                const Index cId1 = clusterMap.at(eId1);

                const internal::FloatGridPtr& closedGrid1 = closedGrids[cId1];
                const internal::FloatGrid::ConstAccessor closedAccessor1 = closedGrid1->getConstAccessor();
                const internal::FloatGrid::ValueType closedDistance1 = closedAccessor1.getValue(coord);

                const internal::IntGridPtr& polygonGrid1 = polygonGrids[cId1];
                const internal::IntGrid::ConstAccessor polygonAccessor1 = polygonGrid1->getConstAccessor();
                const internal::IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);

                if (pId1 >= 0 && std::fabs(closedDistance1 - blendedDistance) > PRESERVE_GAP_THRESHOLD) {
                    preservedFaces[cId1].erase(fieldBirthFace[cId1][pId1]);

                    if (action.operation == OperationType::ATTACH) {
                        for (FaceId adjId : ffAdjs[cId1][fieldBirthFace[cId1][pId1]]) {
                            preservedFaces[cId1].erase(adjId);
                        }
                    }
                }


                //Entry 2
                const Index eId2 = action.entry2;
                if (eId2 != nvl::MAX_INDEX) {
                    const Index cId2 = clusterMap.at(eId2);

                    const internal::FloatGridPtr& closedGrid2 = closedGrids[cId2];
                    const internal::FloatGrid::ConstAccessor closedAccessor2 = closedGrid2->getConstAccessor();
                    const internal::FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);

                    const internal::IntGridPtr& polygonGrid2 = polygonGrids[cId2];
                    const internal::IntGrid::ConstAccessor polygonAccessor2 = polygonGrid2->getConstAccessor();
                    const internal::IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);

                    if (pId2 >= 0 && std::fabs(closedDistance2 - blendedDistance) > PRESERVE_GAP_THRESHOLD) {
                        preservedFaces[cId2].erase(fieldBirthFace[cId2][pId2]);

                        if (action.operation == OperationType::ATTACH) {
                            for (FaceId adjId : ffAdjs[cId2][fieldBirthFace[cId2][pId2]]) {
                                preservedFaces[cId2].erase(adjId);
                            }
                        }
                    }
                }

            }
        }
    }


    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Mesh& mesh = models[cId]->mesh;

        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValues[cId]);
            if (facesInField[cId].find(fId) == facesInField[cId].end()) {
                if (selectValue >= 0.5) {
                    preservedFaces[cId].insert(fId);
                }
                else {
                    preservedFaces[cId].erase(fId);
                }
            }
            else if (selectValue > FACE_KEEP_THRESHOLD) {
                preservedFaces[cId].insert(fId);
            }
        }
    }

    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Mesh& mesh = models[cId]->mesh;

//        skinmixer::meshErodeFaceSelectionNoBorders(mesh, preservedFaces[cId]);

        //Regularization
        for (int i = 0; i < PRESERVE_REGULARIZATION_ITERATIONS; ++i) {
            skinmixer::meshCloseFaceSelectionNoBorders(mesh, preservedFaces[cId]);
            skinmixer::meshOpenFaceSelectionNoBorders(mesh, preservedFaces[cId]);
        }
    }





    //Create preserved
    std::vector<std::pair<nvl::Index, VertexId>> preBirthVertex;
    std::vector<std::pair<nvl::Index, FaceId>> preBirthFace;
    std::unordered_map<const Model*, std::unordered_set<typename Model::Mesh::FaceId>> preservedFacesPerModel;
    preservedFacesPerModel = internal::getPreservedFacesPerModel(data, cluster, models, preservedFaces);
    preMesh = internal::getPreservedMesh(data, cluster, preservedFacesPerModel, preBirthVertex, preBirthFace);

    //Find non snappable vertices in the preserved mesh
    std::unordered_set<VertexId> preNonSnappableVertices;
    for (const Index eId : cluster) {

        const Entry& entry = data.entry(eId);

        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        //Find vertices that were already in the border of the original mesh
        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(mesh);
        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());

        for (VertexId vId = 0; vId < preMesh.nextVertexId(); vId++) {
            if (preMesh.isVertexDeleted(vId))
                continue;

            if (preBirthVertex[vId].first == eId && borderVerticesSet.find(preBirthVertex[vId].second) != borderVerticesSet.end()) {
                preNonSnappableVertices.insert(vId);
            }
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/premesh_1_initial.obj", preMesh);
#endif







    //Fill faces to keep in the blended mesh
    std::unordered_set<FaceId> newSurfaceFaces;

    for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
        if (blendedMesh.isFaceDeleted(fId))
            continue;

        bool isNewSurface = true;

        for (VertexId vId : blendedMesh.face(fId).vertexIds()) {
            const Point& point = blendedMesh.vertex(vId).point();

            Point scaledPoint = scaleTransform * point;
            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

            internal::IntGrid::ConstAccessor actionAccessor = activeActionGrid->getConstAccessor();
            internal::IntGrid::ValueType bestActionId = actionAccessor.getValue(coord);

            if (bestActionId < nvl::maxLimitValue<int>()) {
                const Index& actionId = actions[bestActionId];
                const Action& action = data.action(actionId);


                internal::FloatGrid::ConstAccessor blendedAccessor = blendedGrid->getConstAccessor();
                internal::FloatGrid::ValueType blendedDistance = blendedAccessor.getValue(coord);


                //Entry 1
                const Index eId1 = action.entry1;
                assert(eId1 != nvl::MAX_INDEX);
                const Index cId1 = clusterMap.at(eId1);

                const internal::FloatGridPtr& closedGrid1 = closedGrids[cId1];
                const internal::FloatGrid::ConstAccessor closedAccessor1 = closedGrid1->getConstAccessor();
                const internal::FloatGrid::ValueType closedDistance1 = closedAccessor1.getValue(coord);

                const internal::IntGridPtr& polygonGrid1 = polygonGrids[cId1];
                const internal::IntGrid::ConstAccessor polygonAccessor1 = polygonGrid1->getConstAccessor();
                const internal::IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);

                if (pId1 >= 0 && preservedFaces[cId1].find(fieldBirthFace[cId1][pId1]) != preservedFaces[cId1].end() && std::fabs(blendedDistance - closedDistance1) <= NEWSURFACE_DISTANCE_THRESHOLD) {
                    isNewSurface = false;
                }


                //Entry 2
                const Index eId2 = action.entry2;
                if (eId2 != nvl::MAX_INDEX) {
                    const Index cId2 = clusterMap.at(eId2);

                    const internal::FloatGridPtr& closedGrid2 = closedGrids[cId2];
                    const internal::FloatGrid::ConstAccessor closedAccessor2 = closedGrid2->getConstAccessor();
                    const internal::FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);

                    const internal::IntGridPtr& polygonGrid2 = polygonGrids[cId2];
                    const internal::IntGrid::ConstAccessor polygonAccessor2 = polygonGrid2->getConstAccessor();
                    const internal::IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);

                    if (pId2 >= 0 && preservedFaces[cId2].find(fieldBirthFace[cId2][pId2]) != preservedFaces[cId2].end() && std::fabs(blendedDistance - closedDistance2) <= NEWSURFACE_DISTANCE_THRESHOLD) {
                        isNewSurface = false;
                    }
                }

            }
        }

        if (isNewSurface) {
            newSurfaceFaces.insert(fId);
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    //Transfer in the new surface the faces to be kept
    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(newSurfaceFaces.begin(), newSurfaceFaces.end()), newMesh);
    nvl::meshSaveToFile("results/newmesh_1_non_regularized.obj", newMesh);
    newMesh.clear();
#endif

    //Regularization
    skinmixer::meshCloseFaceSelectionNoBorders(blendedMesh, newSurfaceFaces);
    skinmixer::meshOpenFaceSelectionNoBorders(blendedMesh, newSurfaceFaces);

    //Transfer in the new surface the faces to be kept
    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(newSurfaceFaces.begin(), newSurfaceFaces.end()), newMesh);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh_2_regularized.obj", newMesh);
#endif











    //Attach mesh borders to the preserved mesh
    double attachingMaxDistance = voxelSize * 10;
    std::unordered_set<VertexId> newSnappedVertices;
    std::unordered_set<VertexId> preSnappedVertices;
    internal::attachMeshesByBorders(newMesh, preMesh, attachingMaxDistance, std::unordered_set<VertexId>(), preNonSnappableVertices, newSnappedVertices, preSnappedVertices);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh_2_attached.obj", newMesh);
#endif







    //Create new preserved mesh deleting the not used faces
    std::vector<FaceId> preNotUsedFaces = internal::getPreNotUsedFacesAfterAttaching(preMesh, preNonSnappableVertices, preSnappedVertices);
    for (FaceId& fId : preNotUsedFaces) {
        const Index eId = preBirthFace[fId].first;
        const Entry& entry = data.entry(eId);
        const Model* model = entry.model;

        preservedFacesPerModel[model].erase(preBirthFace[fId].second);
    }

    preBirthVertex.clear();
    preBirthFace.clear();
    preMesh = internal::getPreservedMesh(data, cluster, preservedFacesPerModel, preBirthVertex, preBirthFace);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/premesh.obj", preMesh);
#endif









    //Select vertices to smooth
    std::vector<VertexId> borderVerticesToSmooth;
    std::vector<double> borderVerticesToSmoothAlpha;
    std::vector<VertexId> innerVerticesToSmooth;

    std::vector<std::vector<FaceId>> newFFAdj = nvl::meshFaceFaceAdjacencies(newMesh);

    for (VertexId vId = 0; vId < newMesh.nextVertexId(); ++vId) {
        if (newMesh.isVertexDeleted(vId))
            continue;

        if (nvl::meshIsBorderVertex(newMesh, vId, newFFAdj))
            continue;

        const Point& point = newMesh.vertex(vId).point();

        Point scaledPoint = scaleTransform * point;
        internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));


        internal::IntGrid::ConstAccessor actionAccessor = activeActionGrid->getConstAccessor();
        internal::IntGrid::ValueType bestActionId = actionAccessor.getValue(coord);

        if (bestActionId < nvl::maxLimitValue<int>()) {
            const Index& actionId = actions[bestActionId];
            const Action& action = data.action(actionId);

            double maxSelectValue = 0.0;

            //Entry 1
            const Index eId1 = action.entry1;
            assert(eId1 != nvl::MAX_INDEX);
            const Index cId1 = clusterMap.at(eId1);

            const internal::IntGridPtr& polygonGrid1 = polygonGrids[cId1];
            const internal::IntGrid::ConstAccessor polygonAccessor1 = polygonGrid1->getConstAccessor();
            const internal::IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);

            const std::vector<double>& vertexSelectValues1 = vertexSelectValues[cId1];
            const std::vector<FaceId>& fieldBirthFace1 = fieldBirthFace[cId1];

            const Mesh& mesh1 = models[cId1]->mesh;

            if (pId1 >= 0) {
                FaceId originFaceId1 = fieldBirthFace1[pId1];
                double selectValue1 = internal::interpolateFaceSelectValue(mesh1, originFaceId1, point, vertexSelectValues1);
                maxSelectValue = std::max(maxSelectValue, selectValue1);
            }

            //Entry 2
            const Index eId2 = action.entry2;
            if (eId2 != nvl::MAX_INDEX) {
                const Index cId2 = clusterMap.at(eId2);

                const internal::IntGridPtr& polygonGrid2 = polygonGrids[cId2];
                const internal::IntGrid::ConstAccessor polygonAccessor2 = polygonGrid2->getConstAccessor();
                const internal::IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);

                const std::vector<double>& vertexSelectValues2 = vertexSelectValues[cId2];
                const std::vector<FaceId>& fieldBirthFace2 = fieldBirthFace[cId2];

                const Mesh& mesh2 = models[cId2]->mesh;

                if (pId2 >= 0) {
                    FaceId originFaceId2 = fieldBirthFace2[pId2];
                    double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, vertexSelectValues2);
                    maxSelectValue = std::max(maxSelectValue, selectValue2);
                }
            }

            if (maxSelectValue >= SMOOTHING_THRESHOLD) {
                const double borderSmoothingAlpha = 1.0 - (maxSelectValue - SMOOTHING_THRESHOLD) / (1.0 - SMOOTHING_THRESHOLD);
                borderVerticesToSmooth.push_back(vId);
                borderVerticesToSmoothAlpha.push_back(borderSmoothingAlpha);
            }

            innerVerticesToSmooth.push_back(vId);
        }
    }

    //Laplacian adaptive
    nvl::meshLaplacianSmoothing(newMesh, borderVerticesToSmooth, SMOOTHING_BORDER_ITERATIONS, borderVerticesToSmoothAlpha);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh_3_border_smoothing.obj", newMesh);
#endif

    //Total laplacian
    nvl::meshLaplacianSmoothing(newMesh, innerVerticesToSmooth, SMOOTHING_INNER_ITERATIONS, 0.7);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh.obj", newMesh);
#endif














    //Get final mesh
    Mesh quadrangulation;
    std::vector<std::pair<nvl::Index, VertexId>> resultPreBirthVertex;
    std::vector<std::pair<nvl::Index, FaceId>> resultPreBirthFace;
    resultMesh = internal::quadrangulateMesh(newMesh, preMesh, blendedMesh, quadrangulation, preBirthVertex, preBirthFace, resultPreBirthVertex, resultPreBirthFace);


    //Compute and fill the birth infos
    resultEntry.birth.vertex.resize(resultMesh.nextVertexId());
    for (VertexId vId = 0; vId < resultMesh.nextVertexId(); ++vId) {
        if (resultMesh.isVertexDeleted(vId))
            continue;

        std::vector<VertexInfo>& vertexInfo = resultEntry.birth.vertex[vId];

        nvl::Index birthEId = resultPreBirthVertex[vId].first;
        nvl::Index birthVId = resultPreBirthVertex[vId].second;

        if (birthEId != nvl::MAX_INDEX) {
            VertexInfo info;
            info.eId = birthEId;
            info.vId = birthVId;
            info.weight = 1.0;
            info.closestFaceId = nvl::MAX_INDEX;
            info.distance = 0.0;
            vertexInfo.push_back(info);
        }
        else {
            const Point& point = resultMesh.vertex(vId).point();

            Point scaledPoint = scaleTransform * point;
            internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

            internal::IntGrid::ConstAccessor actionAccessor = activeActionGrid->getConstAccessor();
            internal::IntGrid::ValueType bestActionId = actionAccessor.getValue(coord);

//            double bestDistance = nvl::maxLimitValue<double>();
//            Index bestActionId = nvl::maxLimitValue<double>();

//            for (Index aId = 0; aId < actions.size(); ++aId) {
//                std::vector<const Model*>& actionModels = models[aId];

//                double currentDistance = nvl::maxLimitValue<double>();

//                for (Index mId = 0; mId < actionModels.size(); ++mId) {
//                    const Mesh& mesh = actionModels[mId]->mesh;

//                    internal::FloatGrid::ConstAccessor closedAccessor = closedGrids[aId][mId]->getConstAccessor();
//                    internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);

//                    currentDistance += std::fabs(closedDistance);
//                }

//                currentDistance /= actionModels.size();

//                if (currentDistance < nvl::maxLimitValue<double>() && currentDistance < bestDistance) {
//                    bestDistance = currentDistance;
//                    bestActionId = aId;
//                }
//            }

            if (bestActionId < nvl::maxLimitValue<int>()) {
                const Index& actionId = actions[bestActionId];
                const Action& action = data.action(actionId);



                //Entry 1
                const Index eId1 = action.entry1;
                assert(eId1 != nvl::MAX_INDEX);
                const Index cId1 = clusterMap.at(eId1);

                const internal::FloatGridPtr& closedGrid1 = closedGrids[cId1];
                const internal::FloatGrid::ConstAccessor closedAccessor1 = closedGrid1->getConstAccessor();
                const internal::FloatGrid::ValueType closedDistance1 = closedAccessor1.getValue(coord);

                const internal::IntGridPtr& polygonGrid1 = polygonGrids[cId1];
                const internal::IntGrid::ConstAccessor polygonAccessor1 = polygonGrid1->getConstAccessor();
                const internal::IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);

                const std::vector<double>& vertexSelectValues1 = vertexSelectValues[cId1];
                const std::vector<FaceId>& fieldBirthFace1 = fieldBirthFace[cId1];

                const Mesh& mesh1 = models[cId1]->mesh;




                if (action.operation == OperationType::REMOVE || action.operation == OperationType::DETACH) {
                    if (pId1 >= 0) {
                        FaceId originFaceId1 = fieldBirthFace1[pId1];
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
                    //Entry 2
                    const Index eId2 = action.entry2;
                    assert(eId2 != nvl::MAX_INDEX);
                    const Index cId2 = clusterMap.at(eId2);

                    const internal::FloatGridPtr& closedGrid2 = closedGrids[cId2];
                    const internal::FloatGrid::ConstAccessor closedAccessor2 = closedGrid2->getConstAccessor();
                    const internal::FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);

                    const internal::IntGridPtr& polygonGrid2 = polygonGrids[cId2];
                    const internal::IntGrid::ConstAccessor polygonAccessor2 = polygonGrid2->getConstAccessor();
                    const internal::IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);

                    const std::vector<double>& vertexSelectValues2 = vertexSelectValues[cId2];
                    const std::vector<FaceId>& fieldBirthFace2 = fieldBirthFace[cId2];

                    const Mesh& mesh2 = models[cId2]->mesh;


                    if (pId1 >= 0 && pId2 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        FaceId originFaceId1 = fieldBirthFace1[pId1];
                        double selectValue1 = internal::interpolateFaceSelectValue(mesh1, originFaceId1, point, vertexSelectValues1);

                        FaceId originFaceId2 = fieldBirthFace2[pId2];
                        double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, vertexSelectValues2);

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
                        FaceId originFaceId1 = fieldBirthFace1[pId1];
                        VertexInfo info1;
                        info1.eId = eId1;
                        info1.vId = nvl::MAX_INDEX;
                        info1.weight = 1.0;
                        info1.closestFaceId = originFaceId1;
                        info1.distance = closedDistance1;
                        vertexInfo.push_back(info1);
                    }
                    else if (pId2 >= 0 && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        FaceId originFaceId2 = fieldBirthFace2[pId2];
                        VertexInfo info2;
                        info2.eId = eId2;
                        info2.vId = nvl::MAX_INDEX;
                        info2.weight = 1.0;
                        info2.closestFaceId = originFaceId2;
                        info2.distance = closedDistance2;
                        vertexInfo.push_back(info2);
                    }
                }
                else if (action.operation == OperationType::ATTACH) {
                    //Entry 2
                    const Index eId2 = action.entry2;
                    assert(eId2 != nvl::MAX_INDEX);
                    const Index cId2 = clusterMap.at(eId2);

                    const internal::FloatGridPtr& closedGrid2 = closedGrids[cId2];
                    const internal::FloatGrid::ConstAccessor closedAccessor2 = closedGrid2->getConstAccessor();
                    const internal::FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);

                    const internal::IntGridPtr& polygonGrid2 = polygonGrids[cId2];
                    const internal::IntGrid::ConstAccessor polygonAccessor2 = polygonGrid2->getConstAccessor();
                    const internal::IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);

//                    const std::vector<double>& vertexSelectValues2 = vertexSelectValues[cId2];
                    const std::vector<FaceId>& fieldBirthFace2 = fieldBirthFace[cId2];

//                    const Mesh& mesh2 = models[cId2]->mesh;


                    if (pId1 >= 0 && pId2 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        FaceId originFaceId1 = fieldBirthFace1[pId1];
//                    double selectValue1 = internal::interpolateFaceSelectValue(mesh1, originFaceId1, point, vertexSelectValues1);

                        FaceId originFaceId2 = fieldBirthFace2[pId2];
//                    double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, vertexSelectValues2);

                        if (closedDistance1 <= closedDistance2) {
                            VertexInfo info1;
                            info1.eId = eId1;
                            info1.vId = nvl::MAX_INDEX;
                            info1.weight = 1.0;
                            info1.closestFaceId = originFaceId1;
                            info1.distance = closedDistance1;
                            vertexInfo.push_back(info1);
                        }
                        else {
                            VertexInfo info2;
                            info2.eId = eId2;
                            info2.vId = nvl::MAX_INDEX;
                            info2.weight = 1.0;
                            info2.closestFaceId = originFaceId2;
                            info2.distance = closedDistance2;
                            vertexInfo.push_back(info2);
                        }
                    }
                    else if (pId1 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance) {
                        FaceId originFaceId1 = fieldBirthFace1[pId1];
                        VertexInfo info1;
                        info1.eId = eId1;
                        info1.vId = nvl::MAX_INDEX;
                        info1.weight = 1.0;
                        info1.closestFaceId = originFaceId1;
                        info1.distance = closedDistance1;
                        vertexInfo.push_back(info1);
                    }
                    else if (pId2 >= 0 && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        FaceId originFaceId2 = fieldBirthFace2[pId2];
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
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/quadrangulationmesh.obj", quadrangulation);
    nvl::meshSaveToFile("results/resultmesh.obj", resultMesh);
#endif
}

namespace internal {

template<class Model>
typename Model::Mesh getPreservedMesh(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        std::unordered_map<const Model*, std::unordered_set<typename Model::Mesh::FaceId>>& preservedFacesPerModel,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::VertexId>>& preBirthVertex,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::FaceId>>& preBirthFace)
{
    typedef nvl::Index Index;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::Action Action;

    Mesh preMesh;

    //Create preserved
    for (const Index eId : cluster) {
        const Entry& entry = data.entry(eId);

        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        const std::unordered_set<FaceId>& preFacesToKeepSet = preservedFacesPerModel[model];

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

    return preMesh;
}

template<class Model>
std::unordered_map<const Model*, std::unordered_set<typename Model::Mesh::FaceId>> getPreservedFacesPerModel(
    const SkinMixerData<Model>& data,
    const std::vector<nvl::Index>& cluster,
    const std::vector<const Model*>& models,
    const std::vector<std::unordered_set<typename Model::Mesh::FaceId>>& preservedFaces)
{
    typedef nvl::Index Index;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::FaceId FaceId;
    typedef typename SkinMixerData<Model>::Entry Entry;

    std::unordered_map<const Model*, std::unordered_set<typename Model::Mesh::FaceId>> preservedFacesPerModel;

    //Initialize faces to preserved for each model
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

        preservedFacesPerModel.insert(std::make_pair(model, facesToKeep));
    }

    //Delete faces that are not preserved for each model
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Model* model = models[cId];
        const Mesh& mesh = model->mesh;

        std::unordered_set<FaceId>& preFacesToKeepSet = preservedFacesPerModel[model];

        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            if (preservedFaces[cId].find(fId) == preservedFaces[cId].end()) {
                preFacesToKeepSet.erase(fId);
            }
        }
    }

    return preservedFacesPerModel;
}

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
    par.regularityNonQuadrilaterals = true;
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
    std::vector<int> ilpResults(chartData.subsides.size(), ILP_FIND_SUBDIVISION);
    std::vector<size_t> fixedPositionSubsides;
    for (size_t subsideId = 0; subsideId < chartData.subsides.size(); ++subsideId) {
        QuadRetopology::ChartSubside& subside = chartData.subsides[subsideId];
        if (subside.isOnBorder) {
            fixedPositionSubsides.push_back(subsideId);
            ilpResults[subsideId] = subside.size;
        }
    }

    //Get chart length
    std::vector<double> chartEdgeLength = QuadRetopology::computeChartEdgeLength(chartData, 5, ilpResults, 0.7);

    //Solve ILP to find best side size
    double gap;
    QuadRetopology::findSubdivisions(
            chartData,
            chartEdgeLength,
            par,
            gap,
            ilpResults);

    //Quadrangulate,
    std::vector<int> quadrangulationLabel;
    std::vector<std::vector<size_t>> quadrangulationPartitions;
    std::vector<std::vector<size_t>> quadrangulationCorners;
    QuadRetopology::quadrangulate(
                vcgNewMesh,
                chartData,
                fixedPositionSubsides,
                ilpResults,
                par,
                vcgQuadrangulation,
                quadrangulationLabel,
                quadrangulationPartitions,
                quadrangulationCorners);

    //Get the result
    std::vector<int> vcgPreservedVertexMap;
    std::vector<int> vcgPreservedFaceMap;
    QuadRetopology::computeResult(
                vcgPreMesh, vcgQuadrangulation,
                vcgResult, vcgBlendedMesh,
                par,
                vcgPreservedVertexMap, vcgPreservedFaceMap);

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
        int vcgPreMeshVId = vcgPreservedVertexMap[vcgResultVId];
        if (vcgPreMeshVId >= 0) {
            birthVertex[vId] = preBirthVertex[vcgPreBirthVertex[vcgPreMeshVId]];
        }
    }
    for (FaceId fId = 0; fId < result.nextFaceId(); fId++) {
        if (result.isFaceDeleted(fId))
            continue;

        nvl::Index vcgResultFId = vcgResultBirthFace[fId];
        int vcgPreMeshFId = vcgPreservedFaceMap[vcgResultFId];
        if (vcgPreMeshFId >= 0) {
            birthFace[fId] = preBirthFace[vcgPreBirthFace[vcgPreMeshFId]];
        }
    }

    return result;
}

}

}
