#include "skinmixer_blend_surfaces.h"

#include "internal/skinmixer_attach_borders.h"
#include "internal/skinmixer_field.h"

#include <nvl/math/numeric_limits.h>
#include <nvl/math/transformations.h>
#include <nvl/math/normalization.h>
#include <nvl/math/smoothing.h>

#include <nvl/utilities/color_utils.h>

#include <nvl/models/algorithms/mesh_geometric_information.h>
#include <nvl/models/algorithms/mesh_transformations.h>
#include <nvl/models/algorithms/mesh_adjacencies.h>
#include <nvl/models/algorithms/mesh_borders.h>
#include <nvl/models/algorithms/mesh_transfer.h>
#include <nvl/models/algorithms/mesh_smoothing.h>
#include <nvl/models/algorithms/mesh_differentiation.h>
#include <nvl/models/algorithms/mesh_normals.h>
#include <nvl/models/algorithms/mesh_morphological_operations.h>

#include <nvl/models/algorithms/mesh_vcg_convert.h>
#include <nvl/models/algorithms/mesh_grid.h>
#include <nvl/models/algorithms/mesh_remeshing.h>
#include <nvl/models/structures/vcg_triangle_mesh.h>
#include <nvl/models/structures/vcg_polygon_mesh.h>

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
#include <nvl/models/io/mesh_io.h>
#endif

#include <quadretopology/quadretopology.h>

#include <vector>

#include <QImage>

#define FACE_KEEP_THRESHOLD 0.999

#define NEWSURFACE_DISTANCE_THRESHOLD 0.001
#define NEWSURFACE_REMESHING_FACTOR 0.8
#define NEWSURFACE_REMESHING_ITERATIONS 3

#define NEWMESH_SUBDIVISION_ITERATIONS 1
#define NEWMESH_REMESHING_FACTOR 0.8
#define NEWMESH_REMESHING_ITERATIONS 3

#define PRESERVE_GAP_THRESHOLD 0.01
#define PRESERVE_REGULARIZATION_ITERATIONS 2
#define PRESERVE_SELECT_VALUE 0.999
#define PRESERVE_ATTACH_RADIUS 2

#define VERTEX_COLOR_SMOOTHING_ITERATIONS 10
#define VERTEX_COLOR_SMOOTHING_ALPHA 0.8

namespace skinmixer {

namespace internal {

template<class Model>
typename Model::Mesh computePreservedMesh(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        const std::vector<std::unordered_set<typename Model::Mesh::FaceId>>& preservedFaces,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::VertexId>>& preBirthVertex,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::FaceId>>& preBirthFace);

template<class Mesh>
Mesh quadrangulateMesh(
        const Mesh& newMesh,
        const Mesh& preMesh,
        const Mesh& blendedMesh,
        Mesh& quadrangulation,
        const std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& preBirthVertex,
        const std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& preBirthFace,
        std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& birthVertex,
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& birthFace,
        const unsigned int smoothingResultIterations);

template<class Mesh>
nvl::Color interpolateFaceColors(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<nvl::Color>& colors);

}

template<class Model>
void blendSurfaces(
        SkinMixerData<Model>& data,
        std::vector<nvl::Index> cluster,
        typename SkinMixerData<Model>::Entry& resultEntry,
        const MixMode mixMode,
        const bool blendColorFromTextures,
        const unsigned int smoothingBorderIterations,
        const double smoothingBorderThreshold,
        const unsigned int smoothingInnerIterations,
        const double smoothingInnerAlpha,
        const unsigned int smoothingResultIterations,
        const double voxelSizeFactor,
        const double voxelDistanceFactor)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::FaceNormal FaceNormal;
    typedef typename Mesh::MaterialId MaterialId;
    typedef typename Mesh::Material Material;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::WedgeUVId WedgeUVId;
    typedef typename Mesh::VertexUV VertexUV;
    typedef typename nvl::Index Index;
    typedef typename nvl::Vector3d Vector;
    typedef typename SkinMixerData<Model>::BirthInfo::VertexInfo VertexInfo;
    typedef typename SkinMixerData<Model>::SelectInfo SelectInfo;
    typedef typename SkinMixerData<Model>::Action Action;
    typedef typename SkinMixerData<Model>::Entry Entry;

    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename openvdb::math::Coord GridCoord;
    typedef typename openvdb::Vec3R GridVec;
    typedef typename openvdb::math::Transform GridTransform;
    typedef typename GridTransform::Ptr GridTransformPtr;


#ifdef STEP_TIMES
    chrono::steady_clock::time_point start;
    long duration;
    long totalDuration = 0;
#endif



    // --------------------------------------- DEFINITION AND PREPROCESSING DATA ---------------------------------------

#ifdef STEP_TIMES
    start = chrono::steady_clock::now();
    std::cout << "Initialization... ";
#endif

    Mesh& resultMesh = resultEntry.model->mesh; //Resulting mesh
    std::vector<std::pair<Index, VertexId>> resultPreBirthVertex; //Birth vertex infos
    std::vector<std::pair<Index, FaceId>> resultPreBirthFace; //Birth face infos

    double maxDistance; //Max distance of the distance field
    double voxelSize = nvl::maxLimitValue<Scalar>(); //Voxel size

    std::unordered_map<Index, Index> clusterMap; //Cluster map
    std::vector<Index> actions; //List of actions

    double scaleFactor; //Scale factor

    std::vector<const Model*> models(cluster.size()); //Models
    std::vector<std::vector<double>> vertexSelectValues(cluster.size()); //Vertex select value for each model

    std::vector<std::vector<std::vector<FaceId>>> ffAdjs(cluster.size()); //Face-face adjacencies

    std::vector<Mesh> inputMeshes(cluster.size()); //Input meshes
    std::vector<Mesh> closedMeshes(cluster.size()); //Closed meshes
    std::vector<FloatGridPtr> closedGrids(cluster.size()); //Closed grids
    std::vector<IntGridPtr> polygonGrids(cluster.size()); //Polygon grids

    std::vector<std::unordered_set<FaceId>> fieldFaces(cluster.size()); //Face in the fields
    std::vector<std::vector<VertexId>> fieldBirthVertex(cluster.size()); //Birth vertex for field
    std::vector<std::vector<FaceId>> fieldBirthFace(cluster.size()); //Birth face for field

    FloatGridPtr blendedGrid; //Blended grid
    IntGridPtr actionGrid; //Action grid

    Mesh blendedMesh; //Blended mesh



    //Compute data useful in the next steps
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Index& eId = cluster[cId];

        const Entry& entry = data.entry(eId);
        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        //Fill cluster map
        clusterMap.insert(std::make_pair(eId, cId));

        //Model
        models[cId] = model;

        //Vertex select values
        SelectInfo globalSelectInfo = data.computeGlobalSelectInfo(eId);
        vertexSelectValues[cId] = globalSelectInfo.vertex;

        //Find faces in the field
        ffAdjs[cId] = nvl::meshFaceFaceAdjacencies(mesh);

        //Add action
        actions.insert(actions.end(), entry.relatedActions.begin(), entry.relatedActions.end());

        Scalar avgLength = nvl::meshAverageEdgeLength(mesh);
        voxelSize = std::min(voxelSize, avgLength * voxelSizeFactor);
    }

    //Calculate scale factor and transform
    scaleFactor = voxelSize;
    maxDistance = voxelDistanceFactor * scaleFactor;

    //Remove duplicate actions
    std::sort(actions.begin(), actions.end());
    actions.erase(std::unique(actions.begin(), actions.end()), actions.end());

#ifdef STEP_TIMES
    duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
    std::cout << duration << " ms" << std::endl;
    totalDuration += duration;
#endif




    if (mixMode == MixMode::PREVIEW) {
        Mesh preMesh; //Preserved mesh
        Mesh newMesh; //New surface mesh

        std::vector<std::unordered_set<FaceId>> preservedFaces(cluster.size()); //Preserved faces
        std::vector<std::pair<Index, VertexId>> preBirthVertex; //Preserved mesh birth vertices
        std::vector<std::pair<Index, FaceId>> preBirthFace; //Preserved mesh birth faces




        //Create grid for each model
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Index& eId = cluster[cId];

            clusterMap.insert(std::make_pair(eId, cId));

            const Entry& entry = data.entry(eId);
            const Model* model = entry.model;
            const Mesh& mesh = model->mesh;

            //Model
            models[cId] = model;

            //Vertex select values
            SelectInfo globalSelectInfo = data.computeGlobalSelectInfo(eId);
            vertexSelectValues[cId] = globalSelectInfo.vertex;

            //Find faces in the field
            fieldFaces[cId] = internal::findFieldFaces(mesh, vertexSelectValues[cId], scaleFactor);


            //Find faces in the field
            ffAdjs[cId] = nvl::meshFaceFaceAdjacencies(mesh);

            //Transfer vertices to keep in the current mesh
            nvl::meshTransferFaces(mesh, std::vector<FaceId>(fieldFaces[cId].begin(), fieldFaces[cId].end()), inputMeshes[cId], fieldBirthVertex[cId], fieldBirthFace[cId]);
        }


        //Find preserved faces by select values
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



        //Create preserved
        preMesh = internal::computePreservedMesh(data, cluster, preservedFaces, preBirthVertex, preBirthFace);

    #ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/preMesh.obj", preMesh);
    #endif


        //Get final mesh
        Mesh quadrangulation;
        resultMesh = internal::quadrangulateMesh(newMesh, preMesh, blendedMesh, quadrangulation, preBirthVertex, preBirthFace, resultPreBirthVertex, resultPreBirthFace, smoothingResultIterations);


        //Compute and fill the birth infos
        resultEntry.birth.entries = cluster;
        resultEntry.birth.vertex.resize(resultMesh.nextVertexId());
        for (VertexId vId = 0; vId < resultMesh.nextVertexId(); ++vId) {
            if (resultMesh.isVertexDeleted(vId))
                continue;

            std::vector<VertexInfo>& vertexInfo = resultEntry.birth.vertex[vId];

            Index birthEId = resultPreBirthVertex[vId].first;
            Index birthVId = resultPreBirthVertex[vId].second;

            assert(birthEId != nvl::NULL_ID);
            VertexInfo info;
            info.eId = birthEId;
            info.vId = birthVId;
            info.weight = 1.0;
            info.closestFaceId = nvl::NULL_ID;
            info.distance = 0.0;
            vertexInfo.push_back(info);
        }

        return;
    }


    assert(mixMode == MixMode::MESHING || mixMode == MixMode::MORPHING);

    // --------------------------------------- GET GRIDS AND BLENDED GRID ---------------------------------------


#ifdef STEP_TIMES
    start = chrono::steady_clock::now();
    std::cout << "Get fields... ";
#endif

    //Create grid for each model
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Index& eId = cluster[cId];

        clusterMap.insert(std::make_pair(eId, cId));

        const Entry& entry = data.entry(eId);
        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        //Find faces in the field
        fieldFaces[cId] = internal::findFieldFaces(mesh, vertexSelectValues[cId], scaleFactor);

        //Create mesh to give to the field
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(fieldFaces[cId].begin(), fieldFaces[cId].end()), inputMeshes[cId], fieldBirthVertex[cId], fieldBirthFace[cId]);

        //Get grid
        internal::getClosedGrid(inputMeshes[cId], scaleFactor, maxDistance, closedMeshes[cId], closedGrids[cId], polygonGrids[cId]);
    }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        nvl::meshSaveToFile("results/field_input_" + std::to_string(cId) + ".obj", inputMeshes[cId]);
        nvl::meshSaveToFile("results/field_closed_" + std::to_string(cId) + ".obj", closedMeshes[cId]);
    }
#endif

#ifdef STEP_TIMES
    duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
    std::cout << duration << " ms" << std::endl;
    totalDuration += duration;
#endif






#ifdef STEP_TIMES
    start = chrono::steady_clock::now();
    std::cout << "Blend fields... ";
#endif

    //Blend grids
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
        scaleFactor,
        maxDistance,
        blendedGrid,
        actionGrid);

#ifdef STEP_TIMES
    duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
    std::cout << duration << " ms" << std::endl;
    totalDuration += duration;
#endif




    IntGrid::ConstAccessor actionAccessor(actionGrid->getConstAccessor());
    std::vector<IntGrid::ConstAccessor> polygonAccessors;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        polygonAccessors.push_back(polygonGrids[cId]->getConstAccessor());
    }


    if (mixMode == MixMode::MESHING) {

        // --------------------------------------- DEFINITION OF VARIABLES ---------------------------------------

        Mesh preMesh; //Preserved mesh
        Mesh newMesh; //New surface mesh

        std::vector<std::unordered_set<FaceId>> preservedFaces(cluster.size()); //Preserved faces
        std::vector<std::pair<Index, VertexId>> preBirthVertex; //Preserved mesh birth vertices
        std::vector<std::pair<Index, FaceId>> preBirthFace; //Preserved mesh birth faces

        std::vector<VertexId> borderVerticesToSmooth; //Vertices to smooth in the boorder
        std::vector<double> borderVerticesToSmoothAlpha; //Weight for vertices to smooth in the boorder
        std::vector<VertexId> innerVerticesToSmooth; //Inner vertices to smooth



        // --------------------------------------- GET BLENDED MESH ---------------------------------------

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Extract isosurface... ";
#endif
        //Extract iso surface mesh
        blendedMesh = internal::convertGridToMesh<Mesh>(blendedGrid, true);

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/blendedMesh_1_non_remeshed.obj", blendedMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif





#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Remesh blended mesh... ";
#endif
        //Remesh
        double blendedEdgeSize = nvl::meshAverageEdgeLength(blendedMesh);
        blendedMesh = nvl::isotropicRemeshing(blendedMesh, blendedEdgeSize * NEWSURFACE_REMESHING_FACTOR, NEWSURFACE_REMESHING_ITERATIONS, true);

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshComputeFaceNormalsSVDFitting(blendedMesh);
        nvl::meshComputeVertexNormalsFromFaceNormals(blendedMesh);
        nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif




        // --------------------------------------- GET PRESERVED FACES ---------------------------------------

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Get initial preserved faces... ";
#endif

        //Get preserved faces by select values
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Mesh& mesh = models[cId]->mesh;

            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValues[cId]);

                if (selectValue >= PRESERVE_SELECT_VALUE) {
                    preservedFaces[cId].insert(fId);
                }
            }
        }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        preMesh = internal::computePreservedMesh(data, cluster, preservedFaces, preBirthVertex, preBirthFace);
        nvl::meshSaveToFile("results/premesh_1_initial.obj", preMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif






#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Erase faces from preserved mesh... ";
#endif

        //Erase preserved faces where the blended grid is different from the original distances
        for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
            if (blendedMesh.isFaceDeleted(fId))
                continue;

            for (VertexId vId : blendedMesh.faceVertexIds(fId)) {
                const Point& point = blendedMesh.vertexPoint(vId);
                const GridVec vdbWorldPoint(point.x(), point.y(), point.z());
                const GridVec vdbPoint = blendedGrid->worldToIndex(vdbWorldPoint);
                const GridCoord vdbCoord(std::round(vdbPoint.x()), std::round(vdbPoint.y()), std::round(vdbPoint.z()));

                IntGrid::ValueType bestActionId = actionAccessor.getValue(vdbCoord);

                assert(bestActionId >= 0);

                const Index& actionId = actions[bestActionId];
                const Action& action = data.action(actionId);

                const FloatGrid::ValueType blendedDistance = openvdb::tools::QuadraticSampler::sample(blendedGrid->tree(), vdbPoint);

                //Entry 1
                const Index eId1 = action.entry1;
                assert(eId1 != nvl::NULL_ID);
                const Index cId1 = clusterMap.at(eId1);

                const FloatGrid::ValueType closedDistance1 = openvdb::tools::QuadraticSampler::sample(closedGrids[cId1]->tree(), vdbPoint);
                const IntGrid::ValueType pId1 = polygonAccessors[cId1].getValue(vdbCoord);

                if (pId1 >= 0 && std::fabs(closedDistance1 - blendedDistance) > PRESERVE_GAP_THRESHOLD) {
                    preservedFaces[cId1].erase(fieldBirthFace[cId1][pId1]);

                    if (action.operation == OperationType::ATTACH) {
                        std::queue<std::pair<FaceId, int>> queue;
                        queue.push(std::make_pair(fieldBirthFace[cId1][pId1], 0));
                        while (!queue.empty()) {
                            std::pair<FaceId,int> currentValue = queue.front();
                            queue.pop();

                            if (currentValue.second > PRESERVE_ATTACH_RADIUS) {
                                continue;
                            }

                            for (FaceId adjId : ffAdjs[cId1][currentValue.first]) {
                                preservedFaces[cId1].erase(adjId);

                                queue.push(std::make_pair(adjId, currentValue.second + 1));
                            }
                        }
                    }
                }


                //Entry 2
                const Index eId2 = action.entry2;
                if (eId2 != nvl::NULL_ID) {
                    const Index cId2 = clusterMap.at(eId2);

                    const FloatGrid::ValueType closedDistance2 = openvdb::tools::QuadraticSampler::sample(closedGrids[cId2]->tree(), vdbPoint);
                    const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(vdbCoord);

                    if (pId2 >= 0 && std::fabs(closedDistance2 - blendedDistance) > PRESERVE_GAP_THRESHOLD) {
                        preservedFaces[cId2].erase(fieldBirthFace[cId2][pId2]);

                        if (action.operation == OperationType::ATTACH) {
                            std::queue<std::pair<FaceId, int>> queue;
                            queue.push(std::make_pair(fieldBirthFace[cId2][pId2], 0));
                            while (!queue.empty()) {
                                std::pair<FaceId,int> currentValue = queue.front();
                                queue.pop();

                                if (currentValue.second > PRESERVE_ATTACH_RADIUS) {
                                    continue;
                                }

                                for (FaceId adjId : ffAdjs[cId2][currentValue.first]) {
                                    preservedFaces[cId2].erase(adjId);

                                    queue.push(std::make_pair(adjId, currentValue.second + 1));
                                }
                            }
                        }
                    }
                }
            }
        }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        preMesh = internal::computePreservedMesh(data, cluster, preservedFaces, preBirthVertex, preBirthFace);
        nvl::meshSaveToFile("results/premesh_2_erased_faces.obj", preMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif





#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Regularize preserved mesh... ";
#endif

        //Regularization
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Mesh& mesh = models[cId]->mesh;

            std::vector<std::vector<FaceId>> meshFFAdj = nvl::meshFaceFaceAdjacencies(mesh);

            for (int i = 0; i < PRESERVE_REGULARIZATION_ITERATIONS; ++i) {
                nvl::meshOpenFaceSelection(mesh, preservedFaces[cId], meshFFAdj, true);
                nvl::meshCloseFaceSelection(mesh, preservedFaces[cId], meshFFAdj, true);
            }
        }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        preMesh = internal::computePreservedMesh(data, cluster, preservedFaces, preBirthVertex, preBirthFace);
        nvl::meshSaveToFile("results/premesh_3_regularized.obj", preMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif



#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Extract preserved mesh... ";
#endif

        preMesh = internal::computePreservedMesh(data, cluster, preservedFaces, preBirthVertex, preBirthFace);

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/preMesh.obj", preMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif





        // --------------------------------------- BORDER ATTACHING ---------------------------------------



#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Mark new surface faces... ";
#endif

        //Fill faces to keep in the blended mesh
        std::unordered_set<FaceId> newSurfaceFaces;

        for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
            if (blendedMesh.isFaceDeleted(fId))
                continue;

            bool isNewSurface = true;

            for (VertexId vId : blendedMesh.faceVertexIds(fId)) {
                const Point& point = blendedMesh.vertexPoint(vId);
                const GridVec vdbWorldPoint(point.x(), point.y(), point.z());
                const GridVec vdbPoint = blendedGrid->worldToIndex(vdbWorldPoint);
                const GridCoord vdbCoord(std::round(vdbPoint.x()), std::round(vdbPoint.y()), std::round(vdbPoint.z()));

                IntGrid::ValueType bestActionId = actionAccessor.getValue(vdbCoord);

                assert(bestActionId >= 0);

                const Index& actionId = actions[bestActionId];
                const Action& action = data.action(actionId);


                const FloatGrid::ValueType blendedDistance = openvdb::tools::QuadraticSampler::sample(blendedGrid->tree(), vdbPoint);


                //Entry 1
                const Index eId1 = action.entry1;
                assert(eId1 != nvl::NULL_ID);
                const Index cId1 = clusterMap.at(eId1);

                const FloatGrid::ValueType closedDistance1 = openvdb::tools::QuadraticSampler::sample(closedGrids[cId1]->tree(), vdbPoint);
                const IntGrid::ValueType pId1 = polygonAccessors[cId1].getValue(vdbCoord);

                if (pId1 >= 0 && preservedFaces[cId1].find(fieldBirthFace[cId1][pId1]) != preservedFaces[cId1].end() && std::fabs(blendedDistance - closedDistance1) <= NEWSURFACE_DISTANCE_THRESHOLD) {
                    isNewSurface = false;
                }


                //Entry 2
                const Index eId2 = action.entry2;
                if (eId2 != nvl::NULL_ID) {
                    const Index cId2 = clusterMap.at(eId2);

                    const FloatGrid::ValueType closedDistance2 = openvdb::tools::QuadraticSampler::sample(closedGrids[cId2]->tree(), vdbPoint);
                    const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(vdbCoord);

                    if (pId2 >= 0 && preservedFaces[cId2].find(fieldBirthFace[cId2][pId2]) != preservedFaces[cId2].end() && std::fabs(blendedDistance - closedDistance2) <= NEWSURFACE_DISTANCE_THRESHOLD) {
                        isNewSurface = false;
                    }
                }

            }

            if (isNewSurface) {
                newSurfaceFaces.insert(fId);
            }
        }

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif




#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Attaching borders... ";
#endif

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

        //Attach mesh borders to the preserved mesh
        std::unordered_set<VertexId> newSnappedVertices;
        std::unordered_set<VertexId> preSnappedVertices;
        newMesh = internal::attachMeshesByBorders(blendedMesh, preMesh, preNonSnappableVertices, newSurfaceFaces, newSnappedVertices, preSnappedVertices);

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif







        // --------------------------------------- SMOOTHING ---------------------------------------


#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Mesh border subdivision... ";
#endif


        for (int it = 0; it < NEWMESH_SUBDIVISION_ITERATIONS; ++it) {
            //Select edges to subdivide
            std::vector<std::vector<FaceId>> newFFAdj = nvl::meshFaceFaceAdjacencies(newMesh);

            std::vector<std::pair<VertexId, VertexId>> edgesToSubdivide;
            for (FaceId fId = 0; fId < newMesh.nextFaceId(); ++fId) {
                if (newMesh.isFaceDeleted(fId))
                    continue;

                if (nvl::meshIsBorderFace(newMesh, fId, newFFAdj)) {
                    const std::vector<VertexId>& vertexIds = newMesh.faceVertexIds(fId);
                    for (Index j = 0; j < vertexIds.size(); j++) {

                        if (!nvl::meshIsBorderFaceEdge(newMesh, fId, j, newFFAdj)) {
                            VertexId nextJ = (j + 1) % vertexIds.size();

                            edgesToSubdivide.push_back(std::make_pair(vertexIds[j], vertexIds[nextJ]));
                        }
                    }
                }
            }

            //Split
            std::vector<std::vector<FaceId>> newVFAdj = nvl::meshVertexFaceAdjacencies(newMesh);

            for (const std::pair<VertexId, VertexId>& edge : edgesToSubdivide) {
                Point midPoint =
                        (newMesh.vertexPoint(edge.first) + newMesh.vertexPoint(edge.second)) / 2.0;
                nvl::meshSplitEdge(newMesh, edge.first, edge.second, midPoint, newVFAdj);
            }

            //Find faces to subdivide
            newFFAdj = nvl::meshFaceFaceAdjacencies(newMesh);

            std::vector<FaceId> facesToSubdivide;
            for (FaceId fId = 0; fId < newMesh.nextFaceId(); ++fId) {
                if (newMesh.isFaceDeleted(fId))
                    continue;

                if (nvl::meshIsBorderFace(newMesh, fId, newFFAdj)) {
                    facesToSubdivide.push_back(fId);
                }
            }


            for (const FaceId& fId : facesToSubdivide) {
                nvl::meshSubdivideInBarycenterWithTriangles(newMesh, fId);
            }
        }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/newMesh_4_subdivided.obj", newMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Smoothing... ";
#endif

        //Calculate ff adjancencies
        std::vector<std::vector<FaceId>> newFFAdj = nvl::meshFaceFaceAdjacencies(newMesh);

        //Select vertices to smooth
        for (VertexId vId = 0; vId < newMesh.nextVertexId(); ++vId) {
            if (newMesh.isVertexDeleted(vId))
                continue;

            if (nvl::meshIsBorderVertex(newMesh, vId, newFFAdj))
                continue;

            const Point& point = newMesh.vertexPoint(vId);
            const GridVec vdbWorldPoint(point.x(), point.y(), point.z());
            const GridVec vdbPoint = blendedGrid->worldToIndex(vdbWorldPoint);
            const GridCoord vdbCoord(std::round(vdbPoint.x()), std::round(vdbPoint.y()), std::round(vdbPoint.z()));

            IntGrid::ValueType bestActionId = actionAccessor.getValue(vdbCoord);

            assert(bestActionId >= 0);

            const Index& actionId = actions[bestActionId];
            const Action& action = data.action(actionId);

            double maxSelectValue = 0.0;

            //Entry 1
            const Index eId1 = action.entry1;
            assert(eId1 != nvl::NULL_ID);
            const Index cId1 = clusterMap.at(eId1);

            const IntGrid::ValueType pId1 = polygonAccessors[cId1].getValue(vdbCoord);

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
            if (eId2 != nvl::NULL_ID) {
                const Index cId2 = clusterMap.at(eId2);

                const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(vdbCoord);

                const std::vector<double>& vertexSelectValues2 = vertexSelectValues[cId2];
                const std::vector<FaceId>& fieldBirthFace2 = fieldBirthFace[cId2];

                const Mesh& mesh2 = models[cId2]->mesh;

                if (pId2 >= 0) {
                    FaceId originFaceId2 = fieldBirthFace2[pId2];
                    double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, vertexSelectValues2);
                    maxSelectValue = std::max(maxSelectValue, selectValue2);
                }
            }

            if (maxSelectValue >= smoothingBorderThreshold) {
                const double borderSmoothingAlpha = 1.0 - (maxSelectValue - smoothingBorderThreshold) / (1.0 - smoothingBorderThreshold);
                assert(borderSmoothingAlpha >= 0.0 && borderSmoothingAlpha <= 1.0);
                borderVerticesToSmooth.push_back(vId);
                borderVerticesToSmoothAlpha.push_back(borderSmoothingAlpha);
            }

            innerVerticesToSmooth.push_back(vId);
        }

        //Laplacian adaptive
        nvl::meshLaplacianSmoothing(newMesh, borderVerticesToSmooth, smoothingBorderIterations, borderVerticesToSmoothAlpha);

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/newMesh_5_smoothing.obj", newMesh);
#endif

        //Total laplacian
        nvl::meshLaplacianSmoothing(newMesh, innerVerticesToSmooth, smoothingInnerIterations, smoothingInnerAlpha);

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/newMesh_6_laplacian.obj", newMesh);
#endif



#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif



#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Remeshing... ";
#endif

        //Remesh
        newMesh = nvl::isotropicRemeshing(newMesh, blendedEdgeSize * NEWMESH_REMESHING_FACTOR, NEWMESH_REMESHING_ITERATIONS, true);

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/newMesh.obj", newMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif


        // --------------------------------------- QUADRANGULATION ---------------------------------------

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Computing quadrangulation... ";
#endif

        //Get final mesh
        Mesh quadrangulation;
        resultMesh = internal::quadrangulateMesh(newMesh, preMesh, blendedMesh, quadrangulation, preBirthVertex, preBirthFace, resultPreBirthVertex, resultPreBirthFace, smoothingResultIterations);



#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/resultMesh.obj", resultMesh);
#endif

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/quadrangulationMesh.obj", quadrangulation);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif
    }
    else {
        assert(mixMode == MixMode::MORPHING);


        // --------------------------------------- VARIABLE DEFINITION ---------------------------------------

        std::vector<std::unordered_set<FaceId>> preservedFaces(cluster.size()); //Preserved faces
        std::vector<std::unordered_set<FaceId>> morphingFaces(cluster.size()); //Faces to be morphed
        std::vector<std::unordered_set<VertexId>> preservedVertices(cluster.size()); //Preserved vertices
        std::vector<std::unordered_set<VertexId>> morphingVertices(cluster.size()); //Vertices to be morphed



        // --------------------------------------- GET BLENDED MESH ---------------------------------------

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Extract isosurface... ";
#endif

        //Extract iso surface mesh
        blendedMesh = internal::convertGridToMesh<Mesh>(blendedGrid, true);

    #ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/blendedMesh_1_non_remeshed.obj", blendedMesh);
    #endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif





#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Remesh blended mesh... ";
#endif

        //Remesh
        double edgeSize = nvl::meshAverageEdgeLength(blendedMesh);
        blendedMesh = nvl::isotropicRemeshing(blendedMesh, edgeSize * NEWSURFACE_REMESHING_FACTOR, NEWSURFACE_REMESHING_ITERATIONS, true);

    #ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshComputeFaceNormalsSVDFitting(blendedMesh);
        nvl::meshComputeVertexNormalsFromFaceNormals(blendedMesh);
        nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);
    #endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif





        // --------------------------------------- PRESERVED FACES ---------------------------------------

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Get initial preserved faces... ";
#endif


        //Get preserved faces by select values
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Mesh& mesh = models[cId]->mesh;

            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValues[cId]);

                if (selectValue >= PRESERVE_SELECT_VALUE) {
                    preservedFaces[cId].insert(fId);
                }
            }
        }


#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif





#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Regularize preserved mesh... ";
#endif
        //Regularization
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Mesh& mesh = models[cId]->mesh;

            for (int i = 0; i < PRESERVE_REGULARIZATION_ITERATIONS; ++i) {
                nvl::meshOpenFaceSelection(mesh, preservedFaces[cId], true);
                nvl::meshCloseFaceSelection(mesh, preservedFaces[cId], true);
            }
        }

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif





        // --------------------------------------- GET RESULT MESH ---------------------------------------


#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Get result mesh... ";
#endif

        //Get morphing faces by select values
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Mesh& mesh = models[cId]->mesh;

            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId) && preservedFaces[cId].find(fId) != preservedFaces[cId].end()) {
                    continue;
                }

                double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValues[cId]);

                if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0) &&
                    selectValue < 1.0 && !nvl::epsEqual(selectValue, 1.0))
                {
                    morphingFaces[cId].insert(fId);
                }
            }
        }

        //Find morphing vertices
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Mesh& mesh = models[cId]->mesh;
            for (const FaceId& fId : preservedFaces[cId]) {
                const std::vector<VertexId>& vIds = mesh.faceVertexIds(fId);
                preservedVertices[cId].insert(vIds.begin(), vIds.end());
            }

            for (const FaceId& fId : morphingFaces[cId]) {
                for (const VertexId& vId : mesh.faceVertexIds(fId)) {
                    if (preservedVertices[cId].find(vId) == preservedVertices[cId].end()) {
                        morphingVertices[cId].insert(vId);
                    }
                }
            }
        }


        std::vector<std::pair<Index, VertexId>> resultMorphingBirthVertex; //Birth vertex infos

        //Create result mesh
        for (Index cId = 0; cId < cluster.size(); ++cId) {
            const Index& eId = cluster[cId];
            const Entry& entry = data.entry(eId);

            const Model* model = entry.model;
            const Mesh& mesh = model->mesh;

            std::vector<VertexId> verticesToTransfer;
            verticesToTransfer.insert(verticesToTransfer.end(), preservedVertices[cId].begin(), preservedVertices[cId].end());
            verticesToTransfer.insert(verticesToTransfer.end(), morphingVertices[cId].begin(), morphingVertices[cId].end());

            nvl::Size lastVertexId = resultMesh.nextVertexId();
            nvl::Size lastFaceId = resultMesh.nextFaceId();

            std::vector<VertexId> tmpBirthVertex;
            std::vector<FaceId> tmpBirthFace;
            nvl::meshTransferVerticesWithFaces(mesh, verticesToTransfer, resultMesh, tmpBirthVertex, tmpBirthFace);

            resultPreBirthVertex.resize(resultMesh.nextVertexId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));
            resultPreBirthFace.resize(resultMesh.nextFaceId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));

            resultMorphingBirthVertex.resize(resultMesh.nextVertexId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));

            for (Index vId = lastVertexId; vId < resultMesh.nextVertexId(); ++vId) {
                assert(tmpBirthVertex[vId] != nvl::NULL_ID);
                VertexId birthVertexId = tmpBirthVertex[vId];

                //If to be morphed, then move to the closest point
                if (morphingVertices[cId].find(birthVertexId) != morphingVertices[cId].end()) {
                    resultMorphingBirthVertex[vId] = std::make_pair(eId, birthVertexId);
                    assert(preservedVertices[cId].find(birthVertexId) == preservedVertices[cId].end());
                }
                //Vertex is preserved
                else if (preservedVertices[cId].find(birthVertexId) != preservedVertices[cId].end()) {
                    resultPreBirthVertex[vId] = std::make_pair(eId, birthVertexId);
                }
            }
            for (Index fId = lastFaceId; fId < resultMesh.nextFaceId(); ++fId) {
                assert(tmpBirthFace[fId] != nvl::NULL_ID);
                resultPreBirthFace[fId] = std::make_pair(eId, tmpBirthFace[fId]);
            }
        }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
        nvl::meshSaveToFile("results/morphing_0_initial.obj", resultMesh);
#endif



#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif




        // --------------------------------------- MORPHING ---------------------------------------


#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Morphing... ";
#endif


        //Get field for gradient
        const double maxVoxelDistance = maxDistance / scaleFactor + nvl::EPSILON;

        internal::OpenVDBAdapter<Mesh> gradientAdapter(blendedMesh, scaleFactor);

        GridTransformPtr linearTransform = GridTransform::createLinearTransform(scaleFactor);
        FloatGridPtr gradientGrid = openvdb::tools::meshToVolume<FloatGrid>(
            gradientAdapter, *linearTransform, maxVoxelDistance, maxVoxelDistance, 0);

        FloatGrid::ConstAccessor gradientAccessor(gradientGrid->getConstAccessor());


        const std::vector<std::vector<VertexId>> resultVVAdj = nvl::meshVertexVertexAdjacencies(resultMesh);

        const int morphingIterations = 10;
        const double gradientWeight = 0.5;
        const double dcWeight = 0.5;
        for (int it = 0; it < morphingIterations; ++it) {
            //Differential coordinates
            std::vector<Vector> resultDC = nvl::meshDifferentialCoordinates(resultMesh, resultVVAdj);

            for (Index vId = 0; vId < resultMesh.nextVertexId(); ++vId) {
                if (resultMesh.isVertexDeleted(vId))
                    continue;

                const std::pair<Index,VertexId>& pair = resultMorphingBirthVertex[vId];

                const Index& birthEId = pair.first;
                const VertexId& birthVertexId = pair.second;

                NVL_SUPPRESS_UNUSEDVARIABLE(birthVertexId);

                if (birthEId != nvl::NULL_ID) {
                    assert(birthVertexId != nvl::NULL_ID);

                    const Point& point = resultMesh.vertexPoint(vId);
                    const GridVec vdbWorldPoint(point.x(), point.y(), point.z());
                    const GridVec vdbPoint = blendedGrid->worldToIndex(vdbWorldPoint);
                    const GridCoord vdbCoord(std::round(vdbPoint.x()), std::round(vdbPoint.y()), std::round(vdbPoint.z()));

                    const FloatGrid::ValueType blendedDistance = openvdb::tools::QuadraticSampler::sample(blendedGrid->tree(), vdbPoint);

                    openvdb::math::Vec3d vdbGradient = openvdb::math::ISGradient<openvdb::math::CD_2ND>::result(gradientAccessor, vdbCoord);

                    Vector gradient(vdbGradient.x(), vdbGradient.y(), vdbGradient.z());
                    gradient.normalize();

                    gradient = -gradient * blendedDistance * gradientWeight;

                    Point newPoint = point + gradient;
                    resultMesh.setVertexPoint(vId, newPoint);
                }
            }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
            nvl::meshSaveToFile("results/morphing_" + std::to_string(it + 1) + ".obj", resultMesh);
#endif

            for (Index vId = 0; vId < resultMesh.nextVertexId(); ++vId) {
                if (resultMesh.isVertexDeleted(vId))
                    continue;

                const std::pair<Index,VertexId>& pair = resultMorphingBirthVertex[vId];

                const Index& birthEId = pair.first;
                const VertexId& birthVertexId = pair.second;

                NVL_SUPPRESS_UNUSEDVARIABLE(birthVertexId);

                if (birthEId != nvl::NULL_ID) {
                    assert(birthVertexId != nvl::NULL_ID);

                    const Point& point = resultMesh.vertexPoint(vId);

                    //Calculate delta
                    Point delta = Point::Zero();
                    const std::vector<VertexId>& neighbors = resultVVAdj[vId];
                    for(const VertexId& neighborId : neighbors) {
                        const Point& neighborPoint = resultMesh.vertexPoint(neighborId);
                        delta += neighborPoint;
                    }
                    delta /= neighbors.size();

                    //Calculate point with differential coordinates
                    Point dcPoint = resultDC[vId] + delta;

                    //Calculate new point
                    Point newPoint = (1 - dcWeight) * point + dcWeight * dcPoint;

                    resultMesh.setVertexPoint(vId, newPoint);
                }
            }

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
            nvl::meshSaveToFile("results/morphing_" + std::to_string(it + 1) + "_dc.obj", resultMesh);
#endif
        }

        gradientGrid->clear();
        gradientGrid.reset();

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif




        // --------------------------------------- REPROJECT AND INFLATE/DEFLATE ---------------------------------------


#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Reproject and inflate/deflate... ";
#endif

        nvl::meshComputeFaceNormalsSVDFitting(blendedMesh);
        nvl::meshComputeVertexNormalsFromFaceNormals(blendedMesh);
        nvl::VCGGrid<Mesh> vcgGrid(blendedMesh); //Create vcg grid for closest point detection
        for (Index vId = 0; vId < resultMesh.nextVertexId(); ++vId) {
            if (resultMesh.isVertexDeleted(vId))
                continue;

            const std::pair<Index,VertexId>& pair = resultMorphingBirthVertex[vId];

            const Index& birthEId = pair.first;
            const VertexId& birthVertexId = pair.second;

            NVL_SUPPRESS_UNUSEDVARIABLE(birthVertexId);

            if (birthEId != nvl::NULL_ID) {
                const Index& birthCId = clusterMap.at(birthEId);
                assert(birthVertexId != nvl::NULL_ID);

                Index bestCId = nvl::maxLimitValue<Index>();

                const Point& point = resultMesh.vertexPoint(vId);
                const GridVec vdbWorldPoint(point.x(), point.y(), point.z());
                const GridVec vdbPoint = blendedGrid->worldToIndex(vdbWorldPoint);
                const GridCoord vdbCoord(std::round(vdbPoint.x()), std::round(vdbPoint.y()), std::round(vdbPoint.z()));

                IntGrid::ValueType bestActionId = actionAccessor.getValue(vdbCoord);

                bool inflate = false;

                if (bestActionId >= 0) {
                    const Index& actionId = actions[bestActionId];
                    const Action& action = data.action(actionId);

                    const Index eId1 = action.entry1;
                    assert(eId1 != nvl::NULL_ID);
                    const Index cId1 = clusterMap.at(eId1);

                    const Mesh& mesh1 = models[cId1]->mesh;

                    const std::vector<double>& vertexSelectValues1 = vertexSelectValues[cId1];
                    const std::vector<FaceId>& fieldBirthFace1 = fieldBirthFace[cId1];

                    const IntGrid::ValueType pId1 = polygonAccessors[cId1].getValue(vdbCoord);

                    FaceId originFaceId1 = fieldBirthFace1[pId1];
                    double selectValue1 = internal::interpolateFaceSelectValue(mesh1, originFaceId1, point, vertexSelectValues1);

                    if (pId1 >= 0) {
                        bestCId = cId1;
                    }

                    if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
                        const Index eId2 = action.entry2;
                        assert(eId2 != nvl::NULL_ID);
                        const Index cId2 = clusterMap.at(eId2);

                        const Mesh& mesh2 = models[cId2]->mesh;

                        const std::vector<double>& vertexSelectValues2 = vertexSelectValues[cId2];
                        const std::vector<FaceId>& fieldBirthFace2 = fieldBirthFace[cId2];

                        const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(vdbCoord);

                        if (pId1 >= 0 && pId2 >= 0) {
                            if (action.operation == OperationType::REPLACE) {
                                FaceId originFaceId2 = fieldBirthFace2[pId2];
                                double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, vertexSelectValues2);

                                if (selectValue2 > selectValue1) {
                                    bestCId = cId2;
                                }
                            }
                        }
                    }

                    if (bestCId != nvl::maxLimitValue<Index>()) {
                        inflate = bestCId == birthCId;
                    }
                }

                Point closestPoint;
                FaceId closestFaceId = vcgGrid.getClosestFace(point, closestPoint);

                FaceNormal normal = blendedMesh.faceNormal(closestFaceId);
                normal.normalize();

                const double inflateStep = voxelSize / 10.0;

                Point newPoint = closestPoint;
                if (!inflate) {
                    normal = -normal;
                }

                newPoint = closestPoint + inflateStep * normal;

                resultMesh.setVertexPoint(vId, newPoint);
            }
        }


#ifdef SKINMIXER_DEBUG_SAVE_MESHES
    nvl::meshSaveToFile("results/resultMesh.obj", resultMesh);
#endif

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif
    }



    // --------------------------------------- BIRTH INFOS ---------------------------------------

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Get birth info... ";
#endif

    //Compute and fill the birth infos
    resultEntry.birth.entries = cluster;
    resultEntry.birth.vertex.resize(resultMesh.nextVertexId());
    for (VertexId vId = 0; vId < resultMesh.nextVertexId(); ++vId) {
        if (resultMesh.isVertexDeleted(vId))
            continue;

        std::vector<VertexInfo>& vertexInfo = resultEntry.birth.vertex[vId];

        Index birthEId = resultPreBirthVertex[vId].first;
        Index birthVId = resultPreBirthVertex[vId].second;

        if (birthEId != nvl::NULL_ID) {
            VertexInfo info;
            info.eId = birthEId;
            info.vId = birthVId;
            info.weight = 1.0;
            info.closestFaceId = nvl::NULL_ID;
            info.distance = 0.0;
            vertexInfo.push_back(info);
        }
        else {
            const Point& point = resultMesh.vertexPoint(vId);
            const GridVec vdbWorldPoint(point.x(), point.y(), point.z());
            const GridVec vdbPoint = blendedGrid->worldToIndex(vdbWorldPoint);
            const GridCoord vdbCoord(std::round(vdbPoint.x()), std::round(vdbPoint.y()), std::round(vdbPoint.z()));

            IntGrid::ValueType bestActionId = actionAccessor.getValue(vdbCoord);

            if (bestActionId >= 0) {
                const Index& actionId = actions[bestActionId];
                const Action& action = data.action(actionId);



                //Entry 1
                const Index eId1 = action.entry1;
                assert(eId1 != nvl::NULL_ID);
                const Index cId1 = clusterMap.at(eId1);

                const FloatGrid::ValueType closedDistance1 = openvdb::tools::QuadraticSampler::sample(closedGrids[cId1]->tree(), vdbPoint);
                const IntGrid::ValueType pId1 = polygonAccessors[cId1].getValue(vdbCoord);

                const std::vector<double>& vertexSelectValues1 = vertexSelectValues[cId1];
                const std::vector<FaceId>& fieldBirthFace1 = fieldBirthFace[cId1];

                const Mesh& mesh1 = models[cId1]->mesh;




                if (action.operation == OperationType::REMOVE || action.operation == OperationType::DETACH) {
                    if (pId1 >= 0) {
                        FaceId originFaceId1 = fieldBirthFace1[pId1];
                        VertexInfo info;
                        info.eId = eId1;
                        info.vId = nvl::NULL_ID;
                        info.weight = 1.0;
                        info.closestFaceId = originFaceId1;
                        info.distance = closedDistance1;
                        vertexInfo.push_back(info);
                    }
                }
                else if (action.operation == OperationType::REPLACE) {
                    //Entry 2
                    const Index eId2 = action.entry2;
                    assert(eId2 != nvl::NULL_ID);
                    const Index cId2 = clusterMap.at(eId2);

                    const FloatGrid::ValueType closedDistance2 = openvdb::tools::QuadraticSampler::sample(closedGrids[cId2]->tree(), vdbPoint);
                    const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(vdbCoord);

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
                            info1.vId = nvl::NULL_ID;
                            info1.weight = 1.0;
                            info1.closestFaceId = originFaceId1;
                            info1.distance = closedDistance1;
                            vertexInfo.push_back(info1);
                        }
                        else if (selectValue1 < SELECT_VALUE_MAX_THRESHOLD && selectValue2 >= SELECT_VALUE_MAX_THRESHOLD) {
                            VertexInfo info2;
                            info2.eId = eId2;
                            info2.vId = nvl::NULL_ID;
                            info2.weight = 1.0;
                            info2.closestFaceId = originFaceId2;
                            info2.distance = closedDistance2;
                            vertexInfo.push_back(info2);
                        }
                        else if (selectValue1 <= SELECT_VALUE_MIN_THRESHOLD && selectValue2 <= SELECT_VALUE_MIN_THRESHOLD) {
                            VertexInfo info1;
                            info1.eId = eId1;
                            info1.vId = nvl::NULL_ID;
                            info1.weight = 0.5;
                            info1.closestFaceId = originFaceId1;
                            info1.distance = closedDistance1;
                            vertexInfo.push_back(info1);

                            VertexInfo info2;
                            info2.eId = eId2;
                            info2.vId = nvl::NULL_ID;
                            info2.weight = 0.5;
                            info2.closestFaceId = originFaceId2;
                            info2.distance = closedDistance2;
                            vertexInfo.push_back(info2);
                        }
                        else {
                            VertexInfo info1;
                            info1.eId = eId1;
                            info1.vId = nvl::NULL_ID;
                            info1.weight = selectValue1 / (selectValue1 + selectValue2);
                            info1.closestFaceId = originFaceId1;
                            info1.distance = closedDistance1;
                            vertexInfo.push_back(info1);

                            VertexInfo info2;
                            info2.eId = eId2;
                            info2.vId = nvl::NULL_ID;
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
                        info1.vId = nvl::NULL_ID;
                        info1.weight = 1.0;
                        info1.closestFaceId = originFaceId1;
                        info1.distance = closedDistance1;
                        vertexInfo.push_back(info1);
                    }
                    else if (pId2 >= 0 && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        FaceId originFaceId2 = fieldBirthFace2[pId2];
                        VertexInfo info2;
                        info2.eId = eId2;
                        info2.vId = nvl::NULL_ID;
                        info2.weight = 1.0;
                        info2.closestFaceId = originFaceId2;
                        info2.distance = closedDistance2;
                        vertexInfo.push_back(info2);
                    }
                }
                else if (action.operation == OperationType::ATTACH) {
                    //Entry 2
                    const Index eId2 = action.entry2;
                    assert(eId2 != nvl::NULL_ID);
                    const Index cId2 = clusterMap.at(eId2);

                    const FloatGrid::ValueType closedDistance2 = openvdb::tools::QuadraticSampler::sample(closedGrids[cId2]->tree(), vdbPoint);
                    const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(vdbCoord);

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
                            info1.vId = nvl::NULL_ID;
                            info1.weight = 1.0;
                            info1.closestFaceId = originFaceId1;
                            info1.distance = closedDistance1;
                            vertexInfo.push_back(info1);
                        }
                        else {
                            VertexInfo info2;
                            info2.eId = eId2;
                            info2.vId = nvl::NULL_ID;
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
                        info1.vId = nvl::NULL_ID;
                        info1.weight = 1.0;
                        info1.closestFaceId = originFaceId1;
                        info1.distance = closedDistance1;
                        vertexInfo.push_back(info1);
                    }
                    else if (pId2 >= 0 && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        FaceId originFaceId2 = fieldBirthFace2[pId2];
                        VertexInfo info2;
                        info2.eId = eId2;
                        info2.vId = nvl::NULL_ID;
                        info2.weight = 1.0;
                        info2.closestFaceId = originFaceId2;
                        info2.distance = closedDistance2;
                        vertexInfo.push_back(info2);
                    }
                }
            }
        }
    }

#ifdef STEP_TIMES
    duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
    std::cout << duration << " ms" << std::endl;
    totalDuration += duration;
#endif


    // --------------------------------------- UV TO VERTEX COLOR ---------------------------------------

    if (blendColorFromTextures) {

#ifdef STEP_TIMES
        start = chrono::steady_clock::now();
        std::cout << "Blending vertex colors from textures...";
#endif

        resultMesh.enableVertexColors();

        std::vector<std::vector<std::vector<FaceId>>> vfAdjs(models.size());
        std::vector<std::vector<QImage>> imgs(models.size());

        std::unordered_set<VertexId> colorVertices;
        std::unordered_map<VertexId, FaceId> vertexFaceMap;
        for (FaceId fId = 0; fId < resultMesh.nextFaceId(); ++fId) {
            if (resultMesh.isFaceDeleted(fId))
                continue;

            Index birthFId = resultPreBirthFace[fId].second;
            if (birthFId == nvl::NULL_ID) {
                for (const VertexId& vId : resultMesh.faceVertexIds(fId)) {
                    colorVertices.insert(vId);
                    vertexFaceMap[vId] = fId;
                }
            }
        }

        for (Index cId = 0; cId < models.size(); ++cId) {
            const Mesh& mesh = models[cId]->mesh;

            vfAdjs[cId] = nvl::meshVertexFaceAdjacencies(mesh);

            imgs[cId].resize(mesh.nextMaterialId(), QImage());

            for (MaterialId mId = 0; mId < mesh.nextMaterialId(); ++mId) {
                if (mesh.isMaterialDeleted(mId))
                    continue;

                const Material& mat = mesh.material(mId);

                QImage img;
                bool imageLoaded = img.load(mat.diffuseMap().c_str());
                if (imageLoaded) {
                    imgs[cId][mId] = img;
                }
            }
        }

        std::vector<double> vertexColorWeights(resultMesh.nextVertexId(), 1.0);
        for (const VertexId& vId : colorVertices) {
            if (resultMesh.isVertexDeleted(vId))
                continue;

            const std::vector<VertexInfo>& vertexInfo = resultEntry.birth.vertex[vId];

            std::vector<nvl::Color> colors;
            std::vector<double> colorWeights;

            for (Index i = 0; i < vertexInfo.size(); ++i) {
                const VertexInfo& info = vertexInfo[i];

                assert(info.eId != nvl::NULL_ID);
                const Index cId = clusterMap.at(info.eId);
                const Mesh& mesh = models[cId]->mesh;
                FaceId fId = nvl::NULL_ID;
                Index birthVId = resultPreBirthVertex[vId].second;
                if (birthVId == nvl::NULL_ID) {
                    fId = info.closestFaceId;
                    vertexColorWeights[vId] = VERTEX_COLOR_SMOOTHING_ALPHA;
                }
                else {
                    const std::vector<FaceId>& faces = vfAdjs[cId][birthVId];
                    for (const FaceId& adjFId : faces) {
                        if (!mesh.faceMaterialIsNull(adjFId)) {
                            fId = adjFId;
                            break;
                        }
                    }
                }


                if (fId == nvl::NULL_ID || !mesh.hasFaceMaterials() || mesh.faceMaterialIsNull(fId) || (!mesh.hasVertexUVs() && !mesh.hasWedgeUVs()))
                    continue;

                const MaterialId& mId = mesh.faceMaterial(fId);

                if (mId == nvl::NULL_ID)
                    continue;

                std::vector<nvl::Color> faceTexColors(mesh.faceVertexNumber(fId));

                for (Index j = 0; j < mesh.faceVertexNumber(fId); ++j) {
                    const VertexId& vId = mesh.faceVertexId(fId, j);

                    VertexUV uv = VertexUV::Zero();

                    if (mesh.hasWedgeUVs() && !mesh.faceWedgeUVsAreNull(fId)) {
                        const std::vector<WedgeUVId>& wedgeUVIds = mesh.faceWedgeUVs(fId);
                        if (wedgeUVIds.size() > j) {
                            const WedgeUVId& wedgeUVId = wedgeUVIds[j];

                            if (wedgeUVId != nvl::NULL_ID) {
                                uv = mesh.wedgeUV(wedgeUVId);
                            }
                            else if (mesh.hasVertexUVs()) {
                                uv = mesh.vertexUV(vId);
                            }
                        }
                    }
                    else if (mesh.hasVertexUVs()) {
                        uv = mesh.vertexUV(vId);
                    }

                    const QImage& img = imgs[cId][mId];
                    QColor qColor(img.pixel(nvl::round(nvl::fmod(uv.x(), 1) * img.width()), nvl::round(nvl::fmod(1 - uv.y(), 1) * img.height())));
                    faceTexColors[j] = nvl::Color(qColor);
                }

                if (!faceTexColors.empty()) {
                    colors.push_back(internal::interpolateFaceColors(mesh, fId, resultMesh.vertexPoint(vId), faceTexColors));
                    colorWeights.push_back(info.weight);
                }
            }

            if (!colors.empty()) {
                nvl::normalize(colorWeights);
                resultMesh.setVertexColor(vId, nvl::interpolateColor(colors, colorWeights));
            }
        }

        //Smooth colors
        if (VERTEX_COLOR_SMOOTHING_ITERATIONS > 0) {
            std::vector<std::vector<VertexId>> resultVVAdj = nvl::meshVertexVertexAdjacencies(resultMesh);

            std::vector<float> redValues(resultMesh.nextVertexId());
            std::vector<float> greenValues(resultMesh.nextVertexId());
            std::vector<float> blueValues(resultMesh.nextVertexId());
            std::vector<float> alphaValues(resultMesh.nextVertexId());

            for (const VertexId& vId : colorVertices) {
                const nvl::Color& color = resultMesh.vertexColor(vId);

                redValues[vId] = color.redF();
                greenValues[vId] = color.greenF();
                blueValues[vId] = color.blueF();
                alphaValues[vId] = color.alphaF();
            }

            nvl::laplacianSmoothing(redValues, resultVVAdj, VERTEX_COLOR_SMOOTHING_ITERATIONS, vertexColorWeights);
            nvl::laplacianSmoothing(greenValues, resultVVAdj, VERTEX_COLOR_SMOOTHING_ITERATIONS, vertexColorWeights);
            nvl::laplacianSmoothing(blueValues, resultVVAdj, VERTEX_COLOR_SMOOTHING_ITERATIONS, vertexColorWeights);
            nvl::laplacianSmoothing(alphaValues, resultVVAdj, VERTEX_COLOR_SMOOTHING_ITERATIONS, vertexColorWeights);

            for (const VertexId& vId : colorVertices) {
                nvl::Color color;

                if (nvl::epsEqual(redValues[vId], 0.0)) {
                    redValues[vId] = 0.0;
                }
                else if (nvl::epsEqual(redValues[vId], 1.0)) {
                    redValues[vId] = 1.0;
                }

                if (nvl::epsEqual(greenValues[vId], 0.0)) {
                    greenValues[vId] = 0.0;
                }
                else if (nvl::epsEqual(greenValues[vId], 1.0)) {
                    greenValues[vId] = 1.0;
                }

                if (nvl::epsEqual(blueValues[vId], 0.0)) {
                    blueValues[vId] = 0.0;
                }
                else if (nvl::epsEqual(blueValues[vId], 1.0)) {
                    blueValues[vId] = 1.0;
                }

                if (nvl::epsEqual(alphaValues[vId], 0.0)) {
                    alphaValues[vId] = 0.0;
                }
                else if (nvl::epsEqual(alphaValues[vId], 1.0)) {
                    alphaValues[vId] = 1.0;
                }


                color.setRedF(redValues[vId]);
                color.setGreenF(greenValues[vId]);
                color.setBlueF(blueValues[vId]);
                color.setAlphaF(alphaValues[vId]);

                resultMesh.setVertexColor(vId, color);
            }
        }
    }



#ifdef STEP_TIMES
    duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
    std::cout << duration << " ms" << std::endl;
    totalDuration += duration;
#endif

    // --------------------------------------- CLEAN GRIDS ---------------------------------------

#ifdef STEP_TIMES
    start = chrono::steady_clock::now();
    std::cout << "Clean grids... ";
#endif


    actionGrid->clear();
    actionGrid.reset();

    blendedGrid->clear();
    blendedGrid.reset();

    for (Index cId = 0; cId < cluster.size(); ++cId) {
        polygonGrids[cId]->clear();
        polygonGrids[cId].reset();

        closedGrids[cId]->clear();
        closedGrids[cId].reset();
    }

#ifdef STEP_TIMES
        duration = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count();
        std::cout << duration << " ms" << std::endl;
        totalDuration += duration;
#endif

#ifdef GLOBAL_TIMES
    std::cout << "-> Total duration: " << totalDuration << " ms" << std::endl;
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

    Mesh preMesh;
    preBirthFace.clear();
    preBirthVertex.clear();

    //Create preserved
    for (const Index eId : cluster) {
        const Entry& entry = data.entry(eId);

        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        const std::unordered_set<FaceId>& preFacesToKeepSet = preservedFacesPerModel[model];

        nvl::Size lastVertexId = preMesh.nextVertexId();
        nvl::Size lastFaceId = preMesh.nextFaceId();

        std::vector<VertexId> tmpBirthVertex;
        std::vector<FaceId> tmpBirthFace;
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(preFacesToKeepSet.begin(), preFacesToKeepSet.end()), preMesh, tmpBirthVertex, tmpBirthFace);

        preBirthVertex.resize(preMesh.nextVertexId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));
        preBirthFace.resize(preMesh.nextFaceId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));
        for (Index vId = lastVertexId; vId < preMesh.nextVertexId(); ++vId) {
            assert(tmpBirthVertex[vId] != nvl::NULL_ID);
            preBirthVertex[vId] = std::make_pair(eId, tmpBirthVertex[vId]);
        }
        for (Index fId = lastFaceId; fId < preMesh.nextFaceId(); ++fId) {
            assert(tmpBirthFace[fId] != nvl::NULL_ID);
            preBirthFace[fId] = std::make_pair(eId, tmpBirthFace[fId]);
        }
    }

    return preMesh;
}

template<class Model>
typename Model::Mesh computePreservedMesh(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        const std::vector<std::unordered_set<typename Model::Mesh::FaceId>>& preservedFaces,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::VertexId>>& preBirthVertex,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::FaceId>>& preBirthFace)
{
    typedef nvl::Index Index;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename SkinMixerData<Model>::Entry Entry;

    Mesh preMesh;

    //Create preserved
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        const Index& eId = cluster[cId];
        const Entry& entry = data.entry(eId);

        const Model* model = entry.model;
        const Mesh& mesh = model->mesh;

        nvl::Size lastVertexId = preMesh.nextVertexId();
        nvl::Size lastFaceId = preMesh.nextFaceId();

        std::vector<VertexId> tmpBirthVertex;
        std::vector<FaceId> tmpBirthFace;
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(preservedFaces[cId].begin(), preservedFaces[cId].end()), preMesh, tmpBirthVertex, tmpBirthFace);

        preBirthVertex.resize(preMesh.nextVertexId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));
        preBirthFace.resize(preMesh.nextFaceId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));
        for (Index vId = lastVertexId; vId < preMesh.nextVertexId(); ++vId) {
            assert(tmpBirthVertex[vId] != nvl::NULL_ID);
            preBirthVertex[vId] = std::make_pair(eId, tmpBirthVertex[vId]);
        }
        for (Index fId = lastFaceId; fId < preMesh.nextFaceId(); ++fId) {
            assert(tmpBirthFace[fId] != nvl::NULL_ID);
            preBirthFace[fId] = std::make_pair(eId, tmpBirthFace[fId]);
        }
    }

    return preMesh;
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
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& birthFace,
        const unsigned int smoothingResultIterations)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::MaterialId MaterialId;
    typedef typename Mesh::WedgeNormalId WedgeNormalId;
    typedef typename Mesh::WedgeUVId WedgeUVId;
    typedef typename nvl::Index Index;

    Mesh tmpResult;
    Mesh result;

    nvl::VCGTriangleMesh vcgNewMesh;
    nvl::VCGPolygonMesh vcgPreMesh;
    nvl::VCGPolygonMesh vcgQuadrangulation;
    nvl::VCGPolygonMesh vcgResult;
    nvl::VCGPolygonMesh vcgBlendedMesh;

    nvl::convertMeshToVCGMesh(newMesh, vcgNewMesh);
    nvl::convertMeshToVCGMesh(blendedMesh, vcgBlendedMesh);

    std::vector<VertexId> vcgPreBirthVertex;
    std::vector<FaceId> vcgPreBirthFace;
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
    par.callbackGapLimit = { 0.001, 0.005, 0.01, 0.05, 0.10, 0.15, 0.20, 0.30  };
    par.minimumGap = 0.3;
    par.chartSmoothingIterations = 5;
    par.quadrangulationFixedSmoothingIterations = 5;
    par.quadrangulationNonFixedSmoothingIterations = 5;
    par.doubletRemoval = true;
    par.resultSmoothingIterations = 5;
    par.resultSmoothingNRing = 0;
    par.resultSmoothingLaplacianIterations = smoothingResultIterations;
    par.resultSmoothingLaplacianNRing = 0;

    //Get patch decomposition of the new surface
    std::vector<std::vector<size_t>> newSurfacePartitions;
    std::vector<std::vector<size_t>> newSurfaceCorners;
    std::vector<int> newSurfaceLabel = QuadRetopology::patchDecomposition(vcgNewMesh, vcgPreMesh, newSurfacePartitions, newSurfaceCorners, false, 2.0, true, false, true);

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
    std::vector<Index> vcgResultBirthVertex;
    std::vector<Index> vcgResultBirthFace;
    nvl::convertVCGMeshToMesh(vcgResult, tmpResult, vcgResultBirthVertex, vcgResultBirthFace);

    birthVertex.resize(tmpResult.nextVertexId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));
    birthFace.resize(tmpResult.nextFaceId(), std::make_pair(nvl::NULL_ID, nvl::NULL_ID));

    if (preMesh.hasVertexNormals()) {
        result.enableVertexNormals();
    }
    if (preMesh.hasVertexUVs()) {
        result.enableVertexUVs();
    }
    if (preMesh.hasVertexColors()) {
        result.enableVertexColors();
    }

    for (VertexId vId = 0; vId < tmpResult.nextVertexId(); vId++) {
        if (tmpResult.isVertexDeleted(vId))
            continue;

        Index vcgResultVId = vcgResultBirthVertex[vId];
        int vcgPreMeshVId = vcgPreservedVertexMap[vcgResultVId];
        if (vcgPreMeshVId >= 0) {
            const VertexId& preVId = vcgPreBirthVertex[vcgPreMeshVId];
            const Vertex& preVertex = preMesh.vertex(preVId);

            const VertexId newVId = result.addVertex(preVertex);
            result.setVertexPoint(newVId, tmpResult.vertexPoint(vId));

            birthVertex[newVId] = preBirthVertex[preVId];

            if (preMesh.hasVertexNormals()) {
                result.setVertexNormal(newVId, preMesh.vertexNormal(preVId));
            }
            if (preMesh.hasVertexUVs()) {
                result.setVertexUV(newVId, preMesh.vertexUV(preVId));
            }
            if (preMesh.hasVertexColors()) {
                result.setVertexColor(newVId, preMesh.vertexColor(preVId));
            }
        }
        else {
            result.addVertex(tmpResult.vertex(vId));
        }
    }

    std::vector<MaterialId> materialMap;
    std::vector<WedgeNormalId> wedgeNormalMap;
    std::vector<WedgeUVId> wedgeUVMap;

    if (preMesh.hasFaceNormals()) {
        result.enableFaceNormals();
    }

    if (preMesh.hasFaceMaterials()) {
        result.enableFaceMaterials();
        materialMap.resize(preMesh.nextMaterialId(), nvl::NULL_ID);
    }

    if (preMesh.hasWedgeNormals()) {
        result.enableWedgeNormals();
        wedgeNormalMap.resize(preMesh.nextWedgeNormalId(), nvl::NULL_ID);
    }
    if (preMesh.hasWedgeUVs()) {
        result.enableWedgeUVs();
        wedgeUVMap.resize(preMesh.nextWedgeUVId(), nvl::NULL_ID);
    }

    for (FaceId fId = 0; fId < tmpResult.nextFaceId(); fId++) {
        if (tmpResult.isFaceDeleted(fId))
            continue;

        Index vcgResultFId = vcgResultBirthFace[fId];
        int vcgPreMeshFId = vcgPreservedFaceMap[vcgResultFId];
        if (vcgPreMeshFId >= 0) {
            const FaceId& preFId = vcgPreBirthFace[vcgPreMeshFId];
            assert(preFId != nvl::NULL_ID);

            const Face& preFace = preMesh.face(preFId);

            const std::vector<VertexId>& vertexIds = tmpResult.faceVertexIds(fId);
            FaceId newFId = result.addFace(preFace);
            result.setFaceVertexIds(newFId, vertexIds);

            birthFace[newFId] = preBirthFace[preFId];

            if (preMesh.hasFaceNormals()) {
                result.setFaceNormal(newFId, preMesh.faceNormal(preFId));
            }

            if (preMesh.hasFaceMaterials() && !preMesh.faceMaterialIsNull(preFId)) {
                const MaterialId& preMId = preMesh.faceMaterial(preFId);
                assert(preMId != nvl::NULL_ID);

                assert(preMId < preMesh.nextMaterialId());

                if (materialMap[preMId] == nvl::NULL_ID) {
                    MaterialId newMId = result.addMaterial(preMesh.material(preMId));
                    materialMap[preMId] = newMId;
                }

                assert(materialMap[preMId] < result.nextMaterialId());

                result.setFaceMaterial(newFId, materialMap[preMId]);
            }

            if (preMesh.hasWedgeNormals() && !preMesh.faceWedgeNormalsAreNull(preFId)) {
                const std::vector<WedgeNormalId>& wedgeNormalIds = preMesh.faceWedgeNormals(preFId);

                std::vector<WedgeNormalId> newWedgeNormalIds(preFace.vertexNumber(), nvl::NULL_ID);
                for (Index j = 0; j < wedgeNormalIds.size(); j++) {
                    const WedgeNormalId& wedgeNormalId = wedgeNormalIds[j];

                    if (wedgeNormalId != nvl::NULL_ID) {
                        if (wedgeNormalMap[wedgeNormalId] == nvl::NULL_ID) {
                            WedgeNormalId newWedgeNormalId = result.addWedgeNormal(preMesh.wedgeNormal(wedgeNormalId));
                            wedgeNormalMap[wedgeNormalId] = newWedgeNormalId;
                        }
                        newWedgeNormalIds[j] = wedgeNormalMap[wedgeNormalId];
                    }
                }

                result.setFaceWedgeNormals(newFId, newWedgeNormalIds);
            }

            if (preMesh.hasWedgeUVs() && !preMesh.faceWedgeUVsAreNull(preFId)) {
                const std::vector<WedgeUVId>& wedgeUVIds = preMesh.faceWedgeUVs(preFId);

                std::vector<WedgeUVId> newWedgeUVIds(preFace.vertexNumber(), nvl::NULL_ID);
                for (Index j = 0; j < wedgeUVIds.size(); j++) {
                    const WedgeUVId& wedgeUVId = wedgeUVIds[j];

                    if (wedgeUVId != nvl::NULL_ID) {
                        if (wedgeUVMap[wedgeUVId] == nvl::NULL_ID) {
                            WedgeUVId newWedgeUVId = result.addWedgeUV(preMesh.wedgeUV(wedgeUVId));
                            wedgeUVMap[wedgeUVId] = newWedgeUVId;
                        }
                        newWedgeUVIds[j] = wedgeUVMap[wedgeUVId];
                    }
                }

                result.setFaceWedgeUVs(newFId, newWedgeUVIds);
            }
        }
        else {
            result.addFace(tmpResult.face(fId));
        }
    }

    return result;
}

template<class Mesh>
nvl::Color interpolateFaceColors(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<nvl::Color>& colors)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Point Point;

    nvl::Color value;

    //Get polygon points and select values
    const Face& face = mesh.face(faceId);

    std::vector<Point> polygon(face.vertexNumber());
    std::vector<float> redValues(face.vertexNumber());
    std::vector<float> greenValues(face.vertexNumber());
    std::vector<float> blueValues(face.vertexNumber());
    std::vector<float> alphaValues(face.vertexNumber());
    for (VertexId j = 0; j < face.vertexNumber(); ++j) {
        const VertexId& vId = face.vertexId(j);

        polygon[j] = mesh.vertexPoint(vId);
        redValues[j] = colors[j].redF();
        greenValues[j] = colors[j].greenF();
        blueValues[j] = colors[j].blueF();
        alphaValues[j] = colors[j].alphaF();
    }

    //Interpolation on polygon using barycenter subdivision
    float redValue = nvl::barycentricInterpolationBarycenterSubdivision(
        polygon,
        point,
        redValues,
        true);

    if (nvl::epsEqual(redValue, 0.0)) {
        redValue = 0.0;
    }
    else if (nvl::epsEqual(redValue, 1.0)) {
        redValue = 1.0;
    }

    //Interpolation on polygon using barycenter subdivision
    float greenValue = nvl::barycentricInterpolationBarycenterSubdivision(
        polygon,
        point,
        greenValues,
        true);

    if (nvl::epsEqual(greenValue, 0.0)) {
        greenValue = 0.0;
    }
    else if (nvl::epsEqual(greenValue, 1.0)) {
        greenValue = 1.0;
    }

    //Interpolation on polygon using barycenter subdivision
    float blueValue = nvl::barycentricInterpolationBarycenterSubdivision(
        polygon,
        point,
        blueValues,
        true);

    if (nvl::epsEqual(blueValue, 0.0)) {
        blueValue = 0.0;
    }
    else if (nvl::epsEqual(blueValue, 1.0)) {
        blueValue = 1.0;
    }

    //Interpolation on polygon using barycenter subdivision
    float alphaValue = nvl::barycentricInterpolationBarycenterSubdivision(
        polygon,
        point,
        alphaValues,
        true);

    if (nvl::epsEqual(alphaValue, 0.0)) {
        alphaValue = 0.0;
    }
    else if (nvl::epsEqual(alphaValue, 1.0)) {
        alphaValue = 1.0;
    }

    value.setRedF(redValue);
    value.setGreenF(greenValue);
    value.setBlueF(blueValue);
    value.setAlphaF(alphaValue);

    return value;
}

}

}
