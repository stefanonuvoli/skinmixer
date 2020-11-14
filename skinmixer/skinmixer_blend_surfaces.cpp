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

#define FACE_KEEP_THRESHOLD 0.98
#define SMOOTHING_THRESHOLD 0.94
#define MAX_DISTANCE_BLENDED_MESH 2.0
#define MAX_DISTANCE_SMOOTH_MESH 2.0

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

    Model* model = entry.model;
    Mesh& resultMesh = model->mesh;

    double maxDistance = 100.0;
    double voxelSize = nvl::maxLimitValue<Scalar>();

    std::vector<Model*> models(cluster.size());
    std::vector<std::vector<double>> vertexSelectValue(cluster.size());
    for (Index i = 0; i < cluster.size(); ++i) {
        Index eId = cluster[i];

        models[i] = data.entry(eId).model;

        SelectInfo select = data.computeGlobalSelectInfo(eId);
        vertexSelectValue[i] = select.vertex;
    }

    //Calculate scale transform (minimum of the average length)
    for (Model* modelPtr : models) {
        Mesh& mesh = modelPtr->mesh;
        Scalar avgLength = nvl::meshAverageEdgeLength(mesh);
        voxelSize = std::min(voxelSize, avgLength);
    }

    double scaleFactor = 1.0 / voxelSize;
    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);


    //Fill faces to keep with all faces
    std::vector<std::unordered_set<FaceId>> meshFacesToKeep(models.size());
    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;

        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            meshFacesToKeep[mId].insert(fId);
        }
    }

    //Erase faces to keep for the original meshes
    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;
        //Erase face under the threshold
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValue[mId]);

            if (selectValue < FACE_KEEP_THRESHOLD) {
                meshFacesToKeep[mId].erase(fId);
            }
        }

        nvl::meshOpenFaceSelection(mesh, meshFacesToKeep[mId]);
        nvl::meshCloseFaceSelection(mesh, meshFacesToKeep[mId]);

        //Reinsert deleted face due to open/close
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            const Face& face = mesh.face(fId);

            bool toKeep = true;
            for (Index j = 0; j < face.vertexNumber(); j++) {
                double selectValue = vertexSelectValue[mId][face.vertexId(j)];
                if (!nvl::epsEqual(selectValue, 1.0)) {
                    toKeep = false;
                }
            }
            if (toKeep) {
                meshFacesToKeep[mId].insert(fId);
            }
        }
    }

    //Grid data
    std::vector<internal::FloatGridPtr> unsignedGrids;
    std::vector<internal::IntGridPtr> polygonGrids;
    std::vector<internal::FloatGridPtr> signedGrids;
    std::vector<internal::FloatGridPtr> closedGrids;
    std::vector<openvdb::Vec3i> bbMin;
    std::vector<openvdb::Vec3i> bbMax;

    //Get grids
    internal::getSignedGrids(
        models,
        scaleTransform,
        maxDistance,
        unsignedGrids,
        polygonGrids,
        signedGrids,
        closedGrids,
        bbMin,
        bbMax);

    //Blend meshes
    Mesh blendedMesh = internal::getBlendedMesh(
        models,
        vertexSelectValue,
        scaleTransform,
        maxDistance,
        closedGrids,
        polygonGrids,
        bbMin,
        bbMax);


#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);
#endif

    //Create meshes
    Mesh newMesh;
    Mesh preMesh;

    std::unordered_set<VertexId> preNonSnappableVertices;

    //Create preserved
    std::vector<std::pair<nvl::Index, VertexId>> preBirthVertex;
    std::vector<std::pair<nvl::Index, FaceId>> preBirthFace;
    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;

        //Find vertices that were already in the border of the original mesh
        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(mesh);
        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());

        nvl::Size lastVertexId = preMesh.nextVertexId();
        nvl::Size lastFaceId = preMesh.nextFaceId();

        std::vector<VertexId> tmpBirthVertex;
        std::vector<FaceId> tmpBirthFace;
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(meshFacesToKeep[mId].begin(), meshFacesToKeep[mId].end()), preMesh, tmpBirthVertex, tmpBirthFace);

        preBirthVertex.resize(preMesh.nextVertexId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
        preBirthFace.resize(preMesh.nextFaceId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
        for (Index vId = lastVertexId; vId < preMesh.nextVertexId(); ++vId) {
            assert(tmpBirthVertex[vId] != nvl::MAX_INDEX);
            preBirthVertex[vId] = std::make_pair(mId, tmpBirthVertex[vId]);
        }
        for (Index fId = lastFaceId; fId < preMesh.nextFaceId(); ++fId) {
            assert(tmpBirthFace[fId] != nvl::MAX_INDEX);
            preBirthFace[fId] = std::make_pair(mId, tmpBirthFace[fId]);
        }

        //Non snappable vertices in preserved mesh (border or select value equal to 1)
        for (VertexId vId = lastVertexId; vId < preMesh.nextVertexId(); vId++) {
            if (preMesh.isVertexDeleted(vId))
                continue;

            if (borderVerticesSet.find(tmpBirthVertex[vId]) != borderVerticesSet.end()) {
                preNonSnappableVertices.insert(vId);
            }
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/premesh_1_initial.obj", preMesh);
#endif

    //Fill vertices to keep in the blended mesh
    std::unordered_set<FaceId> blendedMeshFacesToKeep;
    for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
        if (blendedMesh.isFaceDeleted(fId))
            continue;

        bool isNewSurface = true;

        for (VertexId vId : blendedMesh.face(fId).vertexIds()) {
            const Point& point = blendedMesh.vertex(vId).point();

            Point scaledPoint = scaleTransform * point;
            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

            for (Index mId = 0; mId < models.size(); ++mId) {
                internal::FloatGrid::ConstAccessor closedAccessor = closedGrids[mId]->getConstAccessor();
                internal::IntGrid::ConstAccessor polygonAccessor = polygonGrids[mId]->getConstAccessor();

                internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);
                internal::IntGrid::ValueType pId = polygonAccessor.getValue(coord);

                if (pId >= 0 && closedDistance <= MAX_DISTANCE_BLENDED_MESH && closedDistance >= -MAX_DISTANCE_BLENDED_MESH) {
                    if (meshFacesToKeep[mId].find(pId) != meshFacesToKeep[mId].end()) {
                        isNewSurface = false;
                    }
                }
            }
        }

        if (isNewSurface) {
            blendedMeshFacesToKeep.insert(fId);
        }
    }

    //Regularization
    nvl::meshOpenFaceSelection(blendedMesh, blendedMeshFacesToKeep);

    //Transfer in the new surface the faces to be kept
    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(blendedMeshFacesToKeep.begin(), blendedMeshFacesToKeep.end()), newMesh);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh_1_regularized.obj", newMesh);
#endif

    //Attach mesh borders to the preserved mesh
    std::unordered_set<VertexId> newSnappedVertices;
    std::unordered_set<VertexId> preSnappedVertices;
    internal::attachMeshesByBorders(newMesh, preMesh, std::unordered_set<VertexId>(), preNonSnappableVertices, newSnappedVertices, preSnappedVertices);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh.obj", newMesh);
#endif

    internal::cleanPreservedMeshAfterAttaching(preMesh, preNonSnappableVertices, preSnappedVertices, preBirthVertex, preBirthFace);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/premesh.obj", preMesh);
#endif

    //Select vertices to smooth
    std::vector<VertexId> verticesToSmooth;
    std::vector<double> verticesToSmoothAlpha;
    std::vector<std::vector<FaceId>> newMeshFFAdj = nvl::meshFaceFaceAdjacencies(newMesh);
    for (VertexId vId = 0; vId < newMesh.nextVertexId(); ++vId) {
        if (newMesh.isVertexDeleted(vId))
            continue;

        if (nvl::meshIsBorderVertex(newMesh, vId, newMeshFFAdj))
            continue;

        const Point& point = newMesh.vertex(vId).point();

        Point scaledPoint = scaleTransform * point;
        internal::GridCoord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

        bool found = false;
        for (Index mId = 0; mId < models.size() && !found; ++mId) {
            const Mesh& mesh = models[mId]->mesh;

            internal::FloatGrid::ConstAccessor closedAccessor = closedGrids[mId]->getConstAccessor();
            internal::IntGrid::ConstAccessor polygonAccessor = polygonGrids[mId]->getConstAccessor();

            internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);
            internal::IntGrid::ValueType pId = polygonAccessor.getValue(coord);

            if (pId >= 0 && closedDistance <= MAX_DISTANCE_SMOOTH_MESH && closedDistance >= -MAX_DISTANCE_SMOOTH_MESH) {
                double selectValue = internal::interpolateFaceSelectValue(mesh, pId, point, vertexSelectValue[mId]);

                if (selectValue >= SMOOTHING_THRESHOLD) {
                    verticesToSmooth.push_back(vId);

                    const double smoothingAlpha = 0.5 + 0.5 * (selectValue - SMOOTHING_THRESHOLD) / (1.0 - SMOOTHING_THRESHOLD);
                    verticesToSmoothAlpha.push_back(smoothingAlpha);

                    found = true;
                }
            }
        }
    }

    nvl::meshLaplacianSmoothing(newMesh, verticesToSmooth, verticesToSmoothAlpha, 40);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/newmesh_2_smoothed.obj", newMesh);
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
            info.eId = cluster[resultPreBirthVertex[vId].first];
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

            std::vector<std::tuple<nvl::Index, double, FaceId, double>> involvedEntries;
            std::vector<Index> overThresholdValues;

            double selectValueSum = 0.0;
            for (Index mId = 0; mId < models.size(); ++mId) {
                const Mesh& mesh = models[mId]->mesh;

                internal::FloatGrid::ConstAccessor closedAccessor = closedGrids[mId]->getConstAccessor();
                internal::IntGrid::ConstAccessor polygonAccessor = polygonGrids[mId]->getConstAccessor();

                internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);
                internal::IntGrid::ValueType pId = polygonAccessor.getValue(coord);

                if (pId >= 0 && closedDistance < maxDistance && closedDistance > -maxDistance) {
                    double selectValue = internal::interpolateFaceSelectValue(mesh, pId, point, vertexSelectValue[mId]);

                    selectValueSum += selectValue;

                    involvedEntries.push_back(std::make_tuple(mId, selectValue, pId, closedDistance));
                    if (selectValue >= SELECT_VALUE_MAX_THRESHOLD) {
                        overThresholdValues.push_back(involvedEntries.size() - 1);
                    }
                }
            }

            assert(involvedEntries.size() >= 0);

            if (overThresholdValues.size() == 1) {
                const std::tuple<nvl::Index, double, FaceId, double>& tuple = involvedEntries[overThresholdValues[0]];
                VertexInfo info;
                info.eId = cluster[std::get<0>(tuple)];
                info.vId = nvl::MAX_INDEX;
                info.weight = std::get<1>(tuple);
                info.closestFaceId = std::get<2>(tuple);
                info.distance = std::get<3>(tuple);

                vertexInfo.push_back(info);
            }
            else {
                for (Index invId = 0; invId < involvedEntries.size(); ++invId) {
                    const std::tuple<nvl::Index, double, FaceId, double>& tuple = involvedEntries[invId];
                    VertexInfo info;
                    info.eId = cluster[std::get<0>(tuple)];
                    info.vId = nvl::MAX_INDEX;
                    if (selectValueSum >= SELECT_VALUE_MIN_THRESHOLD) {
                        info.weight = std::get<1>(tuple) / selectValueSum;
                    }
                    else {
                        info.weight = 1.0 / involvedEntries.size();
                    }
                    info.closestFaceId = std::get<2>(tuple);
                    info.distance = std::get<3>(tuple);

                    vertexInfo.push_back(info);
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
