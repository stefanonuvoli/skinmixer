#include "skinmixer_blend_surfaces.h"

#ifdef foreach
  #undef foreach
#endif
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

#include <gurobi_c++.h>

#include <nvl/math/closest_point.h>
#include <nvl/math/barycentric_interpolation.h>
#include <nvl/math/numeric_limits.h>
#include <nvl/math/transformations.h>

#include <nvl/models/mesh_geometric_information.h>
#include <nvl/models/mesh_morphological_operations.h>
#include <nvl/models/mesh_transformations.h>
#include <nvl/models/mesh_triangulation.h>
#include <nvl/models/mesh_borders.h>
#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_split.h>
#include <nvl/models/mesh_eigen_convert.h>
#include <nvl/models/mesh_smoothing.h>

#include <nvl/vcglib/vcg_collapse_borders.h>
#include <nvl/vcglib/vcg_convert.h>
#include <nvl/vcglib/vcg_triangle_mesh.h>
#include <nvl/vcglib/vcg_polygon_mesh.h>

#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>

#ifdef SAVE_MESHES_FOR_DEBUG
#include <nvl/models/mesh_io.h>
#endif

#include <quadretopology/quadretopology.h>

#define KEEP_THRESHOLD 0.99
#define DISCARD_THRESHOLD 0.01
#define SMOOTHING_THRESHOLD 0.94
#define MAX_DISTANCE_FOR_NEW_SURFACE 2.0

namespace skinmixer {

namespace internal {

template<class Mesh>
struct OpenVDBAdapter {
    OpenVDBAdapter() : mesh(nullptr) { }
    OpenVDBAdapter(const Mesh* mesh) : mesh(mesh) { }
    const Mesh* mesh;

    size_t polygonCount() const { return mesh->faceNumber(); } // Total number of polygons
    size_t pointCount() const { return mesh->vertexNumber(); } // Total number of points
    size_t vertexCount(size_t n) const { return mesh->face(n).vertexNumber(); } // Vertex count for polygon n
    // Return position pos in local grid index space for polygon n and vertex v
    void getIndexSpacePoint(size_t n, size_t v, openvdb::Vec3d& pos) const {
        const typename Mesh::Vertex& vertex = mesh->vertex(mesh->face(n).vertexId(v));
        const typename Mesh::Point& point = vertex.point();
        pos = openvdb::Vec3d(point.x(), point.y(), point.z());
    }
};

template<class Model>
typename Model::Mesh getBlendedMesh(
        const std::vector<Model*>& models,
        const std::vector<std::vector<double>>& vertexSelectValue,
        const nvl::Scaling3d& scaleTransform,
        const double maxDistance,
        std::vector<openvdb::FloatGrid::Ptr>& unsignedGrids,
        std::vector<openvdb::FloatGrid::Ptr>& signedGrids,
        std::vector<openvdb::Int32Grid::Ptr>& polygonGrids,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& gridBirthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace);

template<class Mesh>
Mesh convertGridToMesh(const typename openvdb::FloatGrid::Ptr& gridPtr, bool transformQuadsToTriangles);

template<class Mesh>
void attachMeshesByBorders(
        Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& meshNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices,
        std::unordered_set<typename Mesh::VertexId>& newSnappedVertices,
        std::unordered_set<typename Mesh::VertexId>& preSnappedVertices);

template<class Mesh>
void cleanPreservedMeshAfterAttaching(
        Mesh& preMesh,
        const std::unordered_set<typename Mesh::VertexId>& preNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& preSnappedVertices,
        std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& preBirthVertex,
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& preBirthFace);

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

template<class Mesh>
double interpolateVertexSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<double>& vertexSelectValue);

template<class Mesh>
double averageVertexSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const std::vector<double>& vertexSelectValue);

}

template<class Model>
void blendSurfaces(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        typename SkinMixerData<Model>::Entry& entry)
{
    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename nvl::Index Index;
    typedef typename SkinMixerData<Model>::BirthInfo::VertexInfo VertexInfo;

    Model* model = entry.model;
    Mesh& resultMesh = model->mesh;

    double maxDistance = 100.0;
    double voxelSize = nvl::maxLimitValue<Scalar>();

    std::vector<Model*> models(cluster.size());
    std::vector<std::vector<double>> vertexSelectValue(cluster.size());
    for (Index i = 0; i < cluster.size(); ++i) {
        Index eId = cluster[i];

        models[i] = data.entry(eId).model;
        vertexSelectValue[i] = data.entry(eId).select.vertex;
    }

    //Calculate scale transform (minimum 1/4 of the average length)
    for (Model* modelPtr : models) {
        Mesh& mesh = modelPtr->mesh;
        Scalar avgLength = nvl::meshAverageEdgeLength(mesh);
        voxelSize = std::min(voxelSize, avgLength);
    }

    double scaleFactor = 1.0 / voxelSize;
    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);

    //Blend meshes
    std::vector<FloatGridPtr> unsignedGrids;
    std::vector<FloatGridPtr> signedGrids;
    std::vector<IntGridPtr> polygonGrids;
    std::vector<std::vector<VertexId>> gridBirthVertex;
    std::vector<std::vector<FaceId>> gridBirthFace;

    Mesh blendedMesh = internal::getBlendedMesh(models, vertexSelectValue, scaleTransform, maxDistance, unsignedGrids, signedGrids, polygonGrids, gridBirthVertex, gridBirthFace);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);
#endif

    //Fill vertices to keep for the original meshes
    std::vector<std::unordered_set<FaceId>> meshFacesToKeep(models.size());
    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId))
                continue;

            double selectValue = internal::averageVertexSelectValue(mesh, fId, vertexSelectValue[mId]);

            if (selectValue >= KEEP_THRESHOLD) {
                meshFacesToKeep[mId].insert(fId);
            }
        }

        nvl::meshOpenFaceSelection(mesh, meshFacesToKeep[mId]);
        nvl::meshCloseFaceSelection(mesh, meshFacesToKeep[mId]);

        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
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
                FloatGridPtr& currentGrid = signedGrids[mId];
                IntGridPtr& currentPolygonGrid = polygonGrids[mId];

                FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonGrid->getConstAccessor();

                FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                if (currentPolygonIndex >= 0 && currentValue <= MAX_DISTANCE_FOR_NEW_SURFACE) {
                    FaceId originFaceId = gridBirthFace[mId][currentPolygonIndex];

                    if (meshFacesToKeep[mId].find(originFaceId) != meshFacesToKeep[mId].end()) {
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
    std::vector<double> verticesToSmoothWeights;
    std::vector<std::vector<FaceId>> newMeshFFAdj = nvl::meshFaceFaceAdjacencies(newMesh);
    for (VertexId vId = 0; vId < newMesh.nextVertexId(); ++vId) {
        if (newMesh.isVertexDeleted(vId))
            continue;

        if (nvl::meshIsBorderVertex(newMesh, vId, newMeshFFAdj))
            continue;

        const Point& point = newMesh.vertex(vId).point();

        Point scaledPoint = scaleTransform * point;
        openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

        bool found = false;
        for (Index mId = 0; mId < models.size() && !found; ++mId) {
            const Mesh& mesh = models[mId]->mesh;

            FloatGridPtr& currentGrid = signedGrids[mId];
            IntGridPtr& currentPolygonGrid = polygonGrids[mId];

            FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
            IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonGrid->getConstAccessor();

            FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
            IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

            if (currentPolygonIndex >= 0 && currentValue <= 2.0) {
                FaceId originFaceId = gridBirthFace[mId][currentPolygonIndex];

                double selectValue = internal::interpolateVertexSelectValue(mesh, originFaceId, point, vertexSelectValue[mId]);

                if (selectValue >= SMOOTHING_THRESHOLD) {
                    verticesToSmooth.push_back(vId);

                    const double smoothingWeight = 0.5 + 0.5 * (selectValue - SMOOTHING_THRESHOLD) / (1.0 - SMOOTHING_THRESHOLD);
                    verticesToSmoothWeights.push_back(smoothingWeight);

                    found = true;
                }
            }
        }
    }

    nvl::meshLaplacianSmoothing(newMesh, verticesToSmooth, verticesToSmoothWeights, 40);

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
            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

            std::vector<std::tuple<nvl::Index, double, FaceId, double>> involvedEntries;
            std::vector<Index> overThresholdValues;

            double selectValueSum = 0.0;
            for (Index mId = 0; mId < models.size(); ++mId) {
                const Mesh& mesh = models[mId]->mesh;

                FloatGridPtr& currentGrid = signedGrids[mId];
                IntGridPtr& currentPolygonGrid = polygonGrids[mId];

                FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonGrid->getConstAccessor();

                FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                if (currentPolygonIndex >= 0 && currentValue < maxDistance && currentValue > -maxDistance) {
                    FaceId originFaceId = gridBirthFace[mId][currentPolygonIndex];

                    double selectValue = internal::interpolateVertexSelectValue(mesh, originFaceId, point, vertexSelectValue[mId]);

                    selectValueSum += selectValue;

                    involvedEntries.push_back(std::make_tuple(mId, selectValue, currentPolygonIndex, currentValue));
                    if (selectValue >= KEEP_THRESHOLD) {
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
                    if (selectValueSum >= DISCARD_THRESHOLD) {
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

template<class Model>
typename Model::Mesh getBlendedMesh(
        const std::vector<Model*>& models,
        const std::vector<std::vector<double>>& vertexSelectValue,
        const nvl::Scaling3d& scaleTransform,
        const double maxDistance,
        std::vector<openvdb::FloatGrid::Ptr>& unsignedGrids,
        std::vector<openvdb::FloatGrid::Ptr>& signedGrids,
        std::vector<openvdb::Int32Grid::Ptr>& polygonGrids,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& gridBirthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace)
{
    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename openvdb::math::Transform::Ptr TransformPtr;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Point Point;    typedef typename nvl::Index Index;

    Mesh blendedMesh;

    //Minimum and maximum coordinates in the scalar fields
    openvdb::Vec3i
            min(
                std::numeric_limits<int>::max(),
                std::numeric_limits<int>::max(),
                std::numeric_limits<int>::max()),
            max(
                std::numeric_limits<int>::min(),
                std::numeric_limits<int>::min(),
                std::numeric_limits<int>::min());

    //Grid of the distance fields
    std::vector<internal::OpenVDBAdapter<Mesh>> adapters(models.size());

    unsignedGrids.resize(models.size());
    signedGrids.resize(models.size());
    polygonGrids.resize(models.size());
    gridBirthVertex.resize(models.size());
    gridBirthFace.resize(models.size());

    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;

        //Data
        Mesh currentMesh;
        FloatGridPtr& currentUnsignedGrid = unsignedGrids[mId];
        FloatGridPtr& currentSignedGrid = signedGrids[mId];
        IntGridPtr& currentPolygonGrid = polygonGrids[mId];
        internal::OpenVDBAdapter<Mesh>& currentAdapter = adapters[mId];

        //Get vertices
        std::vector<VertexId> vertices;
        for (VertexId vId = 0; vId < mesh.nextVertexId(); ++vId) {
            if (mesh.isVertexDeleted(vId))
                continue;

//            if (vertexSelectValue[mId][vId] > DISCARD_THRESHOLD) {
//                vertices.push_back(vId);
//            }

            vertices.push_back(vId);
        }
        nvl::meshTransferVerticesWithFaces(mesh, vertices, currentMesh, gridBirthVertex[mId], gridBirthFace[mId]);

        currentAdapter = internal::OpenVDBAdapter<Mesh>(&currentMesh);
        currentPolygonGrid = IntGrid::create(nvl::maxLimitValue<int>());

        //Scale mesh
        nvl::meshApplyTransformation(currentMesh, scaleTransform);

#ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/input_ " + std::to_string(mId) + ".obj", currentMesh);
#endif

        TransformPtr linearTransform =
                openvdb::math::Transform::createLinearTransform(1.0);

        //Create unsigned distance field
        currentUnsignedGrid = openvdb::tools::meshToVolume<FloatGrid>(
                    currentAdapter, *linearTransform, maxDistance, maxDistance, openvdb::tools::MeshToVolumeFlags::UNSIGNED_DISTANCE_FIELD, currentPolygonGrid.get());
        FloatGrid::Accessor currentAccessor = currentUnsignedGrid->getAccessor();

        //Eigen mesh conversion
        Mesh triangulatedMesh = currentMesh;
        nvl::meshTriangulateConvexFace(triangulatedMesh);
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        nvl::convertMeshToEigenMesh(triangulatedMesh, V, F);

        //Precompute fast winding number data
        igl::FastWindingNumberBVH fwn;
        igl::fast_winding_number(V.cast<float>().eval(), F, 2, fwn);

        //Min and max of the current configuration
        openvdb::Vec3i
                currentMin(
                    std::numeric_limits<int>::max(),
                    std::numeric_limits<int>::max(),
                    std::numeric_limits<int>::max()),
                currentMax(
                    std::numeric_limits<int>::min(),
                    std::numeric_limits<int>::min(),
                    std::numeric_limits<int>::min());

        //Min and max values
        for (FloatGrid::ValueOnIter iter = currentUnsignedGrid->beginValueOn(); iter; ++iter) {
            openvdb::math::Coord coord = iter.getCoord();

            currentMin = openvdb::Vec3i(std::min(coord.x(), currentMin.x()),
                                        std::min(coord.y(), currentMin.y()),
                                        std::min(coord.z(), currentMin.z()));

            currentMax = openvdb::Vec3i(std::max(coord.x(), currentMax.x()),
                                        std::max(coord.y(), currentMax.y()),
                                        std::max(coord.z(), currentMax.z()));

            min = openvdb::Vec3i(std::min(coord.x(), min.x()),
                                 std::min(coord.y(), min.y()),
                                 std::min(coord.z(), min.z()));

            max = openvdb::Vec3i(std::max(coord.x(), max.x()),
                                 std::max(coord.y(), max.y()),
                                 std::max(coord.z(), max.z()));
        }

        //Fast winding number input
        Eigen::MatrixXd Q;
        Eigen::VectorXf W;

        //Resize by number of voxels
        const unsigned int numberVoxels = (currentMax.x() - currentMin.x()) * (currentMax.y() - currentMin.y()) * (currentMax.z() - currentMin.z());
        Q.resize(numberVoxels, 3);

        //Fill fast winding number structure
        unsigned int currentVoxel = 0;
        for (int i = currentMin.x(); i < currentMax.x(); i++) {
            for (int j = currentMin.y(); j < currentMax.y(); j++) {
                for (int k = currentMin.z(); k < currentMax.z(); k++) {
                    openvdb::math::Coord coord(i,j,k);
                    openvdb::Vec3d p = currentUnsignedGrid->indexToWorld(coord);

                    Q(currentVoxel, 0) = p.x();
                    Q(currentVoxel, 1) = p.y();
                    Q(currentVoxel, 2) = p.z();

                    currentVoxel++;
                }
            }
        }

        //Calculate fast winding number
        igl::fast_winding_number(fwn, 2, Q.cast<float>().eval(), W);

        //Set the sign
        currentVoxel = 0;
        for (int i = currentMin.x(); i < currentMax.x(); i++) {
            for (int j = currentMin.y(); j < currentMax.y(); j++) {
                for (int k = currentMin.z(); k < currentMax.z(); k++) {
                    openvdb::math::Coord coord(i,j,k);

                    FloatGrid::ValueType dist = currentAccessor.getValue(coord);
                    //FloatGrid::ValueType dist = openvdb::tools::QuadraticSampler::sample(accessor, p);

                    int fwn = static_cast<int>(std::round(W(currentVoxel)));
                    int sign = (fwn % 2 == 0 ? 1 : -1);

                    assert(dist >= 0);
                    currentAccessor.setValue(coord, sign * dist);

                    currentVoxel++;
                }
            }
        }


        //Create openvdb mesh
        Mesh closedMesh = convertGridToMesh<Mesh>(currentUnsignedGrid, true);

        internal::OpenVDBAdapter<Mesh> closedAdapter(&closedMesh);

#ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/closed_ " + std::to_string(mId) + ".obj", closedMesh);
#endif

        currentSignedGrid = openvdb::tools::meshToVolume<FloatGrid>(
            closedAdapter, *linearTransform, maxDistance, maxDistance, 0);
    }

    FloatGridPtr blendedGrid = FloatGrid::create(maxDistance);
    FloatGrid::Accessor resultAccessor = blendedGrid->getAccessor();

    //Blend grids
    for (int i = min.x(); i < max.x(); i++) {
        for (int j = min.y(); j < max.y(); j++) {
            for (int k = min.z(); k < max.z(); k++) {
                openvdb::math::Coord coord(i,j,k);

                Point point(i, j, k);

                std::vector<std::pair<FloatGrid::ValueType, double>> involvedEntries;
                std::vector<Index> overThresholdValues;

                double selectValueSum = 0.0;
                for (Index mId = 0; mId < models.size(); ++mId) {
                    const Mesh& mesh = models[mId]->mesh;

                    FloatGridPtr& currentGrid = signedGrids[mId];
                    IntGridPtr& currentPolygonGrid = polygonGrids[mId];

                    FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                    IntGrid::ConstAccessor currentPolygonAccessor = currentPolygonGrid->getConstAccessor();

                    FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                    IntGrid::ValueType currentPolygon = currentPolygonAccessor.getValue(coord);

                    if (currentPolygon >= 0 && currentPolygon < nvl::maxLimitValue<int>() && currentValue < maxDistance && currentValue > -maxDistance) {
                        FaceId originFaceId = gridBirthFace[mId][currentPolygon];

                        double selectValue = internal::interpolateVertexSelectValue(mesh, originFaceId, point, vertexSelectValue[mId]);

                        selectValueSum += selectValue;

                        involvedEntries.push_back(std::make_pair(currentValue, selectValue));
                        if (selectValue >= KEEP_THRESHOLD) {
                            overThresholdValues.push_back(involvedEntries.size() - 1);
                        }
                    }
                }

                FloatGrid::ValueType resultValue = 0.0;

                if (involvedEntries.size() == 0) {
                    resultValue = maxDistance;
                }
                else if (overThresholdValues.size() == 1) {
                    FloatGrid::ValueType currentDistance = involvedEntries[overThresholdValues[0]].first;
                    resultValue = currentDistance;
                }
                else {
                    for (Index m = 0; m < involvedEntries.size(); ++m) {

                        FloatGrid::ValueType currentDistance = involvedEntries[m].first;
                        if (selectValueSum >= DISCARD_THRESHOLD) {
                            double selectValue = involvedEntries[m].second;

                            assert(currentDistance < maxDistance && currentDistance > -maxDistance);

                            selectValue /= selectValueSum;

                            resultValue += currentDistance * selectValue;
                        }
                        else {
                            resultValue += (1.0 / involvedEntries.size()) * currentDistance;
                        }
                    }
                }

                resultAccessor.setValue(coord, resultValue);
            }
        }
    }

    //Convert to mesh
    blendedMesh = convertGridToMesh<Mesh>(blendedGrid, true);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/blendedMesh_non_rescaled.obj", blendedMesh);
#endif

    //Rescale back
    nvl::meshApplyTransformation(blendedMesh, scaleTransform.inverse());

    return blendedMesh;
}

template<class Mesh>
Mesh convertGridToMesh(const typename openvdb::FloatGrid::Ptr& gridPtr, bool transformQuadsToTriangles)
{
    typedef typename Mesh::Point Point;
    typedef typename openvdb::FloatGrid FloatGrid;

    Mesh mesh;

    const FloatGrid& grid = *gridPtr;

    //OpenVDB to mesh
    std::vector<openvdb::Vec3f> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;
    openvdb::tools::volumeToMesh(grid, points, triangles, quads);

    //Convert to mesh
    for (const openvdb::Vec3f& p : points) {
        mesh.addVertex(Point(p.x(), p.y(), p.z()));
    }
    for (const openvdb::Vec3I& t : triangles) {
        mesh.addFace(t.x(), t.y(), t.z());
    }
    for (const openvdb::Vec4I& q : quads) {
        if (transformQuadsToTriangles) {
            mesh.addFace(q.x(), q.y(), q.z());
            mesh.addFace(q.x(), q.z(), q.w());
        }
        else {
            mesh.addFace(q.x(), q.y(), q.z(), q.w());
        }
    }

    return mesh;
}

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
            }
        }
    }

    for (Index dChainId = 0; dChainId < dChains.size(); ++dChainId) {
        const std::vector<VertexId>& dChain = dChains[dChainId];
        for (const VertexId& vId : dChain) {
            if (destNonSnappableVertices.find(vId) != destNonSnappableVertices.end()) {
                dSnappable[dChainId] = false;
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
    std::vector<std::vector<double>> candidateDClosestParametrization;

    for (Index dChainId = 0; dChainId < dChains.size(); ++dChainId) {
        if (!dSnappable[dChainId])
            continue;

        const std::vector<VertexId>& dChain = dChains[dChainId];

        for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
            if (!mSnappable[mChainId])
                continue;

            const std::vector<VertexId>& mChain = mChains[mChainId];

            VertexId startI = nvl::MAX_INDEX;
            VertexId startJ = nvl::MAX_INDEX;
            Scalar startDist = nvl::maxLimitValue<Scalar>();

            //Compute the start vertices (closest projected point)
            std::vector<VertexId> minDClosestVertex(dChain.size(), nvl::MAX_INDEX);
            std::vector<double> minDClosestParametrization(dChain.size(), nvl::maxLimitValue<double>());
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
                        minDClosestParametrization[j] = t;

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
                const Scalar& targetT = minDClosestParametrization[j];

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
            candidateDClosestParametrization.push_back(minDClosestParametrization);
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

    for (Index candidateId : orderedCandidateId) {
        Index dChainId = candidateDChains[candidateId];
        Index mChainId = candidateMChains[candidateId];

        if (!dSnappable[dChainId] || !mSnappable[mChainId]) {
            candidateDClosestVertex[candidateId].clear();
            candidateDClosestParametrization[candidateId].clear();
            continue;
        }

        const Index& startJ = candidateDStartIndex[candidateId];
        const Index& startI = candidateMStartIndex[candidateId];
        const std::vector<Index>& dClosestVertex = candidateDClosestVertex[candidateId];
        const std::vector<double>& dClosestParametrization = candidateDClosestParametrization[candidateId];

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
                mSmoothPoint[i] = 0.5 * mChainPointsCopy[i] + 0.5 * (mChainPointsCopy[prevI] + mChainPointsCopy[nextI]);
            }

            std::vector<Point> dChainPointsCopy = dSmoothPoint;
            for (Index j = 0; j < dSmoothPoint.size(); ++j) {
                Index nextJ = (j + 1) % dSmoothPoint.size();
                Index prevJ = j > 0 ? j - 1 : dSmoothPoint.size() - 1;
                dSmoothPoint[j] = 0.5 * dChainPointsCopy[j] + 0.5 * (dChainPointsCopy[prevJ] + dChainPointsCopy[nextJ]);
            }
        }

        //Parametrization functions on length
        std::vector<double>& mParametrization = mFinalParametrization[candidateId];
        mParametrization.resize(mChain.size());
        std::vector<double>& dParametrization = dFinalParametrization[candidateId];
        dParametrization.resize(dChain.size());

        //Total distances
        Scalar dChainTotalDist = 0.0;
        Scalar mChainTotalDist = 0.0;
        for (Index j = 0; j < dChain.size(); ++j) {
            Index nextJ = (j + 1) % dChain.size();
            nvl::Vector3d vec = dSmoothPoint[nextJ] - dSmoothPoint[j];
            dChainTotalDist += vec.norm();
        }
        for (Index i = 0; i < mChain.size(); ++i) {
            Index nextI = (i + 1) % mChain.size();
            nvl::Vector3d vec = mSmoothPoint[nextI] - mSmoothPoint[i];
            mChainTotalDist += vec.norm();
        }

        //Compute parametrization for destination mesh
        Scalar dChainCurrentDist = 0.0;
        dParametrization[startJ] = 0.0;
        for (Index j = 0; j < dChain.size() - 1; ++j) {
            Index currentJ = (startJ + j) % dChain.size();
            Index nextJ = (currentJ + 1) % dChain.size();

            nvl::Vector3d vec = dSmoothPoint[nextJ] - dSmoothPoint[currentJ];
            dChainCurrentDist += vec.norm();

            dParametrization[nextJ] = std::max(std::min(dChainCurrentDist / dChainTotalDist, 1.0), 0.0);
        }

        //Compute offset of the parametrization
        const double offsetT = (
                    dClosestParametrization[startJ] * (
                        tmpMesh.vertex(mChain[(startI + 1) % mChain.size()]).point() -
                        tmpMesh.vertex(mChain[startI]).point()
                    ).norm()
                ) / mChainTotalDist;

        //Compute parametrization for destination mesh
        Scalar mChainCurrentDist = 0.0;
        for (Index i = 0; i < mChain.size(); ++i) {
            Index currentI = (startI + i) % mChain.size();
            Index nextI = (currentI + 1) % mChain.size();

            nvl::Vector3d vec = mSmoothPoint[nextI] - mSmoothPoint[currentI];
            mChainCurrentDist += vec.norm();

            mParametrization[nextI] = std::max(std::min((mChainCurrentDist / mChainTotalDist) - offsetT, 1.0), 0.0);
        }

        //Optimization parameters
        const double alpha = 0.9;
        const double closestCost = alpha;
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
                vars[i] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "t" + std::to_string(i));
            }

            //Objective function
            GRBQuadExpr obj = 0;

            //Length parametrization cost
            for (size_t i = 0; i < mChain.size(); i++) {
                double value = mParametrization[i];
                obj += lengthCost * ((vars[i] - value) * (vars[i] - value)) / mChain.size();
            }

            //Distance cost
            for (size_t j = 0; j < dChain.size(); j++) {
                Index currentI = dClosestVertex[j];
                double currentT = dClosestParametrization[j];
                Index nextI = (currentI + 1) % mChain.size();

                Scalar parametrizedDistance = (tmpMesh.vertex(mChain[nextI]).point() - tmpMesh.vertex(mChain[currentI]).point()).norm() / mChainTotalDist;

                double value1 = dParametrization[j] - currentT * parametrizedDistance;
                double value2 = dParametrization[j] + (1 - currentT) * parametrizedDistance;

                obj += closestCost * (
                        0.5 * ((vars[currentI] - value1) * (vars[currentI] - value1)) +
                        0.5 * ((vars[nextI] - value2) * (vars[nextI] - value2))
                    ) / dChain.size();
            }

            //Add constraints
            model.addConstr(vars[startI] == std::min(0.999999, mParametrization[startI]));
            for (size_t i = 1; i < mChain.size(); i++) {
                Index currentI = (startI + i) % mChain.size();
                Index nextI = (currentI + 1) % mChain.size();
                model.addConstr(vars[nextI] >= vars[currentI] + 0.000001);
            }

            //Set objective function
            model.setObjective(obj, GRB_MINIMIZE);

            //Optimize model
            model.optimize();

            //Retrieve result
            for (size_t i = 0; i < mChain.size(); i++) {
                mParametrization[i] = vars[i].get(GRB_DoubleAttr_X);
            }

            std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
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
void cleanPreservedMeshAfterAttaching(
        Mesh& preMesh,
        const std::unordered_set<typename Mesh::VertexId>& preNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& preSnappedVertices,
        std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& preBirthVertex,
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& preBirthFace)
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

    //Clean mesh from components that are not used
    std::vector<FaceId> preFacesToKeep;
    for (const Index& cId : preUsedComponents) {
        const std::vector<FaceId>& connectedComponent = preConnectedComponents[cId];
        preFacesToKeep.insert(preFacesToKeep.end(), connectedComponent.begin(), connectedComponent.end());
    }

    //Save in the final mesh data-structure
    Mesh preMeshCopy = preMesh;
    std::vector<VertexId> cleaningBirthVertex;
    std::vector<FaceId> cleaningBirthFace;
    preMesh.clear();
    nvl::meshTransferFaces(preMeshCopy, preFacesToKeep, preMesh, cleaningBirthVertex, cleaningBirthFace);

    std::vector<std::pair<Index, VertexId>> tmpPreBirthVertex(preMesh.nextVertexId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));;
    std::vector<std::pair<Index, FaceId>> tmpPreBirthFace(preMesh.nextFaceId(), std::make_pair(nvl::MAX_INDEX, nvl::MAX_INDEX));
    for (VertexId vId = 0; vId < preMesh.nextVertexId(); ++vId) {
        if (preMesh.isVertexDeleted(vId))
            continue;

        assert(cleaningBirthVertex[vId] != nvl::MAX_INDEX);
        const std::pair<Index, VertexId>& pair = preBirthVertex[cleaningBirthVertex[vId]];
        assert(pair.first != nvl::MAX_INDEX);
        assert(pair.second != nvl::MAX_INDEX);
        tmpPreBirthVertex[vId] = std::make_pair(pair.first, pair.second);
    }
    for (FaceId fId = 0; fId < preMesh.nextFaceId(); ++fId) {
        if (preMesh.isFaceDeleted(fId))
            continue;

        assert(cleaningBirthFace[fId] != nvl::MAX_INDEX);
        const std::pair<Index, FaceId>& pair = preBirthFace[cleaningBirthFace[fId]];
        assert(pair.first != nvl::MAX_INDEX);
        assert(pair.second != nvl::MAX_INDEX);
        tmpPreBirthFace[fId] = std::make_pair(pair.first, pair.second);
    }

    preBirthVertex = tmpPreBirthVertex;
    preBirthFace = tmpPreBirthFace;
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
    par.regularityForQuadrilaterals = true;
    par.regularityForNonQuadrilaterals = false;
    par.regularityNonQuadrilateralWeight = 0.0;
    par.feasibilityFix = true;
    par.hardParityConstraint = true;
    par.timeLimit = 60;
    par.gapLimit = 0.0;
    par.minimumGap = 0.2;
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

template<class Mesh>
double interpolateVertexSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<double>& vertexSelectValue)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::Point Point;

    //Get polygon points and select values
    const Face& face = mesh.face(faceId);

    std::vector<Point> polygon(face.vertexNumber());
    std::vector<double> values(face.vertexNumber());
    for (VertexId j = 0; j < face.vertexNumber(); ++j) {
        const VertexId& vId = face.vertexId(j);

        polygon[j] = mesh.vertex(vId).point();
        values[j] = vertexSelectValue[vId];
    }

    //Interpolation on polygon using barycenter subdivision
    double value = nvl::barycentricInterpolationBarycenterSubdivision(
        polygon,
        point,
        values,
        true);

    return value;
}

template<class Mesh>
double averageVertexSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const std::vector<double>& vertexSelectValue)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;

    const Face& face = mesh.face(faceId);

    double selectValue = 0.0;
    for (VertexId j : face.vertexIds()) {
        selectValue += vertexSelectValue[j];
    }
    selectValue /= face.vertexNumber();

    return selectValue;
}

}

}
