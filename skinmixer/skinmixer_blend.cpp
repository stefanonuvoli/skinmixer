#include "skinmixer_blend.h"

#ifdef foreach
  #undef foreach
#endif
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

#include <gurobi_c++.h>

#include <nvl/math/closest_point.h>
#include <nvl/math/numeric_limits.h>

#include <nvl/models/mesh_geometric_information.h>
#include <nvl/models/mesh_morphological_operations.h>
#include <nvl/models/mesh_transformations.h>
#include <nvl/models/mesh_triangulate.h>
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

#ifdef SAVE_MESHES
#include <nvl/io/mesh_io.h>
#endif

#include "includes/quad_steps.h"

#define KEEP_THRESHOLD 0.99
#define DISCARD_THRESHOLD 0.01
#define SMOOTHING_THRESHOLD 0.94

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
        const std::vector<std::vector<float>>& vertexSelectValue,
        const nvl::Scaling3d& scaleTransform,
        const double maxDistance,
        std::vector<openvdb::FloatGrid::Ptr>& signedGrids,
        std::vector<openvdb::Int32Grid::Ptr>& polygonIndexGrid,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& gridBirthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace);

template<class Mesh>
void attachMeshToMeshByBorders(
        Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& meshNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices,
        std::vector<typename Mesh::VertexId>& snappedVertices);

template<class Mesh>
Mesh quadrangulateMesh(
        const Mesh& newMesh,
        const Mesh& preMesh,
        const Mesh& blendedMesh,
        Mesh& quadrangulation,
        std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& birthVertex,
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& birthFace);

}

template<class Model>
void blendModels(
        const std::vector<Model*>& models,
        const std::vector<std::vector<float>>& vertexSelectValue,
        Model* resultModel,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::VertexId>>& birthVertex,
        std::vector<std::pair<nvl::Index, typename Model::Mesh::FaceId>>& birthFace)
{
    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename nvl::Index Index;

    double maxDistance = 100.0;
    double voxelSize = nvl::maxLimitValue<Scalar>();

    //Calculate scale transform (minimum 1/4 of the average length)
    for (Model* modelPtr : models) {
        Mesh& mesh = modelPtr->mesh;
        Scalar avgLength = nvl::meshAverageEdgeLength(mesh);
        voxelSize = std::min(voxelSize, avgLength / 2.0);
    }

    double scaleFactor = 1.0 / voxelSize;
    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);

    //Blend meshes
    std::vector<FloatGridPtr> signedGrids;
    std::vector<IntGridPtr> polygonIndexGrid;
    std::vector<std::vector<VertexId>> gridBirthVertex;
    std::vector<std::vector<FaceId>> gridBirthFace;

    Mesh blendedMesh = internal::getBlendedMesh(models, vertexSelectValue, scaleTransform, maxDistance, signedGrids, polygonIndexGrid, gridBirthVertex, gridBirthFace);

    //Rescale back
    nvl::meshApplyTransformation(blendedMesh, scaleTransform.inverse());

#ifdef SAVE_MESHES
    nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);
#endif


    //Fill vertices to keep for the original meshes
    std::vector<std::unordered_set<FaceId>> meshFacesToKeep(models.size());
    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId))
                continue;

            float fuzzyValue = 0.0;
            for (VertexId j : mesh.face(fId).vertexIds()) {
                fuzzyValue += vertexSelectValue[mId][j];
            }
            fuzzyValue /= mesh.face(fId).vertexNumber();

            if (fuzzyValue >= KEEP_THRESHOLD) {
                meshFacesToKeep[mId].insert(fId);
            }
        }
    }

    Mesh newMesh;
    Mesh preMesh;

    //Create preserved mesh
    std::vector<typename Mesh::VertexId> preBirthVertices;
    std::vector<typename Mesh::FaceId> preBirthFaces;
    std::unordered_set<VertexId> preNonSnappableVertices;
    for (Index mId = 0; mId < models.size(); ++mId) {
        Mesh& mesh = models[mId]->mesh;

        nvl::meshOpenFaceSelection(mesh, meshFacesToKeep[mId]);
        nvl::meshCloseFaceSelection(mesh, meshFacesToKeep[mId]);

        nvl::Size lastVertexId = preMesh.nextVertexId();
        nvl::Size lastFaceId = preMesh.nextFaceId();

        assert(preBirthVertices.size() == lastVertexId);
        assert(preBirthFaces.size() == lastFaceId);
        nvl::meshTransferFaces(mesh, std::vector<VertexId>(meshFacesToKeep[mId].begin(), meshFacesToKeep[mId].end()), preMesh, preBirthVertices, preBirthFaces);

        birthVertex.resize(preMesh.nextVertexId(), std::make_pair(nvl::MAX_ID, nvl::MAX_ID));
        birthFace.resize(preMesh.nextFaceId(), std::make_pair(nvl::MAX_ID, nvl::MAX_ID));
        for (Index vId = lastVertexId; vId < preMesh.nextVertexId(); ++vId) {
            assert(preBirthVertices[vId] != nvl::MAX_ID);
            birthVertex[vId] = std::make_pair(mId, preBirthVertices[vId]);
        }
        for (Index fId = lastFaceId; fId < preMesh.nextFaceId(); ++fId) {
            assert(preBirthFaces[fId] != nvl::MAX_ID);
            birthFace[fId] = std::make_pair(mId, preBirthFaces[fId]);
        }

        //Find vertices that were already in the border of the original mesh
        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(mesh);
        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());
        for (VertexId vId = lastVertexId; vId < preMesh.nextVertexId(); vId++) {
            if (borderVerticesSet.find(preBirthVertices[vId]) != borderVerticesSet.end()) {
                preNonSnappableVertices.insert(vId);
            }
        }
    }

#ifdef SAVE_MESHES
    nvl::meshSaveToFile("results/preMesh.obj", preMesh);
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
                IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

                FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue <= 2.0) {
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

    //Erode-dilate
    nvl::meshOpenFaceSelection(blendedMesh, blendedMeshFacesToKeep); 

    //Transfer in the new surface the faces to be kept
    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(blendedMeshFacesToKeep.begin(), blendedMeshFacesToKeep.end()), newMesh);

#ifdef SAVE_MESHES
    nvl::meshSaveToFile("results/newMeshRegularized.obj", newMesh);
#endif

    std::vector<VertexId> snappedVertices;
    internal::attachMeshToMeshByBorders(newMesh, preMesh, std::unordered_set<VertexId>(), preNonSnappableVertices, snappedVertices);

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
            IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

            FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
            IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

            FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
            IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

            if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue <= 2.0) {
                FaceId originFaceId = gridBirthFace[mId][currentPolygonIndex];

                double selectValue = 0.0;
                for (VertexId j : mesh.face(originFaceId).vertexIds()) {
                    selectValue += vertexSelectValue[mId][j];
                }
                selectValue /= mesh.face(originFaceId).vertexNumber();

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

#ifdef SAVE_MESHES
    nvl::meshSaveToFile("results/newMesh.obj", newMesh);
#endif

    //Get final mesh
    Mesh quadrangulation;
    Mesh resultMesh = internal::quadrangulateMesh(newMesh, preMesh, blendedMesh, quadrangulation, birthVertex, birthFace);

#ifdef SAVE_MESHES
    nvl::meshSaveToFile("results/quadrangulation.obj", quadrangulation);
    nvl::meshSaveToFile("results/resultMesh.obj", resultMesh);
#endif

    resultModel->mesh = resultMesh;
}

namespace internal {

template<class Model>
typename Model::Mesh getBlendedMesh(
        const std::vector<Model*>& models,
        const std::vector<std::vector<float>>& vertexSelectValue,
        const nvl::Scaling3d& scaleTransform,
        const double maxDistance,
        std::vector<openvdb::FloatGrid::Ptr>& signedGrids,
        std::vector<openvdb::Int32Grid::Ptr>& polygonIndexGrid,
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

    signedGrids.resize(models.size());
    polygonIndexGrid.resize(models.size());
    gridBirthVertex.resize(models.size());
    gridBirthFace.resize(models.size());

    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;

        //Data
        Mesh currentMesh;
        FloatGridPtr& currentSignedGrid = signedGrids[mId];
        IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];
        internal::OpenVDBAdapter<Mesh>& currentAdapter = adapters[mId];

        //Get non-zero vertices
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
        currentPolygonIndexGrid = IntGrid::create(nvl::maxLimitValue<int>());

        //Scale mesh
        nvl::meshApplyTransformation(currentMesh, scaleTransform);

#ifdef SAVE_MESHES
        nvl::meshSaveToFile("results/input_ " + std::to_string(mId) + ".obj", currentMesh);
#endif

        TransformPtr linearTransform =
                openvdb::math::Transform::createLinearTransform(1.0);

        //Create unsigned distance field
        FloatGridPtr tmpGrid = openvdb::tools::meshToVolume<FloatGrid>(
                    currentAdapter, *linearTransform, maxDistance, maxDistance, openvdb::tools::MeshToVolumeFlags::UNSIGNED_DISTANCE_FIELD, currentPolygonIndexGrid.get());
        FloatGrid::Accessor currentAccessor = tmpGrid->getAccessor();

        //Eigen mesh conversion
        Mesh triangulatedMesh = currentMesh;
        nvl::meshTriangulate(triangulatedMesh);
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
        for (FloatGrid::ValueOnIter iter = tmpGrid->beginValueOn(); iter; ++iter) {
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
                    openvdb::Vec3d p = tmpGrid->indexToWorld(coord);

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
        std::vector<openvdb::Vec3f> points;
        std::vector<openvdb::Vec3I> triangles;
        std::vector<openvdb::Vec4I> quads;
        openvdb::tools::volumeToMesh(*tmpGrid, points, triangles, quads);

        //Convert to mesh
        Mesh closedMesh;
        internal::OpenVDBAdapter<Mesh> closedAdapter(&closedMesh);
        for (const openvdb::Vec3f& p : points) {
            closedMesh.addVertex(Point(p.x(), p.y(), p.z()));
        }
        for (const openvdb::Vec3I& t : triangles) {
            closedMesh.addFace(t.x(), t.y(), t.z());
        }
        for (const openvdb::Vec4I& q : quads) {
            closedMesh.addFace(q.x(), q.y(), q.z());
            closedMesh.addFace(q.x(), q.z(), q.w());
        }

#ifdef SAVE_MESHES
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

                std::vector<std::pair<FloatGrid::ValueType, double>> involvedEntries;
                std::vector<Index> overThresholdValues;

                float selectValueSum = 0.0;
                for (Index mId = 0; mId < models.size(); ++mId) {
                    const Mesh& mesh = models[mId]->mesh;

                    FloatGridPtr& currentGrid = signedGrids[mId];
                    IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

                    FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                    IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                    FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                    IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                    if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue < maxDistance && currentValue > -maxDistance) {
                        FaceId originFaceId = gridBirthFace[mId][currentPolygonIndex];

                        float selectValue = 0.0;
                        for (VertexId j : mesh.face(originFaceId).vertexIds()) {
                            selectValue += vertexSelectValue[mId][j];
                        }
                        selectValue /= mesh.face(originFaceId).vertexNumber();

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
                    if (selectValueSum >= DISCARD_THRESHOLD) {
                        for (Index m = 0; m < involvedEntries.size(); ++m) {
                            FloatGrid::ValueType currentDistance = involvedEntries[m].first;
                            double selectValue = involvedEntries[m].second;

                            assert(currentDistance < maxDistance && currentDistance > -maxDistance);

                            selectValue /= selectValueSum;

                            resultValue += currentDistance * selectValue;
                        }
                    }
                    else {
                        for (Index m = 0; m < involvedEntries.size(); ++m) {
                            FloatGrid::ValueType currentDistance = involvedEntries[m].first;
                            resultValue += (1.0 / involvedEntries.size()) * currentDistance;
                        }
                    }
                }

                resultAccessor.setValue(coord, resultValue);
            }
        }
    }

    //Create openvdb mesh
    std::vector<openvdb::Vec3f> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;
    openvdb::tools::volumeToMesh(*blendedGrid, points, triangles, quads);

    //Convert to mesh
    for (const openvdb::Vec3f& p : points) {
        blendedMesh.addVertex(nvl::Point3d(p.x(), p.y(), p.z()));
    }
    for (const openvdb::Vec3I& t : triangles) {
        blendedMesh.addFace(t.x(), t.y(), t.z());
    }
    for (const openvdb::Vec4I& q : quads) {
        blendedMesh.addFace(q.x(), q.y(), q.z());
        blendedMesh.addFace(q.x(), q.z(), q.w());
    }

    return blendedMesh;
}

template<class Mesh>
void attachMeshToMeshByBorders(
        Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& meshNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices,
        std::vector<typename Mesh::VertexId>& snappedVertices)
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
    for (Index mChainId = 0; mChainId < mConnectedComponents.size(); mChainId++) {
        const std::vector<FaceId>& component = mConnectedComponents[mChainId];

        std::vector<std::vector<VertexId>> componentVertexChains = nvl::meshSubsetBorderVertexChains(
                    tmpMesh, std::unordered_set<FaceId>(component.begin(), component.end()), mFFAdj);

        for (std::vector<VertexId> componentVertexChain : componentVertexChains) {
            //We reverse to match the order (clockwise)
            std::reverse(componentVertexChain.begin(), componentVertexChain.end());

            mChains.push_back(componentVertexChain);
            mChainsComponent.push_back(mChainId);
        }
    }

    //Calculate chains for destination mesh
    std::vector<std::vector<VertexId>> dChains = nvl::meshBorderVertexChains(destMesh);

    //Define snappable flags
    std::vector<bool> mSnappableChain(mChains.size(), true);
    std::vector<bool> dSnappableChain(dChains.size(), true);
    for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
        const std::vector<VertexId>& mChain = mChains[mChainId];
        for (const VertexId& vId : mChain) {
            if (meshNonSnappableVertices.find(vId) != meshNonSnappableVertices.end()) {
                mSnappableChain[mChainId] = false;
            }
        }
    }
    for (Index dChainId = 0; dChainId < dChains.size(); ++dChainId) {
        const std::vector<VertexId>& dChain = dChains[dChainId];
        for (const VertexId& vId : dChain) {
            if (destNonSnappableVertices.find(vId) != destNonSnappableVertices.end()) {
                dSnappableChain[dChainId] = false;
            }
        }
    }

    //Data to be used for the attaching
    std::unordered_set<Index> usedComponents;
    std::vector<std::pair<Index, Index>> selectedMChains;
    std::vector<std::pair<Index, Index>> selectedDChains;
    std::vector<std::vector<double>> selectedMParametrization;
    std::vector<std::vector<double>> selectedDParametrization;

    //Find parametrizations
    for (Index dChainId = 0; dChainId < dChains.size(); ++dChainId) {
        if (!dSnappableChain[dChainId])
            continue;

        const std::vector<VertexId>& dChain = dChains[dChainId];

        Scalar bestScore = nvl::maxLimitValue<Scalar>();
        Index bestMChainId = nvl::MAX_ID;

        std::pair<Index, Index> bestMChain;
        std::pair<Index, Index> bestDChain;
        std::vector<double> bestMParametrization;
        std::vector<double> bestDParametrization;

        for (Index mChainId = 0; mChainId < mChains.size(); ++mChainId) {
            if (!mSnappableChain[mChainId])
                continue;

            const std::vector<VertexId>& mChain = mChains[mChainId];

            VertexId startI = nvl::MAX_ID;
            VertexId startJ = nvl::MAX_ID;
            Scalar startDist = nvl::maxLimitValue<Scalar>();
            double startT = nvl::maxLimitValue<double>();

            //Compute the start vertices
            std::vector<VertexId> closestIMap(dChain.size(), nvl::MAX_ID);
            std::vector<double> closestTMap(dChain.size(), nvl::maxLimitValue<double>());
            for (Index j = 0; j < dChain.size(); ++j) {
                const Point& pPoint = destMesh.vertex(dChain[j]).point();

                Scalar closestDist = nvl::maxLimitValue<Scalar>();

                for (Index i = 0; i < mChain.size(); ++i) {
                    Index nextI = (i + 1) % mChain.size();

                    const Point& nPoint = tmpMesh.vertex(mChain[i]).point();
                    const Point& nNextPoint = tmpMesh.vertex(mChain[nextI]).point();

                    double t;
                    Point projectionPoint = nvl::getClosestPointOnSegment(nPoint, nNextPoint, pPoint, t);
                    Scalar dist = (projectionPoint - pPoint).norm();

                    if (dist <= closestDist) {
                        closestDist = dist;
                        closestIMap[j] = i;
                        closestTMap[j] = t;

                        if (dist <= startDist) {
                            startDist = dist;
                            startI = i;
                            startJ = j;
                            startT = t;
                        }
                    }
                }
            }

            //Compute score by computing the distortion
            Scalar pairScore = 0.0;
            for (Index j = 0; j < dChain.size(); ++j) {
                Index currentI = closestIMap[j];
                Index nextI = (currentI + 1) % mChain.size();

                const Point& pPoint = destMesh.vertex(dChain[j]).point();
                const Scalar& targetT = closestTMap[j];

                Point currentIPoint = tmpMesh.vertex(mChain[currentI]).point();
                Point nextIPoint = tmpMesh.vertex(mChain[nextI]).point();

                Point nPoint = currentIPoint + (nextIPoint - currentIPoint) * targetT;

                pairScore += (nPoint - pPoint).norm();
            }

            //We save the best score pair of chains
            if (pairScore <= bestScore) {
                bestScore = pairScore;

                //Smooth vertices in the loops
                std::vector<Point> mSmoothPoint(mChain.size());
                std::vector<Point> dSmoothPoint(dChain.size());
                for (Index i = 0; i < mChain.size(); ++i) {
                    mSmoothPoint[i] = tmpMesh.vertex(mChain[i]).point();
                }
                for (Index j = 0; j < dChain.size(); ++j) {
                    dSmoothPoint[j] = destMesh.vertex(dChain[j]).point();
                }

                //TODO FUNCTION IN NVL
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
                std::vector<double> mParametrization(mChain.size());
                std::vector<double> dParametrization(dChain.size());

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

                    dParametrization[nextJ] = dChainCurrentDist / dChainTotalDist;
                    assert(dParametrization[nextJ] >= 0.0 && dParametrization[nextJ] <= 1.0);
                }

                //Compute offset of the parametrization
                const double offsetT = (
                            startT * (
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

                    mParametrization[nextI] = (mChainCurrentDist / mChainTotalDist) - offsetT;
                    assert(mParametrization[nextI] >= 0.0 && mParametrization[nextI] <= 1.0);
                }

                //Optimization parameters
                const double alpha = 0.8;
                const double closestCost = alpha / mChain.size();
                const double lengthCost = (1 - alpha) / (dChain.size() * 2);

                //Final parametrization
                try {
                    GRBEnv env = GRBEnv();

                    GRBModel model = GRBModel(env);

                    //model.set(GRB_IntParam_OutputFlag, 0);

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
                        obj += lengthCost * (vars[i] - value) * (vars[i] - value);
                    }

                    //Distance cost
                    for (size_t j = 0; j < dChain.size(); j++) {
                        Index currentI = closestIMap[j];
                        double currentT = closestTMap[j];
                        Index nextI = (currentI + 1) % mChain.size();

                        Scalar parametrizedDistance = (tmpMesh.vertex(mChain[nextI]).point() - tmpMesh.vertex(mChain[currentI]).point()).norm() / mChainTotalDist;

                        double value1 = dParametrization[j] - currentT * parametrizedDistance;
                        obj += closestCost * (vars[currentI] - value1) * (vars[currentI] - value1);

                        double value2 = dParametrization[j] + (1 - currentT) * parametrizedDistance;
                        obj += closestCost * (vars[nextI] - value2) * (vars[nextI] - value2);
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

                bestMChain = std::make_pair(mChainId, startI);
                bestDChain = std::make_pair(dChainId, startJ);
                bestMParametrization = mParametrization;
                bestDParametrization = dParametrization;
                bestMChainId = mChainId;
            }
        }

        //Select the best pair to connect
        if (bestScore < nvl::maxLimitValue<Scalar>()) {
            selectedMChains.push_back(bestMChain);
            selectedDChains.push_back(bestDChain);
            selectedMParametrization.push_back(bestMParametrization);
            selectedDParametrization.push_back(bestDParametrization);

            usedComponents.insert(mChainsComponent[bestMChainId]);

            dSnappableChain[dChainId] = false;
            mSnappableChain[bestMChainId] = false;
        }
    }

    //Clean mesh from components that are non-splitted in borders
    std::vector<FaceId> facesToKeep;
    for (const Index& cId : usedComponents) {
        const std::vector<FaceId>& connectedComponent = mConnectedComponents[cId];
        facesToKeep.insert(facesToKeep.end(), connectedComponent.begin(), connectedComponent.end());
    }

    //Save in the final mesh data-structure
    mesh.clear();
    std::vector<VertexId> cleaningBirthVertex;
    std::vector<FaceId> cleaningBirthFace;
    nvl::meshTransferFaces(tmpMesh, facesToKeep, mesh, cleaningBirthVertex, cleaningBirthFace);

    std::vector<VertexId> cleaningVertexMap = nvl::getInverseMap(cleaningBirthVertex);
    for (Index k = 0; k < selectedMChains.size(); ++k) {
        Index mChainId = selectedMChains[k].first;
        std::vector<VertexId>& mChain = mChains[mChainId];

        for (Index i = 0; i < mChain.size(); ++i) {
            assert(mChain[i] != nvl::MAX_ID);
            mChain[i] = cleaningVertexMap[mChain[i]];
        }
    }

#ifdef SAVE_MESHES
    nvl::meshSaveToFile("results/newMeshCleaned.obj", mesh);
#endif

    std::vector<std::vector<FaceId>> meshVFAdj = nvl::meshVertexFaceAdjacencies(mesh);
    for (Index k = 0; k < selectedMChains.size(); ++k) {
        Index mChainId = selectedMChains[k].first;
        Index dChainId = selectedDChains[k].first;
        Index startI = selectedMChains[k].second;
        Index startJ = selectedDChains[k].second;

        const std::vector<VertexId>& mChain = mChains[mChainId];
        const std::vector<double>& mParametrization = selectedMParametrization[k];
        const std::vector<VertexId>& dChain = dChains[dChainId];
        const std::vector<double>& dParametrization = selectedDParametrization[k];

        Index i = 0;
        VertexId currentNVertexId = mChain[startI];
        for (Index j = 0; j < dChain.size(); ++j) {
            Index currentJ = (startJ + j) % dChain.size();
            Index currentI = (startI + i) % mChain.size();
            Index nextI = (currentI + 1) % mChain.size();

            const Point& jPoint = destMesh.vertex(dChain[currentJ]).point();

            const Scalar& pT = dParametrization[currentJ];

            while (i < mChain.size() - 1 && mParametrization[nextI] < pT) {
                i++;

                currentI = (startI + i) % mChain.size();
                nextI = (currentI + 1) % mChain.size();

                currentNVertexId = mChain[currentI];
            }

            VertexId newNVertexId = nvl::meshSplitEdge(mesh, currentNVertexId, mChain[nextI], jPoint, meshVFAdj);
            snappedVertices.push_back(newNVertexId);
            currentNVertexId = newNVertexId;
        }
    }

#ifdef SAVE_MESHES
    nvl::meshSaveToFile("results/newMeshSplitted.obj", mesh);
#endif

    std::vector<VertexId> nonCollapsed = nvl::collapseBorders(mesh, snappedVertices);
    std::cout << nonCollapsed.size() << " vertices non collapsed." << std::endl;

    if (!nonCollapsed.empty()) {
        //TODO!!
    }
}

template<class Mesh>
Mesh quadrangulateMesh(
        const Mesh& newMesh,
        const Mesh& preMesh,
        const Mesh& blendedMesh,
        Mesh& quadrangulation,
        std::vector<std::pair<nvl::Index, typename Mesh::VertexId>>& birthVertex,
        std::vector<std::pair<nvl::Index, typename Mesh::FaceId>>& birthFace)
{
    Mesh result;

    nvl::VCGTriangleMesh vcgNewMesh;
    nvl::VCGPolygonMesh vcgPreMesh;
    nvl::VCGPolygonMesh vcgQuadrangulation;
    nvl::VCGPolygonMesh vcgResult;
    nvl::VCGPolygonMesh vcgBlendedMesh;
    nvl::convertMeshToVCGMesh(newMesh, vcgNewMesh);
    nvl::convertMeshToVCGMesh(preMesh, vcgPreMesh);
    nvl::convertMeshToVCGMesh(blendedMesh, vcgBlendedMesh);

    std::unordered_map<size_t, size_t> preservedFacesMap;
    std::unordered_map<size_t, size_t> preservedVerticesMap;

    std::vector<int> newSurfaceLabel;

    QuadBoolean::internal::ChartData chartData;

    std::vector<int> ilpResult;
    std::vector<int> quadrangulationLabel;

//    //Make ILP feasible
//    QuadBoolean::internal::makeILPFeasible(vcgPreMesh, vcgNewMesh, true, true);

    //Get patch decomposition of the new surface
    std::vector<std::vector<size_t>> newSurfacePartitions;
    std::vector<std::vector<size_t>> newSurfaceCorners;
    newSurfaceLabel = QuadBoolean::internal::getPatchDecomposition(vcgNewMesh, vcgPreMesh, newSurfacePartitions, newSurfaceCorners, true, 1.0, true, false, true);

    //Get chart data
    chartData = QuadBoolean::internal::getPatchDecompositionChartData(vcgNewMesh, newSurfaceLabel, newSurfaceCorners);

    //Solve ILP to find best side size
    ilpResult = QuadBoolean::internal::findSubdivisions(
                vcgNewMesh,
                chartData,
                0.5,
                QuadBoolean::internal::LEASTSQUARES);

    //Quadrangulate
    QuadBoolean::internal::quadrangulate(
                vcgNewMesh,
                chartData,
                ilpResult,
                5,
                5,
                vcgQuadrangulation,
                quadrangulationLabel);

    //Get the result
    QuadBoolean::internal::getResult(
                vcgPreMesh, vcgQuadrangulation,
                vcgResult, vcgBlendedMesh,
                5,
                3,
                5,
                3,
                preservedFacesMap, preservedVerticesMap);

    nvl::convertVCGMeshToMesh(vcgQuadrangulation, quadrangulation);
    nvl::convertVCGMeshToMesh(vcgResult, result);

    return result;
}


}

}
