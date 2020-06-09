#include "skinmixer_mix.h"

#ifdef foreach
  #undef foreach
#endif
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

#include <nvl/models/mesh_geometric_information.h>
#include <nvl/models/mesh_transformations.h>
#include <nvl/models/mesh_triangulate.h>

#include <nvl/libigl/igl_convert.h>

#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>

//TODO DELETE
#include <nvl/io/mesh_io.h>

namespace skinmixer {

template<class Mesh>
struct OpenVDBAdapter {
    OpenVDBAdapter() : mesh(nullptr) { }
    OpenVDBAdapter(const Mesh* mesh) : mesh(mesh) { }
    const Mesh* mesh;

    size_t polygonCount() const { return mesh->faceNumber(); }        // Total number of polygons
    size_t pointCount() const { return mesh->vertexNumber(); }          // Total number of points
    size_t vertexCount(size_t n) const { return mesh->face(n).vertexNumber(); } // Vertex count for polygon n
    // Return position pos in local grid index space for polygon n and vertex v
    void getIndexSpacePoint(size_t n, size_t v, openvdb::Vec3d& pos) const {
        const typename Mesh::Vertex& vertex = mesh->vertex(mesh->face(n).vertexId(v));
        const typename Mesh::Point& point = vertex.point();
        pos = openvdb::Vec3d(point.x(), point.y(), point.z());
    }
};

template<class Mesh>
Mesh mixMeshes(
        const std::vector<Mesh*>& meshes,
        const std::vector<std::vector<float>>& vertexFuzzyValue)
{
    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename openvdb::math::Transform::Ptr TransformPtr;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;

    Mesh newMesh;

    typedef typename Mesh::Scalar Scalar;

    double scaleFactor = nvl::maxLimitValue<double>();
    double maxDistance = 100.0;

    //Calculate scale transform (minimum 1/4 of the average length)
    for (Mesh* meshPtr : meshes) {
        Scalar avgLength = nvl::meshAverageEdgeLength(*meshPtr);
        scaleFactor = std::min(scaleFactor, 1.0 / (avgLength / 2.0));
    }
    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);

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
    std::vector<FloatGridPtr> signedGrids(meshes.size());
    std::vector<OpenVDBAdapter<Mesh>> adapters(meshes.size());
    std::vector<IntGridPtr> polygonIndexGrid(meshes.size());
    std::vector<std::vector<VertexId>> birthVertex(meshes.size());
    std::vector<std::vector<FaceId>> birthFace(meshes.size());

    for (nvl::Index mId = 0; mId < meshes.size(); ++mId) {
        Mesh* meshPtr = meshes[mId];

        //Data
        Mesh currentMesh;
        FloatGridPtr& currentSignedGrid = signedGrids[mId];
        IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];
        OpenVDBAdapter<Mesh>& currentAdapter = adapters[mId];

        //Get non-zero vertices
        std::vector<VertexId> vertices;
        for (VertexId vId = 0; vId < meshPtr->nextVertexId(); ++vId) {
            if (meshPtr->isVertexDeleted(vId))
                continue;

            assert(vertexFuzzyValue[mId][vId] > 0.0 && vertexFuzzyValue[mId][vId] < 1.0);
            if (!nvl::epsEqual(vertexFuzzyValue[mId][vId], 0.0) && !nvl::epsEqual(vertexFuzzyValue[mId][vId], 1.0)) {
                vertices.push_back(vId);
            }
        }
        nvl::meshTransferVerticesWithFaces(*meshPtr, vertices, currentMesh, birthVertex[mId], birthFace[mId]);

        currentAdapter = OpenVDBAdapter<Mesh>(&currentMesh);
        currentPolygonIndexGrid = IntGrid::create(nvl::maxLimitValue<int>());

        //Scale mesh
        nvl::meshApplyTransformation(currentMesh, scaleTransform);

        //TODO DELETE
        nvl::meshSaveToFile("results/input_ " + std::to_string(mId) + ".obj", currentMesh);

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
        OpenVDBAdapter<Mesh> closedAdapter(&closedMesh);
        for (const openvdb::Vec3f& p : points) {
            closedMesh.addVertex(nvl::Point3d(p.x(), p.y(), p.z()));
        }
        for (const openvdb::Vec3I& t : triangles) {
            closedMesh.addFace(t.x(), t.y(), t.z());
        }
        for (const openvdb::Vec4I& q : quads) {
            closedMesh.addFace(q.x(), q.y(), q.z());
            closedMesh.addFace(q.x(), q.z(), q.w());
        }

        //TODO DELETE
        nvl::meshSaveToFile("results/closed_ " + std::to_string(mId) + ".obj", closedMesh);

        currentSignedGrid = openvdb::tools::meshToVolume<FloatGrid>(
            closedAdapter, *linearTransform, maxDistance, maxDistance, 0);
    }

    FloatGridPtr resultGrid = FloatGrid::create(maxDistance);
    FloatGrid::Accessor resultAccessor = resultGrid->getAccessor();

    //Blend grids
    for (int i = min.x(); i < max.x(); i++) {
        for (int j = min.y(); j < max.y(); j++) {
            for (int k = min.z(); k < max.z(); k++) {
                openvdb::math::Coord coord(i,j,k);

                int nInvolvedGrids = 0;
                float sumFuzzy = 0.0;
                for (nvl::Index mId = 0; mId < meshes.size(); ++mId) {
                    Mesh* meshPtr = meshes[mId];

                    FloatGridPtr& currentGrid = signedGrids[mId];
                    IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

                    FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                    IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                    FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                    IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                    float fuzzyValue = 0.0;
                    if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue < maxDistance) {
                        FaceId fId = birthFace[mId][currentPolygonIndex];
                        for (VertexId vId : meshPtr->face(fId).vertexIds()) {
                            fuzzyValue += vertexFuzzyValue[mId][vId];
                        }
                        fuzzyValue /= meshPtr->face(fId).vertexNumber();

                        sumFuzzy += fuzzyValue;

                        nInvolvedGrids++;
                    }
                }

                FloatGrid::ValueType resultValue = 0.0;
                for (nvl::Index mId = 0; mId < meshes.size(); ++mId) {
                    Mesh* meshPtr = meshes[mId];

                    FloatGridPtr& currentGrid = signedGrids[mId];
                    IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

                    FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                    IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                    FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                    IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                    if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue < maxDistance) {
                        if (sumFuzzy > nvl::EPSILON) {
                            float fuzzyValue = 0.0;

                            FaceId fId = birthFace[mId][currentPolygonIndex];
                            for (VertexId vId : meshPtr->face(fId).vertexIds()) {
                                fuzzyValue += vertexFuzzyValue[mId][vId];
                            }
                            fuzzyValue /= meshPtr->face(fId).vertexNumber();

                            if (sumFuzzy > 1.0) {
                                fuzzyValue /= sumFuzzy;
                            }

                            resultValue += currentValue * fuzzyValue;
                        }
                        else {
                            resultValue += (1.0 / nInvolvedGrids) * currentValue;
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
    openvdb::tools::volumeToMesh(*resultGrid, points, triangles, quads);

    //Convert to mesh
    for (const openvdb::Vec3f& p : points) {
        newMesh.addVertex(nvl::Point3d(p.x(), p.y(), p.z()));
    }
    for (const openvdb::Vec3I& t : triangles) {
        newMesh.addFace(t.x(), t.y(), t.z());
    }
    for (const openvdb::Vec4I& q : quads) {
        newMesh.addFace(q.x(), q.y(), q.z());
        newMesh.addFace(q.x(), q.z(), q.w());
    }

    nvl::meshApplyTransformation(newMesh, scaleTransform.inverse());


    //TODO DELETE
    nvl::meshSaveToFile("results/newMesh.obj", newMesh);


    return newMesh;
}

}
