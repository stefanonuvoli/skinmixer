#include "skinmixer_openvdb_blending.h"

#ifdef SAVE_MESHES_FOR_DEBUG
#include <nvl/models/mesh_io.h>
#endif

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_transformations.h>
#include <nvl/models/mesh_triangulation.h>

#include <nvl/math/barycentric_interpolation.h>
#include <nvl/math/numeric_limits.h>

#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>

#include <nvl/models/mesh_eigen_convert.h>

#define FACE_DISCARD_THRESHOLD 0.01
#define SELECT_VALUE_MIN_THRESHOLD 0.02
#define SELECT_VALUE_MAX_THRESHOLD 0.98

namespace skinmixer {
namespace internal {

template<class Mesh>
double interpolateVertexSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<double>& vertexSelectValue);

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

//            if (vertexSelectValue[mId][vId] > FACE_DISCARD_THRESHOLD) {
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
            GridCoord coord = iter.getCoord();

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
                    GridCoord coord(i,j,k);
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
                    GridCoord coord(i,j,k);

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
                GridCoord coord(i,j,k);

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
                        if (selectValue >= SELECT_VALUE_MAX_THRESHOLD) {
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
                        if (selectValueSum >= SELECT_VALUE_MIN_THRESHOLD) {
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
Mesh convertGridToMesh(const openvdb::FloatGrid::Ptr& gridPtr, bool transformQuadsToTriangles)
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

}
}
