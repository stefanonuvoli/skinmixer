#include "skinmixer_openvdb_blending.h"

#ifdef SAVE_MESHES_FOR_DEBUG
#include <nvl/models/mesh_io.h>
#endif

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_transformations.h>
#include <nvl/models/mesh_triangulation.h>
#include <nvl/models/mesh_geometric_information.h>

#include <nvl/math/barycentric_interpolation.h>
#include <nvl/math/numeric_limits.h>
#include <nvl/math/comparisons.h>

#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>

#include <nvl/models/mesh_eigen_convert.h>

#define SELECT_VALUE_MIN_THRESHOLD 0.02
#define SELECT_VALUE_MAX_THRESHOLD 0.98
#define EXPANSION_VOXELS 15.0

namespace skinmixer {
namespace internal {

template<class Mesh>
double interpolateFaceSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<double>& vertexSelectValue);

template<class Model>
void getClosedGrids(
        const OperationType operation,
        const std::vector<const Model*>& models,
        const std::vector<const std::vector<double>*>& vertexSelectValue,
        const double& scaleFactor,
        const double& maxDistance,
        std::vector<FloatGridPtr>& unsignedGrids,
        std::vector<FloatGridPtr>& signedGrids,
        std::vector<FloatGridPtr>& closedGrids,        
        std::vector<IntGridPtr>& polygonGrids,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& gridBirthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace,
        std::vector<openvdb::Vec3i>& bbMin,
        std::vector<openvdb::Vec3i>& bbMax)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::FaceId FaceId;
    typedef typename nvl::Index Index;
    typedef typename openvdb::math::Transform::Ptr TransformPtr;

    unsignedGrids.resize(models.size());
    polygonGrids.resize(models.size());
    signedGrids.resize(models.size());
    closedGrids.resize(models.size());
    gridBirthVertex.resize(models.size());
    gridBirthFace.resize(models.size());
    bbMin.resize(models.size());
    bbMax.resize(models.size());

    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);

    for (Index mId = 0; mId < models.size(); ++mId) {
        const Mesh& mesh = models[mId]->mesh;

        std::vector<std::vector<FaceId>> meshFFAdj = nvl::meshFaceFaceAdjacencies(mesh);
        std::vector<std::vector<FaceId>> meshCC = nvl::meshConnectedComponents(mesh, meshFFAdj);

        Scalar maxExpansionDistance = EXPANSION_VOXELS / scaleFactor;

        //Enhance ffadj with closest borders in different components
        std::unordered_set<Index> alreadyMatched;
        for (Index c1 = 0; c1 < meshCC.size(); ++c1) {
            for (const FaceId& f1 : meshCC[c1]) {
                for (Index pos1 = 0; pos1 < meshFFAdj[f1].size(); ++pos1) {
                    if (meshFFAdj[f1][pos1] == nvl::MAX_INDEX) {
                        FaceId bestF2 = nvl::MAX_INDEX;
                        Index bestPos2 = nvl::MAX_INDEX;
                        Scalar bestDistance = nvl::maxLimitValue<Scalar>();
                        double bestScore = nvl::maxLimitValue<double>();

                        Point b1 = nvl::meshFaceBarycenter(mesh, f1);

                        for (Index c2 = 0; c2 < meshCC.size(); ++c2) {
                            if (c1 == c2)
                                continue;

                            for (const FaceId& f2 : meshCC[c2]) {
                                for (Index pos2 = 0; pos2 < meshFFAdj[f2].size(); ++pos2) {
                                    if (meshFFAdj[f2][pos2] == nvl::MAX_INDEX) {
                                        Point b2 = nvl::meshFaceBarycenter(mesh, f2);

                                        Scalar distance = (b1 - b2).norm();
                                        double score = distance;

                                        if (alreadyMatched.find(f2) != alreadyMatched.end()) {
                                            score *= 2;
                                        }

                                        if (score < bestScore) {
                                            bestF2 = f2;
                                            bestPos2 = pos2;
                                            bestDistance = distance;
                                            bestScore = score;
                                        }
                                    }
                                }
                            }
                        }

                        if (bestF2 != nvl::MAX_INDEX) {
                            meshFFAdj[f1][pos1] = bestF2;
                            meshFFAdj[bestF2][bestPos2] = f1;

                            alreadyMatched.insert(f1);
                            alreadyMatched.insert(bestF2);
                        }
                    }
                }
            }
        }

        double selectValueLowerLimit = 0.0;
        if (operation == OperationType::REMOVE || operation == OperationType::DETACH) {
            selectValueLowerLimit = 0.5;
        }

        //Get vertices
        std::vector<Point> points;
        std::unordered_set<FaceId> facesToBlend;
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            double selectValue = internal::averageFaceSelectValue(mesh, fId, *vertexSelectValue[mId]);

            if (selectValue > selectValueLowerLimit && selectValue < 1.0 && !nvl::epsEqual(selectValue, selectValueLowerLimit) && !nvl::epsEqual(selectValue, 1.0)) {
                facesToBlend.insert(fId);

                points.push_back(nvl::meshFaceBarycenter(mesh, fId));
            }
        }

        if (facesToBlend.empty()) {
            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                double selectValue = internal::averageFaceSelectValue(mesh, fId, *vertexSelectValue[mId]);

                if (selectValue > selectValueLowerLimit && !nvl::epsEqual(selectValue, selectValueLowerLimit)) {
                    facesToBlend.insert(fId);

                    points.push_back(nvl::meshFaceBarycenter(mesh, fId));
                }
            }
        }


        //Expand selection
        std::unordered_set<FaceId> newFacesToBlend;

        bool expansion;
        do {
            expansion = false;

            for (const FaceId& fId : facesToBlend) {
                for (const FaceId& adj : meshFFAdj[fId]) {
                    if (adj != nvl::MAX_INDEX) {
                        if (facesToBlend.find(adj) == facesToBlend.end()) {

                            double selectValue = internal::averageFaceSelectValue(mesh, adj, *vertexSelectValue[mId]);
                            if (selectValue > selectValueLowerLimit) {
                                const Point barycenter = nvl::meshFaceBarycenter(mesh, adj);

                                for (const Point& point : points) {
                                    Scalar distance = (barycenter - point).norm();

                                    if (distance < maxExpansionDistance) {
                                        expansion = true;

                                        newFacesToBlend.insert(adj);

                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            facesToBlend.insert(newFacesToBlend.begin(), newFacesToBlend.end());
        } while (expansion);

        //Transfer vertices to keep in the current mesh
        Mesh currentMesh;
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(facesToBlend.begin(), facesToBlend.end()), currentMesh, gridBirthVertex[mId], gridBirthFace[mId]);

        //Scale mesh
        nvl::meshApplyTransformation(currentMesh, scaleTransform);

        //Initialize adapter and polygon grid
        internal::OpenVDBAdapter<Mesh> adapter(&currentMesh);
        polygonGrids[mId] = IntGrid::create(-1);


#ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_input_ " + std::to_string(mId) + ".obj", currentMesh);
#endif

        //Create unsigned distance field
        TransformPtr linearTransform = openvdb::math::Transform::createLinearTransform(1.0);
        unsignedGrids[mId] = openvdb::tools::meshToVolume<FloatGrid>(
                    adapter, *linearTransform, maxDistance, maxDistance, openvdb::tools::MeshToVolumeFlags::UNSIGNED_DISTANCE_FIELD, polygonGrids[mId].get());


        //Create unsigned distance field
        signedGrids[mId] = unsignedGrids[mId]->deepCopy();
        FloatGrid::Accessor signedAccessor = signedGrids[mId]->getAccessor();

        //Eigen mesh conversion
        Mesh triangulatedMesh = currentMesh;
        nvl::meshTriangulateConvexFace(triangulatedMesh);
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        nvl::convertMeshToEigenMesh(triangulatedMesh, V, F);

        //Precompute fast winding number data
        igl::FastWindingNumberBVH fwn;
        igl::fast_winding_number(V.cast<float>().eval(), F, 2, fwn);

        //Minimum and maximum coordinates in the scalar fields
        bbMin[mId] = openvdb::Vec3i(
            std::numeric_limits<int>::max(),
            std::numeric_limits<int>::max(),
            std::numeric_limits<int>::max());
        bbMax[mId] = openvdb::Vec3i(
            std::numeric_limits<int>::min(),
            std::numeric_limits<int>::min(),
            std::numeric_limits<int>::min());

        //Min and max values
        for (FloatGrid::ValueOnIter iter = signedGrids[mId]->beginValueOn(); iter; ++iter) {
            GridCoord coord = iter.getCoord();

            bbMin[mId] = openvdb::Vec3i(
                std::min(coord.x(), bbMin[mId].x()),
                std::min(coord.y(), bbMin[mId].y()),
                std::min(coord.z(), bbMin[mId].z()));

            bbMax[mId] = openvdb::Vec3i(
                std::max(coord.x(), bbMax[mId].x()),
                std::max(coord.y(), bbMax[mId].y()),
                std::max(coord.z(), bbMax[mId].z()));
        }

        //Fast winding number input
        Eigen::MatrixXd Q;
        Eigen::VectorXf W;

        //Resize by number of voxels
        const unsigned int numberVoxels = (bbMax[mId].x() - bbMin[mId].x()) * (bbMax[mId].y() - bbMin[mId].y()) * (bbMax[mId].z() - bbMin[mId].z());
        Q.resize(numberVoxels, 3);

        //Fill fast winding number structure
        unsigned int currentVoxel = 0;
        for (int i = bbMin[mId].x(); i < bbMax[mId].x(); i++) {
            for (int j = bbMin[mId].y(); j < bbMax[mId].y(); j++) {
                for (int k = bbMin[mId].z(); k < bbMax[mId].z(); k++) {
                    GridCoord coord(i,j,k);
                    openvdb::Vec3d p = signedGrids[mId]->indexToWorld(coord);

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
        for (int i = bbMin[mId].x(); i < bbMax[mId].x(); i++) {
            for (int j = bbMin[mId].y(); j < bbMax[mId].y(); j++) {
                for (int k = bbMin[mId].z(); k < bbMax[mId].z(); k++) {
                    GridCoord coord(i,j,k);

                    FloatGrid::ValueType dist = signedAccessor.getValue(coord);
                    //FloatGrid::ValueType dist = openvdb::tools::QuadraticSampler::sample(accessor, p);

                    int fwn = static_cast<int>(std::round(W(currentVoxel)));
                    int sign = (fwn % 2 == 0 ? 1 : -1);

                    assert(dist >= 0);
                    signedAccessor.setValue(coord, sign * dist);

                    currentVoxel++;
                }
            }
        }


        //Create openvdb mesh
        Mesh closedMesh = convertGridToMesh<Mesh>(signedGrids[mId], true);
        internal::OpenVDBAdapter<Mesh> closedAdapter(&closedMesh);

        closedGrids[mId] = openvdb::tools::meshToVolume<FloatGrid>(
            closedAdapter, *linearTransform, maxDistance, maxDistance, 0);

#ifdef SAVE_MESHES_FOR_DEBUG
        nvl::meshSaveToFile("results/action_closed_ " + std::to_string(mId) + ".obj", closedMesh);
#endif
    }
}

template<class Model>
typename Model::Mesh getBlendedMesh(
        const OperationType operation,
        const std::vector<const Model*>& models,
        const std::vector<const std::vector<double>*>& vertexSelectValue,
        const double& scaleFactor,
        const double& maxDistance,
        const std::vector<FloatGridPtr>& closedGrids,
        const std::vector<IntGridPtr>& polygonGrids,
        const std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace,
        openvdb::Vec3i& minCoord,
        const openvdb::Vec3i& maxCoord)
{
    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Point Point;

    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);

    Mesh blendedMesh;

    FloatGridPtr blendedGrid = FloatGrid::create(maxDistance);
    FloatGrid::Accessor resultAccessor = blendedGrid->getAccessor();

    //Blend grids
    for (int i = minCoord.x(); i < maxCoord.x(); i++) {
        for (int j = minCoord.y(); j < maxCoord.y(); j++) {
            for (int k = minCoord.z(); k < maxCoord.z(); k++) {
                GridCoord coord(i,j,k);

                Point point(i, j, k);

                FloatGrid::ValueType resultValue = 0.0;

                FloatGrid::ConstAccessor closedAccessor1 = closedGrids[0]->getConstAccessor();
                IntGrid::ConstAccessor polygonAccessor1 = polygonGrids[0]->getConstAccessor();
                FloatGrid::ValueType closedDistance1 = closedAccessor1.getValue(coord);
                IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);
                const Mesh& mesh1 = models[0]->mesh;

                if (operation == OperationType::REMOVE || operation == OperationType::DETACH) {
                    resultValue = closedDistance1;
                }
                else if (operation == OperationType::REPLACE) {
                    FloatGrid::ConstAccessor closedAccessor2 = closedGrids[1]->getConstAccessor();
                    IntGrid::ConstAccessor polygonAccessor2 = polygonGrids[1]->getConstAccessor();
                    FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);
                    IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);
                    const Mesh& mesh2 = models[1]->mesh;

                    if (pId1 >= 0 && pId2 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        FaceId originFaceId1 = gridBirthFace[0][pId1];
                        double selectValue1 = internal::interpolateFaceSelectValue(mesh1, originFaceId1, point, *vertexSelectValue[0]);

                        FaceId originFaceId2 = gridBirthFace[1][pId2];
                        double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, *vertexSelectValue[1]);

                        if (selectValue1 >= SELECT_VALUE_MAX_THRESHOLD && selectValue2 < SELECT_VALUE_MAX_THRESHOLD) {
                            resultValue = closedDistance1;
                        }
                        else if (selectValue1 < SELECT_VALUE_MAX_THRESHOLD && selectValue2 >= SELECT_VALUE_MAX_THRESHOLD) {
                            resultValue = closedDistance2;
                        }
                        else if (selectValue1 <= SELECT_VALUE_MIN_THRESHOLD && selectValue2 <= SELECT_VALUE_MIN_THRESHOLD) {
                            resultValue = 0.5 * closedDistance1 + 0.5 * closedDistance2;
                        }
                        else {
                            resultValue =
                                    (selectValue1 * closedDistance1 + selectValue2 * closedDistance2) /
                                    (selectValue1 + selectValue2);
                        }
                    }
                    else if (pId1 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance) {
                        resultValue = closedDistance1;
                    }
                    else if (pId2 >= 0 && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                        resultValue = closedDistance2;
                    }
                    else {
                        resultValue = closedDistance1 < 0 && closedDistance2 < 0 ? -maxDistance : maxDistance;
                    }
                }

                resultAccessor.setValue(coord, resultValue);
            }
        }
    }

    //Convert to mesh
    blendedMesh = convertGridToMesh<Mesh>(blendedGrid, true);

#ifdef SAVE_MESHES_FOR_DEBUG
    nvl::meshSaveToFile("results/action_blendedMesh_non_rescaled.obj", blendedMesh);
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
double interpolateFaceSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<double>& vertexSelectValue)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
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
double averageFaceSelectValue(
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
