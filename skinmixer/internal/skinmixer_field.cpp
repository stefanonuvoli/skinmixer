#include "skinmixer_field.h"

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

#define SELECT_VALUE_MIN_THRESHOLD 0.01
#define SELECT_VALUE_MAX_THRESHOLD 0.99
#define EXPANSION_VOXELS 20.0

namespace skinmixer {
namespace internal {

template<class Mesh>
void getClosedGrid(
        const Mesh& inputMesh,
        const double& maxDistance,
        Mesh& closedMesh,
        FloatGridPtr& closedGrid,
        IntGridPtr& polygonGrid,
        openvdb::Vec3i& bbMin,
        openvdb::Vec3i& bbMax)
{
    typedef typename Mesh::Point Point;
    typedef typename openvdb::math::Transform::Ptr TransformPtr;

    //Initialize adapter and polygon grid
    internal::OpenVDBAdapter<Mesh> adapter(&inputMesh);
    polygonGrid = IntGrid::create(-1);

    //Create unsigned distance field
    TransformPtr linearTransform = openvdb::math::Transform::createLinearTransform(1.0);
    FloatGridPtr signedGrid = openvdb::tools::meshToVolume<FloatGrid>(
                adapter, *linearTransform, maxDistance, maxDistance, openvdb::tools::MeshToVolumeFlags::UNSIGNED_DISTANCE_FIELD, polygonGrid.get());

    FloatGrid::Accessor signedAccessor = signedGrid->getAccessor();

    //Eigen mesh conversion
    Mesh triangulatedMesh = inputMesh;
    nvl::meshTriangulateConvexFace(triangulatedMesh);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    nvl::convertMeshToEigenMesh(triangulatedMesh, V, F);

    //Precompute fast winding number data
    igl::FastWindingNumberBVH fwn;
    igl::fast_winding_number(V.cast<float>().eval(), F, 2, fwn);

    //Minimum and maximum coordinates in the scalar fields
    bbMin = openvdb::Vec3i(
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max());
    bbMax = openvdb::Vec3i(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min());

    //Min and max values
    for (FloatGrid::ValueOnIter iter = signedGrid->beginValueOn(); iter; ++iter) {
        GridCoord coord = iter.getCoord();

        bbMin = openvdb::Vec3i(
            std::min(coord.x(), bbMin.x()),
            std::min(coord.y(), bbMin.y()),
            std::min(coord.z(), bbMin.z()));

        bbMax = openvdb::Vec3i(
            std::max(coord.x(), bbMax.x()),
            std::max(coord.y(), bbMax.y()),
            std::max(coord.z(), bbMax.z()));
    }

    //Fast winding number input
    Eigen::MatrixXd Q;
    Eigen::VectorXf W;

    //Resize by number of voxels
    const unsigned int numberVoxels = (bbMax.x() - bbMin.x()) * (bbMax.y() - bbMin.y()) * (bbMax.z() - bbMin.z());
    Q.resize(numberVoxels, 3);

    //Fill fast winding number structure
    unsigned int currentVoxel = 0;
    for (int i = bbMin.x(); i < bbMax.x(); i++) {
        for (int j = bbMin.y(); j < bbMax.y(); j++) {
            for (int k = bbMin.z(); k < bbMax.z(); k++) {
                GridCoord coord(i,j,k);
                openvdb::Vec3d p = signedGrid->indexToWorld(coord);

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
    for (int i = bbMin.x(); i < bbMax.x(); i++) {
        for (int j = bbMin.y(); j < bbMax.y(); j++) {
            for (int k = bbMin.z(); k < bbMax.z(); k++) {
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
    closedMesh = convertGridToMesh<Mesh>(signedGrid, true);

    signedGrid->clear();
    signedGrid.reset();

    internal::OpenVDBAdapter<Mesh> closedAdapter(&closedMesh);

    closedGrid = openvdb::tools::meshToVolume<FloatGrid>(
        closedAdapter, *linearTransform, maxDistance, maxDistance, 0);
}

template<class Model>
void getClosedGrids(
        const OperationType operation,
        const std::vector<const Model*>& models,
        const std::vector<const std::vector<double>*>& vertexSelectValue,
        const double& scaleFactor,
        const double& maxDistance,
        std::vector<typename Model::Mesh>& inputMeshes,
        std::vector<typename Model::Mesh>& closedMeshes,
        std::vector<FloatGridPtr>& closedGrids,
        std::vector<IntGridPtr>& polygonGrids,
        std::vector<std::unordered_set<typename Model::Mesh::FaceId>>& facesInField,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& gridBirthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace,
        std::vector<openvdb::Vec3i>& bbMin,
        std::vector<openvdb::Vec3i>& bbMax)
{
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::FaceId FaceId;
    typedef typename nvl::Index Index;

    inputMeshes.resize(models.size());
    closedMeshes.resize(models.size());
    polygonGrids.resize(models.size());
    closedGrids.resize(models.size());
    facesInField.resize(models.size());
    gridBirthVertex.resize(models.size());
    gridBirthFace.resize(models.size());
    bbMin.resize(models.size());
    bbMax.resize(models.size());

    for (Index mId = 0; mId < models.size(); ++mId) {
        const Model* model = models[mId];
        const Mesh& mesh = model->mesh;

        facesInField[mId] = findFacesInField(mesh, *vertexSelectValue[mId], scaleFactor);

        //Transfer vertices to keep in the current mesh
        nvl::meshTransferFaces(mesh, std::vector<FaceId>(facesInField[mId].begin(), facesInField[mId].end()), inputMeshes[mId], gridBirthVertex[mId], gridBirthFace[mId]);

        //Scale mesh
        const nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);
        nvl::meshApplyTransformation(inputMeshes[mId], scaleTransform);

        getClosedGrid(inputMeshes[mId], maxDistance, closedMeshes[mId], closedGrids[mId], polygonGrids[mId], bbMin[mId], bbMax[mId]);
    }
}

template<class Model>
void getBlendedGrid(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& actions,
        const std::vector<std::vector<const Model*>>& models,
        const std::vector<std::vector<const std::vector<double>*>>& vertexSelectValues,
        const std::vector<std::vector<FloatGridPtr>>& closedGrids,
        const std::vector<std::vector<IntGridPtr>>& polygonGrids,
        const std::vector<std::vector<std::vector<typename Model::Mesh::FaceId>>>& gridBirthFace,
        const std::vector<std::vector<openvdb::Vec3i>>& bbMin,
        const std::vector<std::vector<openvdb::Vec3i>>& bbMax,
        const double& scaleFactor,
        const double& maxDistance,
        FloatGridPtr& blendedGrid,
        IntGridPtr& activeActionGrid)
{
    typedef nvl::Index Index;
    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Point Point;
    typedef typename SkinMixerData<Model>::Action Action;


    //Minimum and maximum coordinates in the scalar fields
    openvdb::Vec3i minCoord(
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max());
    openvdb::Vec3i maxCoord(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min());

    for (Index aId = 0; aId < actions.size(); ++aId) {
        for (nvl::Index mId = 0; mId < models.size(); ++mId) {
            minCoord = openvdb::Vec3i(
                        std::min(minCoord.x(), bbMin[aId][mId].x()),
                        std::min(minCoord.y(), bbMin[aId][mId].y()),
                        std::min(minCoord.z(), bbMin[aId][mId].z()));

            maxCoord = openvdb::Vec3i(
                        std::max(maxCoord.x(), bbMax[aId][mId].x()),
                        std::max(maxCoord.y(), bbMax[aId][mId].y()),
                        std::max(maxCoord.z(), bbMax[aId][mId].z()));
        }
    }


    nvl::Scaling3d scaleTransform(scaleFactor, scaleFactor, scaleFactor);

    Mesh blendedMesh;

    blendedGrid = FloatGrid::create(maxDistance);
    FloatGrid::Accessor blendedAccessor = blendedGrid->getAccessor();

    activeActionGrid = IntGrid::create(nvl::maxLimitValue<IntGrid::ValueType>());
    IntGrid::Accessor actionAccessor = activeActionGrid->getAccessor();

    //Blend grids
    for (int i = minCoord.x(); i < maxCoord.x(); i++) {
        for (int j = minCoord.y(); j < maxCoord.y(); j++) {
            for (int k = minCoord.z(); k < maxCoord.z(); k++) {
                GridCoord coord(i, j, k);
                Point point(i, j, k);

                FloatGrid::ValueType resultValue = maxDistance;

                //Find best action
                double bestScore = nvl::maxLimitValue<double>();
                IntGrid::ValueType bestActionId = nvl::maxLimitValue<IntGrid::ValueType>();

                for (Index aId = 0; aId < actions.size(); ++aId) {
                    const std::vector<const Model*>& actionModels = models[aId];
                    const std::vector<const std::vector<double>*>& actionVertexSelectValues = vertexSelectValues[aId];
                    const std::vector<internal::IntGridPtr>& actionPolygonGrids = polygonGrids[aId];
                    const std::vector<internal::FloatGridPtr>& actionClosedGrids = closedGrids[aId];
                    const std::vector<std::vector<FaceId>>& actionGridBirthFace = gridBirthFace[aId];

                    double currentScore = 0.0;
                    int numValues = 0;

                    for (Index mId = 0; mId < actionModels.size(); ++mId) {
                        const Mesh& mesh = actionModels[mId]->mesh;

                        internal::FloatGrid::ConstAccessor closedAccessor = actionClosedGrids[mId]->getConstAccessor();
                        internal::FloatGrid::ValueType closedDistance = closedAccessor.getValue(coord);

                        IntGrid::ConstAccessor polygonAccessor = actionPolygonGrids[mId]->getConstAccessor();
                        IntGrid::ValueType pId = polygonAccessor.getValue(coord);

                        if (closedDistance > -maxDistance && closedDistance < maxDistance && pId >= 0) {
                            FaceId originFaceId = actionGridBirthFace[mId][pId];
                            double selectValue = internal::interpolateFaceSelectValue(mesh, originFaceId, point, *actionVertexSelectValues[mId]);

                            currentScore += 1.0 * (std::fabs(closedDistance) / maxDistance) + (std::fabs(0.5 - selectValue) * 0.0);
                            numValues++;
                        }
                    }

                    if (numValues > 0) {
                        currentScore /= numValues;

                        if (currentScore < bestScore) {
                            bestScore = currentScore;
                            bestActionId = static_cast<IntGrid::ValueType>(aId);
                        }
                    }
                }

                if (bestActionId < nvl::maxLimitValue<IntGrid::ValueType>()) {
                    //Action data
                    const Index& actionId = actions[bestActionId];
                    const Action& action = data.action(actionId);

                    const std::vector<const Model*>& actionModels = models[bestActionId];
                    const std::vector<const std::vector<double>*>& actionVertexSelectValues = vertexSelectValues[bestActionId];
                    const std::vector<internal::IntGridPtr>& actionPolygonGrids = polygonGrids[bestActionId];
                    const std::vector<internal::FloatGridPtr>& actionClosedGrids = closedGrids[bestActionId];
                    const std::vector<std::vector<FaceId>>& actionGridBirthFace = gridBirthFace[bestActionId];


                    //Data for first model
                    const Mesh& mesh1 = actionModels[0]->mesh;
                    FloatGrid::ConstAccessor closedAccessor1 = actionClosedGrids[0]->getConstAccessor();
                    IntGrid::ConstAccessor polygonAccessor1 = actionPolygonGrids[0]->getConstAccessor();

                    FloatGrid::ValueType closedDistance1 = closedAccessor1.getValue(coord);
                    IntGrid::ValueType pId1 = polygonAccessor1.getValue(coord);
                    if (action.operation == OperationType::REMOVE || action.operation == OperationType::DETACH) {
                        resultValue = closedDistance1;
                    }
                    else if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
                        const Mesh& mesh2 = actionModels[1]->mesh;
                        FloatGrid::ConstAccessor closedAccessor2 = actionClosedGrids[1]->getConstAccessor();
                        IntGrid::ConstAccessor polygonAccessor2 = actionPolygonGrids[1]->getConstAccessor();

                        FloatGrid::ValueType closedDistance2 = closedAccessor2.getValue(coord);
                        IntGrid::ValueType pId2 = polygonAccessor2.getValue(coord);

                        if (pId1 >= 0 && pId2 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                            FaceId originFaceId1 = actionGridBirthFace[0][pId1];
                            double selectValue1 = internal::interpolateFaceSelectValue(mesh1, originFaceId1, point, *actionVertexSelectValues[0]);

                            FaceId originFaceId2 = actionGridBirthFace[1][pId2];
                            double selectValue2 = internal::interpolateFaceSelectValue(mesh2, originFaceId2, point, *actionVertexSelectValues[1]);

                            //Replace operation blending
                            if (action.operation == OperationType::REPLACE) {
                                if (selectValue1 >= SELECT_VALUE_MAX_THRESHOLD && selectValue2 < SELECT_VALUE_MAX_THRESHOLD) {
                                    resultValue = closedDistance1;
                                }
                                else if (selectValue1 < SELECT_VALUE_MAX_THRESHOLD && selectValue2 >= SELECT_VALUE_MAX_THRESHOLD) {
                                    resultValue = closedDistance2;
                                }
                                else if (selectValue1 <= SELECT_VALUE_MIN_THRESHOLD && selectValue2 <= SELECT_VALUE_MIN_THRESHOLD) {
                                    resultValue = 0.5 * closedDistance1 + 0.5 * closedDistance2;
                                }
                                else if (selectValue1 >= SELECT_VALUE_MAX_THRESHOLD && selectValue2 >= SELECT_VALUE_MAX_THRESHOLD) {
                                    resultValue = std::min(closedDistance1, closedDistance2);
                                }
                                else {
                                    resultValue =
                                        (selectValue1 * closedDistance1 + selectValue2 * closedDistance2) /
                                        (selectValue1 + selectValue2);
                                }
                            }

                            //Attach operation blending
                            else if (action.operation == OperationType::ATTACH) {
//                                if (selectValue1 >= SELECT_VALUE_MAX_THRESHOLD && selectValue2 < 0.5) {
//                                    resultValue = closedDistance1;
//                                }
//                                else {
//                                    resultValue = nvl::min(closedDistance1, closedDistance2);
//                                }

                                resultValue = nvl::min(closedDistance1, closedDistance2);
                            }
                        }
                        else if (pId1 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance) {
                            resultValue = closedDistance1;
                        }
                        else if (pId2 >= 0 && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {
                            resultValue = closedDistance2;
                        }
                        else {
                            resultValue = closedDistance1 > 0 || closedDistance2 > 0 ? maxDistance : -maxDistance;
                        }
                    }
                }

                blendedAccessor.setValue(coord, resultValue);
                actionAccessor.setValue(coord, bestActionId);
            }
        }
    }
}

template<class Mesh>
std::unordered_set<typename Mesh::FaceId> findFacesInField(
        const Mesh& mesh,
        const std::vector<double>& vertexSelectValues,
        const double& scaleFactor)
{
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::FaceId FaceId;
    typedef typename nvl::Index Index;

    std::unordered_set<typename Mesh::FaceId> facesInField;

    std::vector<std::vector<FaceId>> meshFFAdj = nvl::meshFaceFaceAdjacencies(mesh);
    std::vector<std::vector<FaceId>> meshCC = nvl::meshConnectedComponents(mesh, meshFFAdj);

    Scalar maxExpansionDistance = EXPANSION_VOXELS / scaleFactor;
    if (maxExpansionDistance == 0) {
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            double selectValue = averageFaceSelectValue(mesh, fId, vertexSelectValues);

            if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0) && selectValue < 1.0 && !nvl::epsEqual(selectValue, 1.0)) {
                facesInField.insert(fId);
            }
        }
    }
    else {
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

        //Get vertices
        std::vector<Point> points;
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValues);

            if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0) && selectValue < 1.0 && !nvl::epsEqual(selectValue, 1.0)) {
                facesInField.insert(fId);

                points.push_back(nvl::meshFaceBarycenter(mesh, fId));
            }
        }

        if (facesInField.empty()) {
            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                double selectValue = internal::averageFaceSelectValue(mesh, fId, vertexSelectValues);

                if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0)) {
                    facesInField.insert(fId);

                    points.push_back(nvl::meshFaceBarycenter(mesh, fId));
                }
            }
        }

        //Expand selection
        std::unordered_set<FaceId> newFacesInField;

        bool expansion;
        do {
            expansion = false;

            for (const FaceId& fId : facesInField) {
                for (const FaceId& adj : meshFFAdj[fId]) {
                    if (adj != nvl::MAX_INDEX) {
                        if (facesInField.find(adj) == facesInField.end()) {

                            double selectValue = internal::averageFaceSelectValue(mesh, adj, vertexSelectValues);
                            if (selectValue > 0.0) {
                                const Point barycenter = nvl::meshFaceBarycenter(mesh, adj);

                                for (const Point& point : points) {
                                    Scalar distance = (barycenter - point).norm();

                                    if (distance < maxExpansionDistance) {
                                        expansion = true;

                                        newFacesInField.insert(adj);

                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            facesInField.insert(newFacesInField.begin(), newFacesInField.end());
        } while (expansion);
    }

    return facesInField;
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
