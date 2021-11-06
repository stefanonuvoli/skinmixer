#include "skinmixer_field.h"

#ifdef SKINMIXER_DEBUG_SAVE_MESHES
#include <nvl/models/mesh_io.h>
#endif

#include <nvl/models/mesh_transfer.h>
#include <nvl/models/mesh_transformations.h>
#include <nvl/models/mesh_triangulation.h>
#include <nvl/models/mesh_geometric_information.h>
#include <nvl/models/mesh_eigen_convert.h>

#include <nvl/math/barycentric_interpolation.h>
#include <nvl/math/numeric_limits.h>
#include <nvl/math/comparisons.h>

#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/Composite.h>

#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>

#define SELECT_VALUE_MIN_THRESHOLD 0.001
#define SELECT_VALUE_MAX_THRESHOLD 0.999
#define EXPANSION_VOXELS 10.0

namespace skinmixer {
namespace internal {

template<class Mesh>
void getClosedGrid(
        const Mesh& inputMesh,
        const double& scaleFactor,
        const double& maxDistance,
        Mesh& closedMesh,
        openvdb::FloatGrid::Ptr& closedGrid,
        openvdb::Int32Grid::Ptr& polygonGrid,
        openvdb::math::Coord& bbMin,
        openvdb::math::Coord& bbMax)
{
    typedef typename Mesh::Point Point;    

    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename openvdb::math::Coord GridCoord;
    typedef typename openvdb::Vec3R GridVec;
    typedef typename openvdb::math::Transform GridTransform;
    typedef typename GridTransform::Ptr GridTransformPtr;

    const double maxVoxelDistance = maxDistance / scaleFactor + nvl::EPSILON;

    //Initialize adapter and polygon grid
    OpenVDBAdapter<Mesh> adapter(inputMesh, scaleFactor);
    polygonGrid = IntGrid::create(-1);

    //Create unsigned distance field
    GridTransformPtr linearTransform = GridTransform::createLinearTransform(scaleFactor);
    FloatGridPtr signedGrid = openvdb::tools::meshToVolume<FloatGrid>(
                adapter, *linearTransform, maxVoxelDistance, maxVoxelDistance, openvdb::tools::MeshToVolumeFlags::UNSIGNED_DISTANCE_FIELD, polygonGrid.get());

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
    bbMin = GridCoord(
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max());
    bbMax = GridCoord(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min());

    //Min and max values
    for (FloatGrid::ValueOnIter iter = signedGrid->beginValueOn(); iter; ++iter) {
        GridCoord coord = iter.getCoord();

        bbMin = GridCoord(
            std::min(coord.x(), bbMin.x()),
            std::min(coord.y(), bbMin.y()),
            std::min(coord.z(), bbMin.z()));

        bbMax = GridCoord(
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
                GridVec vdbPoint = signedGrid->indexToWorld(coord);

                Q(currentVoxel, 0) = vdbPoint.x();
                Q(currentVoxel, 1) = vdbPoint.y();
                Q(currentVoxel, 2) = vdbPoint.z();

                currentVoxel++;
            }
        }
    }
    //Calculate fast winding number
    igl::fast_winding_number(fwn, 2, Q.cast<float>().eval(), W);

    //Set the sign
    currentVoxel = 0;

    FloatGrid::Accessor signedAccessor = signedGrid->getAccessor();
    for (int i = bbMin.x(); i < bbMax.x(); i++) {
        for (int j = bbMin.y(); j < bbMax.y(); j++) {
            for (int k = bbMin.z(); k < bbMax.z(); k++) {
                GridCoord coord(i,j,k);

                int fwn = static_cast<int>(std::round(W(currentVoxel)));

                if (fwn % 2 != 0) {
                    FloatGrid::ValueType dist = signedAccessor.getValue(coord);
                    assert(dist >= 0);

                    signedAccessor.setValue(coord, -dist);
                }

                currentVoxel++;
            }
        }
    }

    //Create openvdb mesh
    closedMesh = convertGridToMesh<Mesh>(signedGrid, true);

    signedGrid->clear();
    signedGrid.reset();

    OpenVDBAdapter<Mesh> closedAdapter(closedMesh, scaleFactor);

    closedGrid = openvdb::tools::meshToVolume<FloatGrid>(
        closedAdapter, *linearTransform, maxVoxelDistance, maxVoxelDistance, 0);
}

template<class Model>
void getBlendedGrid(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
        const std::unordered_map<nvl::Index, nvl::Index>& clusterMap,
        const std::vector<nvl::Index>& actions,
        const std::vector<const Model*>& models,
        const std::vector<std::vector<double>>& vertexSelectValues,
        const std::vector<openvdb::FloatGrid::Ptr>& closedGrids,
        const std::vector<openvdb::Int32Grid::Ptr>& polygonGrids,
        const std::vector<std::vector<typename Model::Mesh::FaceId>>& fieldBirthFace,
        const std::vector<openvdb::math::Coord>& bbMin,
        const std::vector<openvdb::math::Coord>& bbMax,
        const double& scaleFactor,
        const double& maxDistance,
        openvdb::FloatGrid::Ptr& blendedGrid,
        openvdb::Int32Grid::Ptr& activeActionGrid)
{
    typedef nvl::Index Index;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Point Point;
    typedef typename SkinMixerData<Model>::Action Action;

    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename openvdb::math::Coord GridCoord;
    typedef typename openvdb::Vec3R GridVec;
    typedef typename openvdb::math::Transform GridTransform;
    typedef typename GridTransform::Ptr GridTransformPtr;

    //Minimum and maximum coordinates in the scalar fields
    GridCoord minCoord(
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max());
    GridCoord maxCoord(
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min(),
        std::numeric_limits<int>::min());

    for (nvl::Index mId = 0; mId < cluster.size(); ++mId) {
        minCoord = GridCoord(
            std::min(minCoord.x(), bbMin[mId].x()),
            std::min(minCoord.y(), bbMin[mId].y()),
            std::min(minCoord.z(), bbMin[mId].z()));

        maxCoord = GridCoord(
            std::max(maxCoord.x(), bbMax[mId].x()),
            std::max(maxCoord.y(), bbMax[mId].y()),
            std::max(maxCoord.z(), bbMax[mId].z()));
    }

    Mesh blendedMesh;


    blendedGrid = FloatGrid::create(maxDistance);
    GridTransformPtr linearTransform = GridTransform::createLinearTransform(scaleFactor);
    blendedGrid->setTransform(linearTransform);

    activeActionGrid = IntGrid::create(-1);

    FloatGrid::Accessor blendedAccessor = blendedGrid->getAccessor();
    IntGrid::Accessor actionAccessor = activeActionGrid->getAccessor();

    std::vector<IntGrid::ConstAccessor> polygonAccessors;
    std::vector<FloatGrid::ConstAccessor> closedAccessors;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        polygonAccessors.push_back(polygonGrids[cId]->getConstAccessor());
        closedAccessors.push_back(closedGrids[cId]->getConstAccessor());
    }

    //Blend grids
    for (int i = minCoord.x(); i < maxCoord.x(); i++) {
        for (int j = minCoord.y(); j < maxCoord.y(); j++) {
            for (int k = minCoord.z(); k < maxCoord.z(); k++) {
                GridCoord coord(i, j, k);
                GridVec openvdbPoint = blendedGrid->indexToWorld(coord);
                Point point(openvdbPoint.x(), openvdbPoint.y(), openvdbPoint.z());

                FloatGrid::ValueType resultValue = maxDistance;

                //Find best action
                double bestScore = nvl::maxLimitValue<double>();
                IntGrid::ValueType bestActionId = -1;

                for (Index aId = 0; aId < actions.size(); ++aId) {
                    const Index& actionId = actions[aId];
                    const Action& action = data.action(actionId);

                    const Index eId1 = action.entry1;
                    assert(eId1 != nvl::MAX_INDEX);
                    const Index cId1 = clusterMap.at(eId1);

                    const IntGrid::ValueType pId1 = polygonAccessors[cId1].getValue(coord);

                    double actionScore = nvl::maxLimitValue<double>();
                    if (pId1 >= 0) {
                        const FloatGrid::ValueType closedDistance1 = closedAccessors[cId1].getValue(coord);

                        const std::vector<FaceId>& fieldBirthFace1 = fieldBirthFace[cId1];

                        const Mesh& mesh1 = models[cId1]->mesh;

                        FaceId originFaceId1 = fieldBirthFace1[pId1];
                        double actionSelectValue1 = interpolateFaceSelectValue(mesh1, originFaceId1, point, action.select1.vertex);

                        double score1 = 0.1 * (std::fabs(closedDistance1) / maxDistance) +
                                0.9 * (std::fabs(0.5 - actionSelectValue1) * 2.0);

                        const Index eId2 = action.entry2;
                        if (eId2 != nvl::MAX_INDEX) {
                            const Index cId2 = clusterMap.at(eId2);

                            const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(coord);

                            if (pId2 >= 0) {
                                const FloatGrid::ValueType closedDistance2 = closedAccessors[cId2].getValue(coord);

                                const std::vector<FaceId>& fieldBirthFace2 = fieldBirthFace[cId2];
                                const Mesh& mesh2 = models[cId2]->mesh;

                                FaceId originFaceId2 = fieldBirthFace2[pId2];
                                double actionSelectValue2 = interpolateFaceSelectValue(mesh2, originFaceId2, point, action.select2.vertex);

                                double score2 = 0.1 * (std::fabs(closedDistance2) / maxDistance) +
                                        0.9 * (std::fabs(0.5 - actionSelectValue2) * 2.0);

                                actionScore = (score1 + score2) / 2.0;
                            }
                        }
                        else {
                            actionScore = score1;
                        }
                    }

                    if (actionScore <= bestScore) {
                        bestScore = actionScore;
                        bestActionId = static_cast<IntGrid::ValueType>(aId);
                    }
                }

                assert(bestActionId >= 0);

                const Index& actionId = actions[bestActionId];
                const Action& action = data.action(actionId);

                const Index eId1 = action.entry1;
                assert(eId1 != nvl::MAX_INDEX);
                const Index cId1 = clusterMap.at(eId1);

                const FloatGrid::ValueType closedDistance1 = closedAccessors[cId1].getValue(coord);

                if (action.operation == OperationType::REMOVE || action.operation == OperationType::DETACH) {
                    resultValue = closedDistance1;
                }
                else if (action.operation == OperationType::REPLACE || action.operation == OperationType::ATTACH) {
                    const IntGrid::ValueType pId1 = polygonAccessors[cId1].getValue(coord);


                    const std::vector<double>& vertexSelectValues1 = vertexSelectValues[cId1];
                    const std::vector<FaceId>& fieldBirthFace1 = fieldBirthFace[cId1];

                    const Mesh& mesh1 = models[cId1]->mesh;

                    const Index eId2 = action.entry2;
                    assert(eId2 != nvl::MAX_INDEX);
                    const Index cId2 = clusterMap.at(eId2);


                    const FloatGrid::ValueType closedDistance2 = closedAccessors[cId2].getValue(coord);
                    const IntGrid::ValueType pId2 = polygonAccessors[cId2].getValue(coord);

                    const std::vector<double>& vertexSelectValues2 = vertexSelectValues[cId2];
                    const std::vector<FaceId>& fieldBirthFace2 = fieldBirthFace[cId2];

                    const Mesh& mesh2 = models[cId2]->mesh;

                    if (pId1 >= 0 && pId2 >= 0 && closedDistance1 < maxDistance && closedDistance1 > -maxDistance && closedDistance2 < maxDistance && closedDistance2 > -maxDistance) {

                        //Replace operation
                        if (action.operation == OperationType::REPLACE) {
                            //Blend mode
                            if (action.replaceMode == ReplaceMode::BLEND) {
                                FaceId originFaceId1 = fieldBirthFace1[pId1];
                                double selectValue1 = interpolateFaceSelectValue(mesh1, originFaceId1, point, vertexSelectValues1);

                                FaceId originFaceId2 = fieldBirthFace2[pId2];
                                double selectValue2 = interpolateFaceSelectValue(mesh2, originFaceId2, point, vertexSelectValues2);

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
                            //Union mode
                            else {
                                assert(action.replaceMode == ReplaceMode::UNION);
                                resultValue = nvl::min(closedDistance1, closedDistance2);
                            }
                        }

                        //Attach operation
                        else if (action.operation == OperationType::ATTACH) {
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

                blendedAccessor.setValue(coord, resultValue);
                actionAccessor.setValue(coord, bestActionId);
            }
        }
    }
}

template<class Mesh>
std::unordered_set<typename Mesh::FaceId> findFieldFaces(
        const Mesh& mesh,
        const std::vector<double>& vertexSelectValues,
        const double& scaleFactor)
{
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::FaceId FaceId;
    typedef typename nvl::Index Index;

    std::unordered_set<typename Mesh::FaceId> fieldFaces;

    std::vector<std::vector<FaceId>> meshFFAdj = nvl::meshFaceFaceAdjacencies(mesh);
    std::vector<std::vector<FaceId>> meshCC = nvl::meshConnectedComponents(mesh, meshFFAdj);

    Scalar maxExpansionDistance = EXPANSION_VOXELS * scaleFactor;
    if (nvl::epsEqual(maxExpansionDistance, 0.0)) {
        for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            double selectValue = averageFaceSelectValue(mesh, fId, vertexSelectValues);

            if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0) && selectValue < 1.0 && !nvl::epsEqual(selectValue, 1.0)) {
                fieldFaces.insert(fId);
            }
        }

        if (fieldFaces.empty()) {
            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                double selectValue = averageFaceSelectValue(mesh, fId, vertexSelectValues);

                if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0)) {
                    fieldFaces.insert(fId);
                }
            }
        }
    }
    else {
        std::vector<std::vector<FaceId>> meshFFAdj = nvl::meshFaceFaceAdjacencies(mesh);
        std::vector<std::vector<FaceId>> meshCC = nvl::meshConnectedComponents(mesh, meshFFAdj);

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

            double selectValue = averageFaceSelectValue(mesh, fId, vertexSelectValues);

            if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0) && selectValue < 1.0 && !nvl::epsEqual(selectValue, 1.0)) {
                fieldFaces.insert(fId);

                points.push_back(nvl::meshFaceBarycenter(mesh, fId));
            }
        }

        if (fieldFaces.empty()) {
            for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
                if (mesh.isFaceDeleted(fId)) {
                    continue;
                }

                double selectValue = averageFaceSelectValue(mesh, fId, vertexSelectValues);

                if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0)) {
                    fieldFaces.insert(fId);

                    points.push_back(nvl::meshFaceBarycenter(mesh, fId));
                }
            }
        }

        //Expand selection
        std::unordered_set<FaceId> newfieldFaces;

        bool expansion;
        do {
            expansion = false;

            for (const FaceId& fId : fieldFaces) {
                for (const FaceId& adj : meshFFAdj[fId]) {
                    if (adj != nvl::MAX_INDEX) {
                        if (fieldFaces.find(adj) == fieldFaces.end()) {

                            double selectValue = averageFaceSelectValue(mesh, adj, vertexSelectValues);
                            if (selectValue > 0.0 && !nvl::epsEqual(selectValue, 0.0)) {
                                const Point barycenter = nvl::meshFaceBarycenter(mesh, adj);

                                for (const Point& point : points) {
                                    Scalar distance = (barycenter - point).norm();

                                    if (distance < maxExpansionDistance) {
                                        expansion = true;

                                        newfieldFaces.insert(adj);

                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            fieldFaces.insert(newfieldFaces.begin(), newfieldFaces.end());
        } while (expansion);
    }

    return fieldFaces;
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

    if (nvl::epsEqual(value, 0.0)) {
        value = 0.0;
    }
    else if (nvl::epsEqual(value, 1.0)) {
        value = 1.0;
    }

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

    if (nvl::epsEqual(selectValue, 0.0)) {
        selectValue = 0.0;
    }
    else if (nvl::epsEqual(selectValue, 1.0)) {
        selectValue = 1.0;
    }

    return selectValue;
}

}
}
