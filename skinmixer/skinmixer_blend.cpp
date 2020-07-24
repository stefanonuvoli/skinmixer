#include "skinmixer_blend.h"

#ifdef foreach
  #undef foreach
#endif
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

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

#include <nvl/vcglib/vcg_collapse_borders.h>

#include <igl/fast_winding_number.h>
#include <igl/winding_number.h>

//TODO DELETE
#include <nvl/io/mesh_io.h>

#define KEEP_THRESHOLD 0.99
#define DISCARD_THRESHOLD 0.01
#define MIN_CHAIN 8

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

template<class Mesh>
void attachMeshToMeshByBorders(
        Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& meshNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices);

}

template<class Mesh>
void blendMeshes(
        const std::vector<Mesh*>& meshes,
        const std::vector<std::vector<float>>& vertexFuzzyValue,
        Mesh& preMesh,
        Mesh& newMesh,
        std::vector<typename Mesh::VertexId>& preBirthVertices,
        std::vector<typename Mesh::FaceId>& preBirthFaces,
        std::vector<nvl::Index>& preVerticesBirthModel,
        std::vector<nvl::Index>& preFacesBirthModel)
{
    typedef typename openvdb::FloatGrid FloatGrid;
    typedef typename FloatGrid::Ptr FloatGridPtr;
    typedef typename openvdb::Int32Grid IntGrid;
    typedef typename IntGrid::Ptr IntGridPtr;
    typedef typename openvdb::math::Transform::Ptr TransformPtr;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename nvl::Index Index;

    double maxDistance = 100.0;
    double voxelSize = nvl::maxLimitValue<Scalar>();

    //Calculate scale transform (minimum 1/4 of the average length)
    for (Mesh* meshPtr : meshes) {
        Scalar avgLength = nvl::meshAverageEdgeLength(*meshPtr);
        voxelSize = std::min(voxelSize, avgLength / 2.0);
    }


    double scaleFactor = 1.0 / voxelSize;
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
    std::vector<internal::OpenVDBAdapter<Mesh>> adapters(meshes.size());
    std::vector<IntGridPtr> polygonIndexGrid(meshes.size());
    std::vector<std::vector<VertexId>> gridBirthVertex(meshes.size());
    std::vector<std::vector<FaceId>> gridBirthFace(meshes.size());

    for (Index mId = 0; mId < meshes.size(); ++mId) {
        Mesh* meshPtr = meshes[mId];

        //Data
        Mesh currentMesh;
        FloatGridPtr& currentSignedGrid = signedGrids[mId];
        IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];
        internal::OpenVDBAdapter<Mesh>& currentAdapter = adapters[mId];

        //Get non-zero vertices
        std::vector<VertexId> vertices;
        for (VertexId vId = 0; vId < meshPtr->nextVertexId(); ++vId) {
            if (meshPtr->isVertexDeleted(vId))
                continue;

            if (vertexFuzzyValue[mId][vId] > DISCARD_THRESHOLD) {
                vertices.push_back(vId);
            }
        }
        nvl::meshTransferVerticesWithFaces(*meshPtr, vertices, currentMesh, gridBirthVertex[mId], gridBirthFace[mId]);

        currentAdapter = internal::OpenVDBAdapter<Mesh>(&currentMesh);
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

        //TODO DELETE
        nvl::meshSaveToFile("results/closed_ " + std::to_string(mId) + ".obj", closedMesh);

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

                int nInvolvedGrids = 0;
                float sumFuzzy = 0.0;

                bool overThresholdMoreThanOne = false;
                Index overThresholdMeshId = nvl::MAX_ID;

                for (Index mId = 0; mId < meshes.size(); ++mId) {
                    Mesh* meshPtr = meshes[mId];

                    FloatGridPtr& currentGrid = signedGrids[mId];
                    IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

                    FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                    IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                    FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                    IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                    float fuzzyValue = 0.0;
                    if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue < maxDistance && currentValue > -maxDistance) {
                        FaceId fId = gridBirthFace[mId][currentPolygonIndex];
                        for (VertexId j : meshPtr->face(fId).vertexIds()) {
                            fuzzyValue += vertexFuzzyValue[mId][j];
                        }
                        fuzzyValue /= meshPtr->face(fId).vertexNumber();

                        sumFuzzy += fuzzyValue;

                        if (fuzzyValue > KEEP_THRESHOLD) {
                            if (overThresholdMeshId == nvl::MAX_ID) {
                                overThresholdMeshId = mId;
                            }
                            else {
                                overThresholdMoreThanOne = true;
                            }
                        }

                        nInvolvedGrids++;
                    }
                }

                FloatGrid::ValueType resultValue = 0.0;

                if (overThresholdMeshId == nvl::MAX_ID || overThresholdMoreThanOne) {
                    for (Index mId = 0; mId < meshes.size(); ++mId) {
                        Mesh* meshPtr = meshes[mId];

                        FloatGridPtr& currentGrid = signedGrids[mId];
                        IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

                        FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                        IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                        FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                        IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                        if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue < maxDistance && currentValue > -maxDistance) {
                            if (sumFuzzy > DISCARD_THRESHOLD) {
                                float fuzzyValue = 0.0;

                                FaceId fId = gridBirthFace[mId][currentPolygonIndex];
                                for (VertexId j : meshPtr->face(fId).vertexIds()) {
                                    fuzzyValue += vertexFuzzyValue[mId][j];
                                }
                                fuzzyValue /= meshPtr->face(fId).vertexNumber();

                                fuzzyValue /= sumFuzzy;

                                resultValue += currentValue * fuzzyValue;
                            }
                            else {
                                if (nInvolvedGrids == 0) {
                                    if (resultValue >= 0) {
                                        assert(currentValue == maxDistance || currentValue == -maxDistance);
                                        resultValue = currentValue;
                                    }
                                }
                                else {
                                    resultValue += (1.0 / nInvolvedGrids) * currentValue;
                                }
                            }
                        }
                    }
                }
                else {
                    FloatGridPtr& currentGrid = signedGrids[overThresholdMeshId];
                    IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[overThresholdMeshId];

                    FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                    IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                    FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);

                    resultValue = currentValue;
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
    Mesh blendedMesh;
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

    //Rescale back
    nvl::meshApplyTransformation(blendedMesh, scaleTransform.inverse());

    //TODO DELETE
    nvl::meshSaveToFile("results/blendedMesh.obj", blendedMesh);




    //Fill vertices to keep for the original meshes
    std::vector<std::unordered_set<FaceId>> meshFacesToKeep(meshes.size());
    for (Index mId = 0; mId < meshes.size(); ++mId) {
        const Mesh* meshPtr = meshes[mId];
        for (FaceId fId = 0; fId < meshPtr->nextFaceId(); ++fId) {
            if (meshPtr->isFaceDeleted(fId))
                continue;

            float fuzzyValue = 0.0;
            for (VertexId j : meshPtr->face(fId).vertexIds()) {
                fuzzyValue += vertexFuzzyValue[mId][j];
            }
            fuzzyValue /= meshPtr->face(fId).vertexNumber();

            if (fuzzyValue > KEEP_THRESHOLD) {
                meshFacesToKeep[mId].insert(fId);
            }
        }
    }

    //Create preserved mesh
    preBirthVertices.clear();
    preBirthFaces.clear();
    preMesh.clear();
    std::unordered_set<VertexId> preNonSnappableVertices;
    for (Index mId = 0; mId < meshes.size(); ++mId) {
        Mesh* meshPtr = meshes[mId];

        nvl::meshOpenFaceSelection(*meshPtr, meshFacesToKeep[mId]);
        nvl::meshCloseFaceSelection(*meshPtr, meshFacesToKeep[mId]);

        VertexId startVertexId = preMesh.nextVertexId();

        nvl::meshTransferFaces(*meshPtr, std::vector<VertexId>(meshFacesToKeep[mId].begin(), meshFacesToKeep[mId].end()), preMesh, preBirthVertices, preBirthFaces);
        preVerticesBirthModel.resize(preBirthVertices.size(), mId);
        preFacesBirthModel.resize(preBirthFaces.size(), mId);

        //Find vertices that were already in the border of the original mesh
        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(*meshPtr);
        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());
        for (VertexId vId = startVertexId; vId < preMesh.nextVertexId(); vId++) {
            if (borderVerticesSet.find(preBirthVertices[vId]) != borderVerticesSet.end()) {
                preNonSnappableVertices.insert(vId);
            }
        }
    }

    //TODO DELETE
    nvl::meshSaveToFile("results/preMesh.obj", preMesh);

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

            for (Index mId = 0; mId < meshes.size(); ++mId) {
                FloatGridPtr& currentGrid = signedGrids[mId];
                IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

                FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
                IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

                FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
                IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

                if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue <= 2.0) {
                    FaceId meshOriginFId = gridBirthFace[mId][currentPolygonIndex];

                    if (meshFacesToKeep[mId].find(meshOriginFId) != meshFacesToKeep[mId].end()) {
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

    //TODO DELETE
    nvl::meshSaveToFile("results/newMeshRegularized.obj", newMesh);

    internal::attachMeshToMeshByBorders(newMesh, preMesh, std::unordered_set<VertexId>(), preNonSnappableVertices);

    //TODO DELETE
    nvl::meshSaveToFile("results/newMesh.obj", newMesh);


}

namespace internal {

template<class Mesh>
void attachMeshToMeshByBorders(
        Mesh& mesh,
        const Mesh& destMesh,
        const std::unordered_set<typename Mesh::VertexId>& meshNonSnappableVertices,
        const std::unordered_set<typename Mesh::VertexId>& destNonSnappableVertices)
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
            std::vector<VertexId> closestMVertex(dChain.size(), nvl::MAX_ID);
            for (Index j = 0; j < dChain.size(); ++j) {
                const Point& pPoint = destMesh.vertex(dChain[j]).point();

                VertexId closestI = nvl::MAX_ID;
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
                        closestI = i;

                        if (dist <= startDist) {
                            startDist = dist;
                            startI = i;
                            startJ = j;
                            startT = t;
                        }
                    }
                }

                closestMVertex[j] = closestI;
            }

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
            std::vector<double> mLengthParametrization(mChain.size());
            std::vector<double> dLengthParametrization(dChain.size());

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

            //TODO FUNCTION IN NVL
            //Compute parametrization for destination mesh
            Scalar dChainCurrentDist = 0.0;
            dLengthParametrization[0] = 0.0;
            for (Index j = 0; j < dChain.size() - 1; ++j) {
                Index currentJ = (startJ + j) % dChain.size();
                Index nextJ = (currentJ + 1) % dChain.size();

                nvl::Vector3d vec = dSmoothPoint[nextJ] - dSmoothPoint[currentJ];
                dChainCurrentDist += vec.norm();

                dLengthParametrization[nextJ] = dChainCurrentDist / dChainTotalDist;
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
            mLengthParametrization[0] = offsetT;
            for (Index i = 0; i < mChain.size() - 1; ++i) {
                Index currentI = (startI + i) % mChain.size();
                Index nextI = (currentI + 1) % mChain.size();

                nvl::Vector3d vec = mSmoothPoint[nextI] - mSmoothPoint[currentI];
                mChainCurrentDist += vec.norm();

                //+ offsetT to be checked
                mLengthParametrization[nextI] = offsetT + (mChainCurrentDist / mChainTotalDist);
            }

            //Final parametrization
            std::vector<double> mParametrization = mLengthParametrization;
            std::vector<double> dParametrization = dLengthParametrization;

            //Compute score by computing the distortion
            Scalar score = 0.0;
            Index i = 0;
            for (Index j = 0; j < dChain.size(); ++j) {
                Index currentJ = (startJ + j) % dChain.size();
                Index currentI = (startI + i) % mChain.size();
                Index nextI = (currentI + 1) % mChain.size();

                const Point& pPoint = destMesh.vertex(dChain[currentJ]).point();

                const Scalar& pT = dParametrization[currentJ];

                while (i < mChain.size() - 1 && mParametrization[nextI] < pT) {
                    i++;

                    currentI = (startI + i) % mChain.size();
                    nextI = (currentI + 1) % mChain.size();
                }

                Point currentIPoint = tmpMesh.vertex(mChain[currentI]).point();
                Point nextIPoint = tmpMesh.vertex(mChain[nextI]).point();

                const Scalar& mT = mParametrization[currentI];
                Scalar nextParameter = 1.0;
                if (i < mChain.size() - 1) {
                    nextParameter = mParametrization[nextI];
                }

                Scalar currentWeight = (pT - mT) / (nextParameter - mT);

                Point nPoint = currentWeight * currentIPoint + (1 - currentWeight) * nextIPoint;

                score += (nPoint - pPoint).norm();
            }

            //We save the best score pair of chains
            if (score <= bestScore) {
                bestScore = score;
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
            assert(currentMChain[i] != nvl::MAX_ID);
            mChain[i] = cleaningVertexMap[mChain[i]];
        }
    }

    //TODO DELETE
    nvl::meshSaveToFile("results/newMeshCleaned.obj", mesh);

    //Split vertices
    std::unordered_set<VertexId> splitVertices;
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
        VertexId currentNVertexId = mChain[i];
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
            splitVertices.insert(newNVertexId);
            currentNVertexId = newNVertexId;
        }
    }

    //TODO DELETE
    nvl::meshSaveToFile("results/newMeshSplitted.obj", mesh);

    std::vector<VertexId> nonCollapsed = nvl::collapseBorders(mesh, splitVertices);
    std::cout << nonCollapsed.size() << " vertices non collapsed." << std::endl;
}

}

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    //Associated vertices to be connected
//    std::unordered_set<Index> usedComponents;
//    std::vector<Index> associatedNChain(preChains.size(), nvl::MAX_ID);
//    std::vector<std::vector<Index>> associatedNChainEdgeToSplit(preChains.size());

//    for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//        if (!preservedSnappableChain[pChainId])
//            continue;

//        const std::vector<VertexId>& pChain = preChains[pChainId];

//        Scalar bestScore = nvl::maxLimitValue<Scalar>();
//        Index bestNChainId = nvl::MAX_ID;
//        std::vector<Index> bestAssociation;

//        for (Index nChainId = 0; nChainId < newChains.size(); ++nChainId) {
//            if (!newSnappableChain[nChainId])
//                continue;

//            const std::vector<VertexId>& nChain = newChains[nChainId];

//            Scalar currentScore = 0.0;
//            std::vector<Index> currentAssociation(pChain.size(), nvl::maxLimitValue<Index>());
//            Index firstI = nvl::MAX_ID;
//            Index currentI = 0;

//            unsigned int maxIterations = std::max(std::max(nChain.size() / pChain.size() * 3, nChain.size() / 4), static_cast<size_t>(2));
//            for (Index j = 0; j < pChain.size(); ++j) {
//                const Point& pPoint = preservedMesh.vertex(pChain[j]).point();

//                Scalar bestDistance = nvl::maxLimitValue<Scalar>();

//                Index stopI = currentI;
//                Index i = currentI;

//                unsigned int iteration = 0;
//                do {
//                    Index nextI = (i + 1) % nChain.size();

//                    const Point& nPoint = tmpNewMesh.vertex(nChain[i]).point();
//                    const Point& nNextPoint = tmpNewMesh.vertex(nChain[nextI]).point();

//                    Point closestPoint = nvl::getClosestPointOnSegment(nPoint, nNextPoint, pPoint);
//                    Scalar dist = (closestPoint - pPoint).norm();

//                    if (dist <= bestDistance) {
//                        bestDistance = dist;
//                        currentAssociation[j] = i;
//                        currentI = i;
//                    }

//                    i = nextI;

//                    iteration++;
//                }
//                while (i != firstI && i != stopI && iteration < maxIterations);

//                if (firstI == nvl::MAX_ID) {
//                    firstI = currentI;
//                }

//                currentScore += bestDistance;
//            }

//            if (currentScore <= bestScore) {
//                bestScore = currentScore;
//                bestNChainId = nChainId;
//                bestAssociation = currentAssociation;
//            }
//        }

//        if (bestScore < nvl::MAX_ID) {
//            usedComponents.insert(newChainsComponent[bestNChainId]);

//            associatedNChain[pChainId] = bestNChainId;
//            associatedNChainEdgeToSplit[pChainId] = bestAssociation;

//            preservedSnappableChain[pChainId] = false;
//            newSnappableChain[bestNChainId] = false;
//        }
//    }

//    //Clean mesh from components that are non-splitted in borders
//    std::vector<FaceId> facesToKeep;
//    for (const Index& cId : usedComponents) {
//        const std::vector<FaceId>& connectedComponent = connectedComponents[cId];
//        facesToKeep.insert(facesToKeep.end(), connectedComponent.begin(), connectedComponent.end());
//    }
//    std::vector<VertexId> cleaningBirthVertex;
//    std::vector<FaceId> cleaningBirthFace;
//    nvl::meshTransferFaces(tmpNewMesh, facesToKeep, newMesh, cleaningBirthVertex, cleaningBirthFace);

//    //We transform the old vertices in the new ones
//    std::vector<VertexId> cleaningVertexMap = nvl::getInverseMap(cleaningBirthVertex);
//    for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//        const Index& nChainId = associatedNChain[pChainId];
//        if (nChainId == nvl::MAX_ID)
//            continue;

//        std::vector<VertexId>& nChain = newChains[nChainId];

//        for (Index i = 0; i < nChain.size(); ++i) {
//            assert(cleaningVertexMap[nChain[i]] != nvl::MAX_ID);
//            nChain[i] = cleaningVertexMap[nChain[i]];
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMeshCleaned.obj", newMesh);

//    //Split vertices
//    std::unordered_set<VertexId> splitVertices;
//    std::vector<std::vector<FaceId>> newMeshVFAdj = nvl::meshVertexFaceAdjacencies(newMesh);
//    for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//        const std::vector<VertexId>& pChain = preChains[pChainId];


//        const Index& nChainId = associatedNChain[pChainId];

//        if (nChainId == nvl::MAX_ID)
//            continue;

//        std::vector<VertexId>& nChain = newChains[nChainId];
//        std::vector<Index>& edgeToSplit = associatedNChainEdgeToSplit[pChainId];

//        for (Index j = 0; j < pChain.size(); ++j) {
//            const Point& splitPoint = preservedMesh.vertex(pChain[j]).point();

//            const Index nChainVertexId = edgeToSplit[j];
//            const Index nChainNextVertexId = (nChainVertexId + 1) % nChain.size();

//            const VertexId& splitV1 = nChain[nChainVertexId];
//            const VertexId& splitV2 = nChain[nChainNextVertexId];

//            VertexId newNVertexId = nvl::meshSplitEdge(newMesh, splitV1, splitV2, splitPoint, newMeshVFAdj);
//            splitVertices.insert(newNVertexId);

//            //Add vertex to the chain and update the edges of the chain to split
//            nChain.insert(nChain.begin() + nChainNextVertexId, newNVertexId);
//            for (Index k = j + 1; k < edgeToSplit.size(); ++k) {
//                if (edgeToSplit[k] == nChainVertexId) {
//                    edgeToSplit[k] = nChainNextVertexId;
//                }
//                else if (edgeToSplit[k] >= nChainNextVertexId) {
//                    edgeToSplit[k]++;
//                }
//            }
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMeshSplitted.obj", newMesh);

//    std::vector<VertexId> nonCollapsed = internal::collapseBorders(newMesh, splitVertices);
//    std::cout << nonCollapsed.size() << " vertices non collapsed." << std::endl;

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMesh.obj", newMesh);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    std::unordered_set<Index> usedComponents;
//    std::vector<std::vector<VertexId>> selectedNChainsOrdered;
//    std::vector<std::vector<VertexId>> selectedPChainsOrdered;
//    std::vector<std::vector<double>> selectedNChainsParametrization;
//    std::vector<std::vector<double>> selectedPChainsParametrization;

//    for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//        if (!preservedSnappableChain[pChainId])
//            continue;

//        const std::vector<VertexId>& pChain = preChains[pChainId];

//        Scalar bestScore = nvl::maxLimitValue<Scalar>();
//        Index bestNChainId = nvl::MAX_ID;

//        std::vector<VertexId> bestNChainOrdered;
//        std::vector<double> bestNChainParametrization;
//        std::vector<VertexId> bestPChainOrdered;
//        std::vector<double> bestPChainParametrization;

//        for (Index nChainId = 0; nChainId < newChains.size(); ++nChainId) {
//            if (!newSnappableChain[nChainId])
//                continue;

//            const std::vector<VertexId>& nChain = newChains[nChainId];

//            VertexId startI = nvl::MAX_ID;
//            VertexId startJ = nvl::MAX_ID;
//            Scalar startDist = nvl::maxLimitValue<Scalar>();

//            for (Index j = 0; j < pChain.size(); ++j) {
//                const Vertex& pVertex = preservedMesh.vertex(pChain[j]);

//                for (Index i = 0; i < nChain.size(); ++i) {
//                    const Vertex& nVertex = tmpNewMesh.vertex(nChain[i]);

//                    Scalar dist = (nVertex.point() - pVertex.point()).norm();
//                    if (dist <= startDist) {
//                        startDist = dist;
//                        startI = i;
//                        startJ = j;
//                    }
//                }
//            }

//            std::vector<VertexId> nChainOrdered(nChain.size(), nvl::MAX_ID);
//            std::vector<VertexId> pChainOrdered(pChain.size(), nvl::MAX_ID);
//            std::vector<double> nChainParametrization(nChain.size());
//            std::vector<double> pChainParametrization(pChain.size());

//            Scalar nChainTotalDist = 0;
//            for (Index i = 0; i < nChain.size(); ++i) {
//                Index offsetI = (startI + i) % nChain.size();
//                nChainOrdered[i] = nChain[offsetI];

//                Index nextI = (i + 1) % nChain.size();
//                nvl::Vector3d vec = tmpNewMesh.vertex(nChain[nextI]).point() - tmpNewMesh.vertex(nChain[i]).point();
//                nChainTotalDist += vec.norm();
//            }
//            Scalar pChainTotalDist = 0;
//            for (Index j = 0; j < pChain.size(); ++j) {
//                long long int indexDiff = static_cast<long long int>(startJ) - static_cast<long long int>(j);
//                Index offsetJ = (indexDiff >= 0 ? indexDiff : pChain.size() + indexDiff);
//                pChainOrdered[j] = pChain[offsetJ];

//                Index nextJ = j > 0 ? j - 1 : pChain.size() - 1;
//                nvl::Vector3d vec = preservedMesh.vertex(pChain[nextJ]).point() - preservedMesh.vertex(pChain[j]).point();
//                pChainTotalDist += vec.norm();
//            }

//            Scalar nChainCurrentDist = 0.0;
//            nChainParametrization[0] = 0.0;
//            for (Index i = 0; i < nChainOrdered.size() - 1; ++i) {
//                nvl::Vector3d vec = tmpNewMesh.vertex(nChainOrdered[i + 1]).point() - tmpNewMesh.vertex(nChainOrdered[i]).point();
//                nChainCurrentDist += vec.norm();

//                nChainParametrization[i + 1] = nChainCurrentDist / nChainTotalDist;
//            }
//            Scalar pChainCurrentDist = 0.0;
//            pChainParametrization[0] = 0.0;
//            for (Index j = 0; j < pChainOrdered.size() - 1; ++j) {
//                nvl::Vector3d vec = preservedMesh.vertex(pChainOrdered[j + 1]).point() - preservedMesh.vertex(pChainOrdered[j]).point();
//                pChainCurrentDist += vec.norm();

//                pChainParametrization[j + 1] = pChainCurrentDist / pChainTotalDist;
//            }

//            Scalar score = 0.0;

//            Index i = 0;
//            for (Index j = 0; j < pChainOrdered.size(); ++j) {
//                const Point& pPoint = preservedMesh.vertex(pChainOrdered[j]).point();

//                const Scalar& t = pChainParametrization[j];

//                while (i < nChainOrdered.size() - 1 && nChainParametrization[(i + 1) % nChainOrdered.size()] < t) {
//                    i++;
//                }

//                Point currentIPoint = tmpNewMesh.vertex(nChainOrdered[i]).point();
//                Point nextIPoint = tmpNewMesh.vertex(nChainOrdered[(i + 1) % nChainOrdered.size()]).point();

//                const Scalar& currentIT = nChainParametrization[i];
//                Scalar nextIT = 1.0;
//                if (i < nChainOrdered.size() - 1) {
//                   nextIT = nChainParametrization[i + 1];
//                }

//                Scalar currentWeight = (t - currentIT) / (nextIT - currentIT);

//                Point nPoint = currentWeight * currentIPoint + (1 - currentWeight) * nextIPoint;

//                score += (nPoint - pPoint).norm();
//            }

//            if (score <= bestScore) {
//                bestScore = score;
//                bestNChainOrdered = nChainOrdered;
//                bestPChainOrdered = pChainOrdered;
//                bestNChainParametrization = nChainParametrization;
//                bestPChainParametrization = pChainParametrization;
//                bestNChainId = nChainId;
//            }
//        }


//        if (bestScore < nvl::MAX_ID) {
//            selectedNChainsOrdered.push_back(bestNChainOrdered);
//            selectedPChainsOrdered.push_back(bestPChainOrdered);
//            selectedNChainsParametrization.push_back(bestNChainParametrization);
//            selectedPChainsParametrization.push_back(bestPChainParametrization);

//            usedComponents.insert(newChainsComponent[bestNChainId]);

//            preservedSnappableChain[pChainId] = false;
//            newSnappableChain[bestNChainId] = false;
//        }
//    }

//    //Clean mesh from components that are non-splitted in borders
//    std::vector<FaceId> facesToKeep;
//    for (const Index& cId : usedComponents) {
//        const std::vector<FaceId>& connectedComponent = connectedComponents[cId];
//        facesToKeep.insert(facesToKeep.end(), connectedComponent.begin(), connectedComponent.end());
//    }
//    std::vector<VertexId> cleaningBirthVertex;
//    std::vector<FaceId> cleaningBirthFace;
//    nvl::meshTransferFaces(tmpNewMesh, facesToKeep, newMesh, cleaningBirthVertex, cleaningBirthFace);

//    std::vector<VertexId> cleaningVertexMap = nvl::getInverseMap(cleaningBirthVertex);
//    for (Index k = 0; k < selectedNChainsOrdered.size(); ++k) {
//        std::vector<VertexId>& currentNChainOrdered = selectedNChainsOrdered[k];

//        for (Index i = 0; i < currentNChainOrdered.size(); ++i) {
//            assert(currentNChainOrdered[i] != nvl::MAX_ID);
//            currentNChainOrdered[i] = cleaningVertexMap[currentNChainOrdered[i]];
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMeshCleaned.obj", newMesh);

//    //Split vertices
//    std::unordered_set<VertexId> splitVertices;
//    std::vector<std::vector<FaceId>> newMeshVFAdj = nvl::meshVertexFaceAdjacencies(newMesh);
//    for (Index k = 0; k < selectedNChainsOrdered.size(); ++k) {
//        const std::vector<VertexId>& currentNChainOrdered = selectedNChainsOrdered[k];
//        const std::vector<double>& currentNChainParametrization = selectedNChainsParametrization[k];
//        const std::vector<VertexId>& currentPChainOrdered = selectedPChainsOrdered[k];
//        const std::vector<double>& currentPChainParametrization = selectedPChainsParametrization[k];

//        Index i = 0;
//        VertexId currentNVertexId = currentNChainOrdered[i];
//        for (Index j = 0; j < currentPChainOrdered.size(); ++j) {
//            const Point& jPoint = preservedMesh.vertex(currentPChainOrdered[j]).point();

//            const Scalar& t = currentPChainParametrization[j];

//            while (i < currentNChainOrdered.size() - 1 && currentNChainParametrization[(i + 1) % currentNChainOrdered.size()] < t) {
//                i++;
//                currentNVertexId = currentNChainOrdered[i];
//            }

//            VertexId newNVertexId = nvl::meshSplitEdge(newMesh, currentNVertexId, currentNChainOrdered[(i + 1) % currentNChainOrdered.size()], jPoint, newMeshVFAdj);
//            splitVertices.insert(newNVertexId);
//            currentNVertexId = newNVertexId;
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMeshSplitted.obj", newMesh);

//    std::vector<VertexId> nonCollapsed = internal::collapseBorders(newMesh, splitVertices);
//    std::cout << nonCollapsed.size() << " vertices non collapsed." << std::endl;

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMesh.obj", newMesh);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    //Snap maps
//    std::vector<std::pair<Index, VertexId>> blendedSnapMap(blendedMesh.nextVertexId(), std::make_pair(nvl::MAX_ID, nvl::MAX_ID));
//    std::vector<std::vector<std::set<VertexId>>> preservedSnapMap(meshes.size());
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];
//        preservedSnapMap[mId].resize(meshPtr->nextVertexId());
//    }

//    //Snap vertices of the triangles on the preserved vertices
//    for (VertexId bVId = 0; bVId < blendedMesh.nextVertexId(); ++bVId) {
//        if (blendedMesh.isVertexDeleted(bVId))
//            continue;

//        Point point = blendedMesh.vertex(bVId).point();

//        VertexId bestVId = nvl::MAX_ID;
//        Scalar bestDistance = nvl::maxLimitValue<Scalar>();
//        Index bestMId = nvl::MAX_ID;

//        for (Index mId = 0; mId < meshes.size(); ++mId) {
//            const Mesh* meshPtr = meshes[mId];

//            IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

//            IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

//            Point scaledPoint = scaleTransform * point;
//            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

//            IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

//            if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>()) {
//                FaceId mFId = gridBirthFace[mId][currentPolygonIndex];

//                float fuzzyValue = 0.0;
//                for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                    fuzzyValue += vertexFuzzyValue[mId][mVId];
//                }
//                fuzzyValue /= meshPtr->face(mFId).vertexNumber();

//                //TODO CONTROLLO DIST?
//                if (fuzzyValue > KEEP_THRESHOLD) {
//                    for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                        Point targetPoint = meshPtr->vertex(mVId).point();

//                        Scalar dist = (targetPoint - point).norm();

//                        if (dist <= bestDistance) {
//                            bestVId = mVId;
//                            bestDistance = dist;
//                            bestMId = mId;
//                        }
//                    }
//                }
//            }
//        }

//        if (bestDistance < nvl::maxLimitValue<Scalar>()) {
//            Point bestPoint = meshes[bestMId]->vertex(bestVId).point();

//            blendedMesh.vertex(bVId).setPoint(bestPoint);

//            blendedSnapMap[bVId] = std::make_pair(bestMId, bestVId);
//            preservedSnapMap[bestMId][bestVId].insert(bVId);
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/snappedMesh.obj", blendedMesh);


//    //Faces and vertices to keep
//    std::set<FaceId> blendedMeshFacesToKeep;
//    std::set<VertexId> blendedMeshVertexInKeptFaces;
//    std::vector<std::set<FaceId>> preservedMeshFacesToKeep(meshes.size());
//    std::vector<std::set<VertexId>> preservedMeshVerticesInKeptFaces(meshes.size());

//    //Find initial vertices and faces to keep in the blended mesh
//    for (FaceId bFId = 0; bFId < blendedMesh.nextFaceId(); ++bFId) {
//        if (blendedMesh.isFaceDeleted(bFId))
//            continue;

//        bool isFaceKept = false;

//        const Face& bFace = blendedMesh.face(bFId);

//        for (const VertexId& bVId : bFace.vertexIds()) {
//            const VertexId& pVId = blendedSnapMap[bVId].second;

//            //If a vertex has not been snapped
//            if (pVId == nvl::MAX_ID) {
//                isFaceKept = true;
//            }
//        }

//        if (isFaceKept) {
//            blendedMeshFacesToKeep.insert(bFId);
//            for (const VertexId& bVId : bFace.vertexIds()) {
//                blendedMeshVertexInKeptFaces.insert(bVId);
//            }
//        }
//    }

//    //Find initial vertices and faces to keep in the original meshes
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];

//        for (FaceId mFId = 0; mFId < meshPtr->nextFaceId(); ++mFId) {
//            if (meshPtr->isFaceDeleted(mFId))
//                continue;

//            float fuzzyValue = 0.0;
//            for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                fuzzyValue += vertexFuzzyValue[mId][mVId];
//            }
//            fuzzyValue /= meshPtr->face(mFId).vertexNumber();

//            if (fuzzyValue > KEEP_THRESHOLD) {
//                preservedMeshFacesToKeep[mId].insert(mFId);
//                for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                    preservedMeshVerticesInKeptFaces[mId].insert(mVId);
//                }
//            }
//        }
//    }

//    //Delete faces in the preserved meshes that have at least one vertex that has been kept in the blended mesh
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];

//        for (FaceId mFId = 0; mFId < meshPtr->nextFaceId(); ++mFId) {
//            if (meshPtr->isFaceDeleted(mFId))
//                continue;

//            if (preservedMeshFacesToKeep[mId].find(mFId) == preservedMeshFacesToKeep[mId].end())
//                continue;

//            const Face& mFace = meshPtr->face(mFId);

//            bool deleteFace = false;

//            for (const VertexId& mVId : mFace.vertexIds()) {
//                for (const VertexId& bVId : preservedSnapMap[mId][mVId]) {
//                    if (blendedMeshVertexInKeptFaces.find(bVId) != blendedMeshVertexInKeptFaces.end()) {
//                        deleteFace = true;
//                    }
//                }
//            }

//            if (deleteFace) {
//                preservedMeshFacesToKeep[mId].erase(mFId);
//                for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                    preservedMeshVerticesInKeptFaces[mId].erase(mVId);
//                }
//            }
//        }
//    }

//    //Add faces in the blended mesh that have all vertices that have not been kept in a preserved mesh
//    for (FaceId bFId = 0; bFId < blendedMesh.nextFaceId(); ++bFId) {
//        if (blendedMesh.isFaceDeleted(bFId))
//            continue;

//        if (blendedMeshFacesToKeep.find(bFId) != blendedMeshFacesToKeep.end())
//            continue;

//        const Face& bFace = blendedMesh.face(bFId);

//        bool insertFace = true;

//        for (const VertexId& bVId : bFace.vertexIds()) {
//            const Index& mId = blendedSnapMap[bVId].first;
//            const VertexId& pVId = blendedSnapMap[bVId].second;

//            assert(pVId != nvl::MAX_ID);
//            if (preservedMeshVerticesInKeptFaces[mId].find(pVId) != preservedMeshVerticesInKeptFaces[mId].end()) {
//                insertFace = false;
//            }
//        }

//        if (insertFace) {
//            blendedMeshFacesToKeep.insert(bFId);
//        }
//    }





//    //Transfer in the new surface the faces to be kept
//    newMesh.clear();
//    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(blendedMeshFacesToKeep.begin(), blendedMeshFacesToKeep.end()), newMesh);

//    //Create preserved mesh from the vertices which have been found in the blended mesh
//    preservedMesh.clear();
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        Mesh* meshPtr = meshes[mId];

//        nvl::meshTransferFaces(*meshPtr, std::vector<VertexId>(preservedMeshFacesToKeep[mId].begin(), preservedMeshFacesToKeep[mId].end()), preservedMesh, preservedBirthVertices, preservedBirthFaces);
//        preservedVerticesBirthModel.resize(preservedBirthVertices.size(), mId);
//        preservedFacesBirthModel.resize(preservedBirthFaces.size(), mId);
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/preservedMesh.obj", preservedMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMesh.obj", newMesh);





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    //Fill vertices to keep for the original meshes
//    std::vector<std::unordered_set<FaceId>> meshFacesToKeep(meshes.size());
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];
//        for (FaceId fId = 0; fId < meshPtr->nextFaceId(); ++fId) {
//            if (meshPtr->isFaceDeleted(fId))
//                continue;

//            float fuzzyValue = 0.0;
//            for (VertexId j : meshPtr->face(fId).vertexIds()) {
//                fuzzyValue += vertexFuzzyValue[mId][j];
//            }
//            fuzzyValue /= meshPtr->face(fId).vertexNumber();

//            if (fuzzyValue > KEEP_THRESHOLD) {
//                meshFacesToKeep[mId].insert(fId);
//            }
//        }
//    }

//    //Fill vertices to keep in the blended mesh
//    std::unordered_set<FaceId> blendedMeshFacesToKeep;
//    for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
//        if (blendedMesh.isFaceDeleted(fId))
//            continue;

//        bool isNewSurface = true;

//        for (VertexId vId : blendedMesh.face(fId).vertexIds()) {
//            const Point& point = blendedMesh.vertex(vId).point();

//            Point scaledPoint = scaleTransform * point;
//            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

//            for (Index mId = 0; mId < meshes.size(); ++mId) {
//                FloatGridPtr& currentGrid = signedGrids[mId];
//                IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

//                FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
//                IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

//                FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
//                IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

//                if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue <= 2.0) {
//                    FaceId meshOriginFId = gridBirthFace[mId][currentPolygonIndex];

//                    if (meshFacesToKeep[mId].find(meshOriginFId) != meshFacesToKeep[mId].end()) {
//                        isNewSurface = false;
//                    }
//                }
//            }
//        }

//        if (isNewSurface) {
//            blendedMeshFacesToKeep.insert(fId);
//        }
//    }

//    //Create preserved mesh
//    std::unordered_set<VertexId> preservedVerticesAlreadyBorders;
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        Mesh* meshPtr = meshes[mId];

//        VertexId startVertexId = preservedMesh.nextVertexId();

//        nvl::meshTransferFaces(*meshPtr, std::vector<VertexId>(meshFacesToKeep[mId].begin(), meshFacesToKeep[mId].end()), preservedMesh, preservedBirthVertices, preservedBirthFaces);
//        preservedVerticesBirthModel.resize(preservedBirthVertices.size(), mId);
//        preservedFacesBirthModel.resize(preservedBirthFaces.size(), mId);


//        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(*meshPtr);
//        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());
//        for (VertexId vId = startVertexId; vId < preservedMesh.nextVertexId(); vId++) {
//            if (borderVerticesSet.find(preservedBirthVertices[vId]) != borderVerticesSet.end()) {
//                preservedVerticesAlreadyBorders.insert(vId);
//            }
//        }
//    }

//    //Transfer in the new surface the faces to be kept
//    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(blendedMeshFacesToKeep.begin(), blendedMeshFacesToKeep.end()), newMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/preservedMesh.obj", preservedMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMeshNonSnapped.obj", newMesh);

//    //Get the border vertex chains for the two meshes
//    std::vector<std::vector<VertexId>> preChains = nvl::meshBorderVertexChains(preservedMesh);
//    std::vector<std::vector<VertexId>> newChains = nvl::meshBorderVertexChains(newMesh);

//    //Snappable preserved chains: we do not snap the chains which were borders in the original meshes
//    std::vector<bool> preservedSnappableChain(preChains.size(), true);
//    for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//        const std::vector<VertexId>& chain = preChains[pChainId];
//        for (const VertexId& vId : chain) {
//            if (preservedVerticesAlreadyBorders.find(vId) != preservedVerticesAlreadyBorders.end()) {
//                preservedSnappableChain[pChainId] = false;
//            }
//        }
//    }

//    //Sort new chain by sizes
//    std::sort(newChains.begin(), newChains.end(), [](const std::vector<VertexId>& lhs, const std::vector<VertexId>& rhs)
//    {
//        return lhs.size() > rhs.size();
//    });
//    //Snappable new surface chains: we do not snap chains with less than 10 vertices
//    std::vector<bool> newSnappableChain(newChains.size(), true);
//    for (Index nChainId = 0; nChainId < newChains.size(); ++nChainId) {
//        const std::vector<VertexId>& nChain = newChains[nChainId];
//        if (nChain.size() <= MIN_CHAIN) {
//            newSnappableChain[nChainId] = false;
//        }
//    }

//    for (Index nChainId = 0; nChainId < newChains.size(); ++nChainId) {
//        if (!newSnappableChain[nChainId])
//            continue;

//        const std::vector<VertexId>& nChain = newChains[nChainId];

//        VertexId bestI = nvl::MAX_ID;
//        VertexId bestJ = nvl::MAX_ID;
//        VertexId bestPChainId = nvl::MAX_ID;
//        Scalar bestDist = nvl::maxLimitValue<Scalar>();

//        for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//            if (!preservedSnappableChain[pChainId])
//                continue;

//            const std::vector<VertexId>& pChain = preChains[pChainId];

//            for (Index i = 0; i < nChain.size(); ++i) {
//                const Vertex& nVertex = newMesh.vertex(nChain[i]);

//                for (Index j = 0; j < pChain.size(); ++j) {
//                    const Vertex& pVertex = preservedMesh.vertex(pChain[j]);

//                    Scalar dist = (nVertex.point() - pVertex.point()).norm();

//                    if (dist <= bestDist) {
//                        bestDist = dist;
//                        bestI = i;
//                        bestJ = j;
//                        bestPChainId = pChainId;
//                    }
//                }
//            }
//        }

//        //TODO add control on best dist (???)
//        if (bestPChainId < nvl::MAX_ID) {
//            const std::vector<VertexId>& nChain = newChains[nChainId];
//            const std::vector<VertexId>& pChain = preChains[bestPChainId];

//            Index i = bestI;
//            Index j = bestJ;

//            bool incrementI = true;
//            bool incrementJ = true;

//            unsigned int numConsecutive = 0;
//            const unsigned int factor = nChain.size() / pChain.size();

//            do {
//                Vertex& newVertex = newMesh.vertex(nChain[i]);

//                if (incrementJ) {
//                    VertexId nextJ = (j > 0 ? j - 1 : pChain.size() - 1);

//                    const Vertex& preservedVertex = preservedMesh.vertex(pChain[j]);
//                    const Vertex& nextPreservedVertex = preservedMesh.vertex(pChain[nextJ]);

//                    Scalar currentDist = (newVertex.point() - preservedVertex.point()).norm();
//                    Scalar nextDist = (newVertex.point() - nextPreservedVertex.point()).norm();

//                    Scalar multiplier = 1;
//                    if (numConsecutive > factor) {
//                        multiplier += (numConsecutive - factor) * (1.0 / numConsecutive);
//                    }

//                    if (!incrementI || nextDist <= currentDist * multiplier) {
//                        j = nextJ;
//                        incrementJ = j != bestJ;

//                        numConsecutive = 0;
//                    }
//                    else {
//                        numConsecutive++;
//                    }
//                }

//                const Vertex& targetVertex = preservedMesh.vertex(pChain[j]);

//                newVertex.setPoint(targetVertex.point());

//                if (incrementI) {
//                    VertexId nextI = (i + 1) % nChain.size();
//                    i = nextI;
//                    incrementI = i != bestI;
//                }
//            } while (incrementI || incrementJ);

//            preservedSnappableChain[bestPChainId] = false;
//            newSnappableChain[nChainId] = false;
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMesh.obj", newMesh);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    //Fill vertices to keep for the original meshes
//    std::vector<std::unordered_set<FaceId>> meshFacesToKeep(meshes.size());
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];
//        for (FaceId fId = 0; fId < meshPtr->nextFaceId(); ++fId) {
//            if (meshPtr->isFaceDeleted(fId))
//                continue;

//            float fuzzyValue = 0.0;
//            for (VertexId j : meshPtr->face(fId).vertexIds()) {
//                fuzzyValue += vertexFuzzyValue[mId][j];
//            }
//            fuzzyValue /= meshPtr->face(fId).vertexNumber();

//            if (fuzzyValue > KEEP_THRESHOLD) {
//                meshFacesToKeep[mId].insert(fId);
//            }
//        }
//    }

//    //Fill vertices to keep in the blended mesh
//    std::unordered_set<FaceId> blendedMeshFacesToKeep;
//    for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
//        if (blendedMesh.isFaceDeleted(fId))
//            continue;

//        bool isNewSurface = true;

//        for (VertexId vId : blendedMesh.face(fId).vertexIds()) {
//            const Point& point = blendedMesh.vertex(vId).point();

//            Point scaledPoint = scaleTransform * point;
//            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

//            for (Index mId = 0; mId < meshes.size(); ++mId) {
//                FloatGridPtr& currentGrid = signedGrids[mId];
//                IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

//                FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
//                IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

//                FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
//                IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

//                if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue <= 2.0) {
//                    FaceId meshOriginFId = gridBirthFace[mId][currentPolygonIndex];

//                    if (meshFacesToKeep[mId].find(meshOriginFId) != meshFacesToKeep[mId].end()) {
//                        isNewSurface = false;
//                    }
//                }
//            }
//        }

//        if (isNewSurface) {
//            blendedMeshFacesToKeep.insert(fId);
//        }
//    }

//    //Create preserved mesh
//    std::unordered_set<VertexId> preservedVerticesAlreadyBorders;
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        Mesh* meshPtr = meshes[mId];

//        VertexId startVertexId = preservedMesh.nextVertexId();

//        nvl::meshTransferFaces(*meshPtr, std::vector<VertexId>(meshFacesToKeep[mId].begin(), meshFacesToKeep[mId].end()), preservedMesh, preservedBirthVertices, preservedBirthFaces);
//        preservedVerticesBirthModel.resize(preservedBirthVertices.size(), mId);
//        preservedFacesBirthModel.resize(preservedBirthFaces.size(), mId);


//        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(*meshPtr);
//        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());
//        for (VertexId vId = startVertexId; vId < preservedMesh.nextVertexId(); vId++) {
//            if (borderVerticesSet.find(preservedBirthVertices[vId]) != borderVerticesSet.end()) {
//                preservedVerticesAlreadyBorders.insert(vId);
//            }
//        }
//    }

//    //Transfer in the new surface the faces to be kept
//    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(blendedMeshFacesToKeep.begin(), blendedMeshFacesToKeep.end()), newMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/preservedMesh.obj", preservedMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMeshNonSnapped.obj", newMesh);

//    //Get the border vertex chains for the two meshes
//    std::vector<std::vector<VertexId>> preChains = nvl::meshBorderVertexChains(preservedMesh);
//    std::vector<std::vector<VertexId>> newChains = nvl::meshBorderVertexChains(newMesh);

//    //Snappable preserved chains: we do not snap the chains which were borders in the original meshes
//    std::vector<bool> preservedSnappableChain(preChains.size(), true);
//    for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//        const std::vector<VertexId>& chain = preChains[pChainId];
//        for (const VertexId& vId : chain) {
//            if (preservedVerticesAlreadyBorders.find(vId) != preservedVerticesAlreadyBorders.end()) {
//                preservedSnappableChain[pChainId] = false;
//            }
//        }
//    }

//    //Sort new chain by sizes
//    std::sort(newChains.begin(), newChains.end(), [](const std::vector<VertexId>& lhs, const std::vector<VertexId>& rhs)
//    {
//        return lhs.size() > rhs.size();
//    });
//    //Snappable new surface chains: we do not snap chains with less than 10 vertices
//    std::vector<bool> newSnappableChain(newChains.size(), true);
//    for (Index nChainId = 0; nChainId < newChains.size(); ++nChainId) {
//        const std::vector<VertexId>& nChain = newChains[nChainId];
//        if (nChain.size() <= MIN_CHAIN) {
//            newSnappableChain[nChainId] = false;
//        }
//    }

//    for (Index nChainId = 0; nChainId < newChains.size(); ++nChainId) {
//        if (!newSnappableChain[nChainId])
//            continue;

//        const std::vector<VertexId>& nChain = newChains[nChainId];

//        VertexId bestI = nvl::MAX_ID;
//        VertexId bestJ = nvl::MAX_ID;
//        VertexId bestPChainId = nvl::MAX_ID;
//        Scalar bestScore = nvl::maxLimitValue<Scalar>();

//        for (Index pChainId = 0; pChainId < preChains.size(); ++pChainId) {
//            if (!preservedSnappableChain[pChainId])
//                continue;
//            const std::vector<VertexId>& pChain = preChains[pChainId];


//            for (Index i = 0; i < nChain.size(); ++i) {
//                const Vertex& nVertex = newMesh.vertex(nChain[i]);

//                for (Index j = 0; j < pChain.size(); ++j) {
//                    const Vertex& pVertex = preservedMesh.vertex(pChain[j]);

//                    Scalar dist = (nVertex.point() - pVertex.point()).norm();

//                    Scalar score = dist;

//                    if (score <= bestScore) {
//                        bestScore = score;
//                        bestI = i;
//                        bestJ = j;
//                        bestPChainId = pChainId;
//                    }
//                }
//            }
//        }

//        if (bestPChainId < nvl::MAX_ID) {
//            const std::vector<VertexId>& nChain = newChains[nChainId];
//            const std::vector<VertexId>& pChain = preChains[bestPChainId];

//            //Add border vertices in the new mesh
//            std::map<VertexId, VertexId> newVertexMap;
//            for (Index j = 0; j < pChain.size(); ++j) {
//                if (newVertexMap.find(pChain[j]) == newVertexMap.end()) {
//                    const Vertex& targetVertex = preservedMesh.vertex(pChain[j]);
//                    newVertexMap.insert(std::make_pair(pChain[j], newMesh.addVertex(targetVertex.point())));
//                }
//            }

//            const unsigned int factor = nChain.size() / pChain.size();

//            Index i = bestI;
//            Index j = bestJ;

//            bool isIncrementableI = true;
//            bool isIncrementableJ = true;

//            unsigned int numConsecutive = 0;

//            do {
//                VertexId nextI = (i + 1) % nChain.size();
//                VertexId nextJ = (j > 0 ? j - 1 : pChain.size() - 1);

//                if (isIncrementableJ) {
//                    Vertex& newVertex = newMesh.vertex(nChain[i]);

//                    const Vertex& preservedVertex = preservedMesh.vertex(pChain[j]);
//                    const Vertex& nextPreservedVertex = preservedMesh.vertex(pChain[nextJ]);

//                    Scalar currentDist = (newVertex.point() - preservedVertex.point()).norm();
//                    Scalar nextDist = (newVertex.point() - nextPreservedVertex.point()).norm();

//                    Scalar multiplier = 1;
//                    if (numConsecutive > factor) {
//                        multiplier += (numConsecutive - factor) * 0.05;
//                    }

//                    Scalar currentScore = currentDist * multiplier;
//                    Scalar nextScore = nextDist;

//                    if (!isIncrementableI || nextScore <= currentScore) {
//                        newMesh.addFace(nChain[i], newVertexMap.at(pChain[j]), newVertexMap.at(pChain[nextJ]));

//                        j = nextJ;
//                        isIncrementableJ = j != bestJ;

//                        numConsecutive = 0;
//                    }
//                    else {
//                        numConsecutive++;
//                    }
//                }

//                newMesh.addFace(nChain[nextI], nChain[i], newVertexMap.at(pChain[j]));

//                if (isIncrementableI) {
//                    i = nextI;
//                    isIncrementableI = i != bestI;
//                }
//            } while (isIncrementableI || isIncrementableJ);

//            preservedSnappableChain[bestPChainId] = false;
//            newSnappableChain[nChainId] = false;
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMesh.obj", newMesh);






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    //Snap maps
//    std::vector<std::pair<Index, VertexId>> blendedSnapMap(blendedMesh.nextVertexId(), std::make_pair(nvl::MAX_ID, nvl::MAX_ID));
//    std::vector<std::vector<std::set<VertexId>>> preservedSnapMap(meshes.size());
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];
//        preservedSnapMap[mId].resize(meshPtr->nextVertexId());
//    }

//    //Snap vertices of the triangles on the preserved vertices
//    for (VertexId bVId = 0; bVId < blendedMesh.nextVertexId(); ++bVId) {
//        if (blendedMesh.isVertexDeleted(bVId))
//            continue;

//        Point point = blendedMesh.vertex(bVId).point();

//        VertexId bestVId = nvl::MAX_ID;
//        Scalar bestDistance = nvl::maxLimitValue<Scalar>();
//        Index bestMId = nvl::MAX_ID;

//        for (Index mId = 0; mId < meshes.size(); ++mId) {
//            const Mesh* meshPtr = meshes[mId];

//            IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

//            IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

//            Point scaledPoint = scaleTransform * point;
//            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

//            IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

//            if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>()) {
//                FaceId mFId = gridBirthFace[mId][currentPolygonIndex];

//                float fuzzyValue = 0.0;
//                for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                    fuzzyValue += vertexFuzzyValue[mId][mVId];
//                }
//                fuzzyValue /= meshPtr->face(mFId).vertexNumber();

//                //TODO CONTROLLO DIST?
//                if (fuzzyValue > KEEP_THRESHOLD) {
//                    for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                        Point targetPoint = meshPtr->vertex(mVId).point();

//                        Scalar dist = (targetPoint - point).norm();

//                        if (dist <= bestDistance) {
//                            bestVId = mVId;
//                            bestDistance = dist;
//                            bestMId = mId;
//                        }
//                    }
//                }
//            }
//        }

//        if (bestDistance < nvl::maxLimitValue<Scalar>()) {
//            Point bestPoint = meshes[bestMId]->vertex(bestVId).point();

//            blendedMesh.vertex(bVId).setPoint(bestPoint);

//            blendedSnapMap[bVId] = std::make_pair(bestMId, bestVId);
//            preservedSnapMap[bestMId][bestVId].insert(bVId);
//        }
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/snappedMesh.obj", blendedMesh);

//    //Find vertices and faces to keep in the blended mesh
//    std::set<FaceId> blendedMeshFacesToKeep;
//    std::set<VertexId> blendedMeshVerticesNotSnappedVertices;
//    for (FaceId bFId = 0; bFId < blendedMesh.nextFaceId(); ++bFId) {
//        if (blendedMesh.isFaceDeleted(bFId))
//            continue;

//        bool isFaceKept = false;

//        const Face& bFace = blendedMesh.face(bFId);

//        for (const VertexId& bVId : bFace.vertexIds()) {
//            const Index& pVId = blendedSnapMap[bVId].second;

//            //If a vertex has not been snapped
//            if (pVId == nvl::MAX_ID) {
//                isFaceKept = true;
//                blendedMeshVerticesNotSnappedVertices.insert(bVId);
//            }
//        }

//        if (isFaceKept) {
//            blendedMeshFacesToKeep.insert(bFId);
//        }
//    }

//    //Find vertices and faces to keep in the original meshes
//    std::vector<std::set<FaceId>> preservedMeshFacesToKeep(meshes.size());
//    std::vector<std::set<VertexId>> preservedMeshVerticesToKeep(meshes.size());
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];

//        for (FaceId mFId = 0; mFId < meshPtr->nextFaceId(); ++mFId) {
//            if (meshPtr->isFaceDeleted(mFId))
//                continue;

//            float fuzzyValue = 0.0;
//            for (VertexId mVId : meshPtr->face(mFId).vertexIds()) {
//                fuzzyValue += vertexFuzzyValue[mId][mVId];
//            }
//            fuzzyValue /= meshPtr->face(mFId).vertexNumber();

//            if (fuzzyValue > KEEP_THRESHOLD) {
//                preservedMeshFacesToKeep[mId].insert(mFId);
//            }
//        }
//    }

//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];

//        for (FaceId mFId = 0; mFId < meshPtr->nextFaceId(); ++mFId) {
//            if (meshPtr->isFaceDeleted(mFId))
//                continue;

//            const Face& mFace = meshPtr->face(mFId);

//            bool isFaceKept = false;

//            for (const VertexId& mVId : mFace.vertexIds()) {
//                for (const VertexId& bVId : preservedSnapMap[mId][mVId]) {
//                    if (blendedMeshVerticesNotSnappedVertices.find(bVId) == blendedMeshVerticesNotSnappedVertices.end()) {
//                        isFaceKept = true;
//                    }
//                }
//            }

//            if (!isFaceKept) {
//                preservedMeshFacesToKeep[mId].erase(mFId);
//            }
//        }
//    }

//    for (FaceId bFId = 0; bFId < blendedMesh.nextFaceId(); ++bFId) {
//        if (blendedMesh.isFaceDeleted(bFId))
//            continue;

//        const Face& bFace = blendedMesh.face(bFId);

//        for (const VertexId& bVId : bFace.vertexIds()) {
//            const Index& pVId = blendedSnapMap[bVId].second;

//            //If a vertex has not been snapped
//            if (pVId == nvl::MAX_ID) {

//            }
//        }
//    }


//    //Transfer in the new surface the faces to be kept
//    newMesh.clear();
//    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(blendedMeshFacesToKeep.begin(), blendedMeshFacesToKeep.end()), newMesh);

//    //Create preserved mesh from the vertices which have been found in the blended mesh
//    preservedMesh.clear();
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        Mesh* meshPtr = meshes[mId];

//        nvl::meshTransferFaces(*meshPtr, std::vector<VertexId>(preservedMeshFacesToKeep[mId].begin(), preservedMeshFacesToKeep[mId].end()), preservedMesh, preservedBirthVertices, preservedBirthFaces);
//        preservedVerticesBirthModel.resize(preservedBirthVertices.size(), mId);
//        preservedFacesBirthModel.resize(preservedBirthFaces.size(), mId);
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/preservedMesh.obj", preservedMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMesh.obj", newMesh);





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    //Fill vertices to keep for the original meshes
//    std::vector<std::unordered_set<FaceId>> meshFacesToKeep(meshes.size());
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        const Mesh* meshPtr = meshes[mId];
//        for (FaceId fId = 0; fId < meshPtr->nextFaceId(); ++fId) {
//            if (meshPtr->isFaceDeleted(fId))
//                continue;

//            float fuzzyValue = 0.0;
//            for (VertexId j : meshPtr->face(fId).vertexIds()) {
//                fuzzyValue += vertexFuzzyValue[mId][j];
//            }
//            fuzzyValue /= meshPtr->face(fId).vertexNumber();

//            if (fuzzyValue > KEEP_THRESHOLD) {
//                meshFacesToKeep[mId].insert(fId);
//            }
//        }
//    }

//    //Fill vertices to keep in the blended mesh
//    std::unordered_set<FaceId> blendedMeshFacesToKeep;
//    for (FaceId fId = 0; fId < blendedMesh.nextFaceId(); ++fId) {
//        if (blendedMesh.isFaceDeleted(fId))
//            continue;

//        bool isNewSurface = true;

//        for (VertexId vId : blendedMesh.face(fId).vertexIds()) {
//            const Point& point = blendedMesh.vertex(vId).point();

//            Point scaledPoint = scaleTransform * point;
//            openvdb::math::Coord coord(std::round(scaledPoint.x()), std::round(scaledPoint.y()), std::round(scaledPoint.z()));

//            for (Index mId = 0; mId < meshes.size(); ++mId) {
//                FloatGridPtr& currentGrid = signedGrids[mId];
//                IntGridPtr& currentPolygonIndexGrid = polygonIndexGrid[mId];

//                FloatGrid::ConstAccessor currentAccessor = currentGrid->getConstAccessor();
//                IntGrid::ConstAccessor currentPolygonIndexAccessor = currentPolygonIndexGrid->getConstAccessor();

//                FloatGrid::ValueType currentValue = currentAccessor.getValue(coord);
//                IntGrid::ValueType currentPolygonIndex = currentPolygonIndexAccessor.getValue(coord);

//                if (currentPolygonIndex >= 0 && currentPolygonIndex < nvl::maxLimitValue<int>() && currentValue <= 2.0) {
//                    FaceId meshOriginFId = gridBirthFace[mId][currentPolygonIndex];

//                    if (meshFacesToKeep[mId].find(meshOriginFId) != meshFacesToKeep[mId].end()) {
//                        isNewSurface = false;
//                    }
//                }
//            }
//        }

//        if (isNewSurface) {
//            blendedMeshFacesToKeep.insert(fId);
//        }
//    }

//    //Create preserved mesh
//    std::unordered_set<VertexId> preservedVerticesAlreadyBorders;
//    for (Index mId = 0; mId < meshes.size(); ++mId) {
//        Mesh* meshPtr = meshes[mId];

//        VertexId startVertexId = preservedMesh.nextVertexId();

//        nvl::meshTransferFaces(*meshPtr, std::vector<VertexId>(meshFacesToKeep[mId].begin(), meshFacesToKeep[mId].end()), preservedMesh, preservedBirthVertices, preservedBirthFaces);
//        preservedVerticesBirthModel.resize(preservedBirthVertices.size(), mId);
//        preservedFacesBirthModel.resize(preservedBirthFaces.size(), mId);

//        std::vector<VertexId> borderVertices = nvl::meshBorderVertices(*meshPtr);
//        std::unordered_set<VertexId> borderVerticesSet(borderVertices.begin(), borderVertices.end());
//        for (VertexId vId = startVertexId; vId < preservedMesh.nextVertexId(); vId++) {
//            if (borderVerticesSet.find(preservedBirthVertices[vId]) != borderVerticesSet.end()) {
//                preservedVerticesAlreadyBorders.insert(vId);
//            }
//        }
//    }

//    //Transfer in the new surface the faces to be kept
//    nvl::meshTransferFaces(blendedMesh, std::vector<VertexId>(blendedMeshFacesToKeep.begin(), blendedMeshFacesToKeep.end()), newMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/preservedMesh.obj", preservedMesh);

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMeshNonSnapped.obj", newMesh);

//    //Get the borders
//    std::vector<VertexId> preservedBorders = nvl::meshBorderVertices(preservedMesh);
//    std::vector<VertexId> newBorders = nvl::meshBorderVertices(newMesh);

//    std::vector<VertexId> newSnapMap(newMesh.nextVertexId(), nvl::MAX_ID);
//    std::vector<std::vector<VertexId>> preservedSnapMap(preservedMesh.nextVertexId());
//    for (const VertexId& nVId : newBorders) {
//        const Vertex& nVertex = newMesh.vertex(nVId);

//        VertexId bestNVId = nvl::MAX_ID;
//        VertexId bestPVId = nvl::MAX_ID;
//        Scalar bestDist = nvl::maxLimitValue<Scalar>();

//        for (const VertexId& pVId : preservedBorders) {
//            if (preservedVerticesAlreadyBorders.find(pVId) != preservedVerticesAlreadyBorders.end()) {
//                continue;
//            }

//            const Vertex& pVertex = preservedMesh.vertex(pVId);

//            Scalar dist = (nVertex.point() - pVertex.point()).norm();

//            if (dist <= bestDist) {
//                bestDist = dist;
//                bestNVId = nVId;
//                bestPVId = pVId;
//            }
//        }

//        assert (bestDist < nvl::maxLimitValue<Scalar>());
//        Vertex& newVertex = newMesh.vertex(bestNVId);
//        Vertex& preservedVertex = preservedMesh.vertex(bestPVId);

//        newVertex.setPoint(preservedVertex.point());

//        newSnapMap[bestNVId] = bestPVId;
//        preservedSnapMap[bestPVId].push_back(bestNVId);
//    }

//    //TODO DELETE
//    nvl::meshSaveToFile("results/newMesh.obj", newMesh);

