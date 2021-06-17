#ifndef SKINMIXER_OPENVDB_BLENDING_H
#define SKINMIXER_OPENVDB_BLENDING_H

#include <nvl/nuvolib.h>

#include <utility>
#include <unordered_set>

#ifdef foreach
  #undef foreach
#endif
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

#include <nvl/math/scaling.h>

#include "skinmixer/skinmixer_data.h"

namespace skinmixer {
namespace internal {

typedef typename openvdb::FloatGrid FloatGrid;
typedef typename FloatGrid::Ptr FloatGridPtr;
typedef typename openvdb::Int32Grid IntGrid;
typedef typename IntGrid::Ptr IntGridPtr;
typedef typename openvdb::math::Coord GridCoord;

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
void getClosedGrid(
        const Mesh& inputMesh,
        const double& maxDistance,
        Mesh& closedMesh,
        FloatGridPtr& closedGrid,
        IntGridPtr& polygonGrid,
        openvdb::Vec3i& bbMin,
        openvdb::Vec3i& bbMax);

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
        std::vector<openvdb::Vec3i>& bbMax);

template<class Model>
void getBlendedGrid(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& actions,
        const std::vector<std::vector<const Model*>>& models,
        const std::vector<std::vector<const std::vector<double>*>>& vertexSelectValue,
        const std::vector<std::vector<FloatGridPtr>>& closedGrids,
        const std::vector<std::vector<IntGridPtr>>& polygonGrids,
        const std::vector<std::vector<std::vector<typename Model::Mesh::FaceId>>>& gridBirthFace,
        const std::vector<std::vector<openvdb::Vec3i>>& bbMin,
        const std::vector<std::vector<openvdb::Vec3i>>& bbMax,
        const double& scaleFactor,
        const double& maxDistance,
        FloatGridPtr& blendedGrid,
        IntGridPtr& activeActionGrid);

template<class Mesh>
std::unordered_set<typename Mesh::FaceId> findFacesInField(
        const Mesh& mesh,
        const std::vector<double>& vertexSelectValues,
        const double& scaleFactor);

template<class Model>
typename Model::Mesh getRemoveDetachMesh(
        const std::vector<const Model*>& models,
        const std::vector<const std::vector<double>*>& vertexSelectValue,
        const double& scaleFactor,
        const double& maxDistance,
        const std::vector<FloatGridPtr>& closedGrids,
        const std::vector<IntGridPtr>& polygonGrids,
        const std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace,
        const openvdb::Vec3i& minCoord,
        const openvdb::Vec3i& maxCoord);

template<class Mesh>
double averageFaceSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const std::vector<double>& vertexSelectValue);

template<class Mesh>
double interpolateFaceSelectValue(
        const Mesh& mesh,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point,
        const std::vector<double>& vertexSelectValue);

template<class Mesh>
Mesh convertGridToMesh(const FloatGridPtr& gridPtr, bool transformQuadsToTriangles);


}
}

#include "skinmixer_field.cpp"

#endif // SKINMIXER_OPENVDB_BLENDING_H
