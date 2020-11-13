#ifndef SKINMIXER_OPENVDB_BLENDING_H
#define SKINMIXER_OPENVDB_BLENDING_H

#include <nvl/nuvolib.h>

#include <utility>

#ifdef foreach
  #undef foreach
#endif
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

#include <nvl/math/scaling.h>

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

template<class Model>
typename Model::Mesh getBlendedMesh(
        const std::vector<Model*>& models,
        const std::vector<std::vector<double>>& vertexSelectValue,
        const nvl::Scaling3d& scaleTransform,
        const double maxDistance,
        std::vector<FloatGridPtr>& signedGrids,
        std::vector<FloatGridPtr>& closedGrids,
        std::vector<IntGridPtr>& polygonGrids,
        std::vector<std::vector<typename Model::Mesh::VertexId>>& gridBirthVertex,
        std::vector<std::vector<typename Model::Mesh::FaceId>>& gridBirthFace);

template<class Mesh>
Mesh convertGridToMesh(const FloatGridPtr& gridPtr, bool transformQuadsToTriangles);


}
}

#include "skinmixer_openvdb_blending.cpp"

#endif // SKINMIXER_OPENVDB_BLENDING_H
