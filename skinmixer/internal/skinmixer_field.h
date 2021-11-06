#ifndef SKINMIXER_OPENVDB_BLENDING_H
#define SKINMIXER_OPENVDB_BLENDING_H

#include <nvl/nuvolib.h>

#include <utility>
#include <unordered_set>

#ifdef foreach
  #undef foreach
#endif
#include <openvdb/openvdb.h>

#include <nvl/math/scaling.h>

#include "skinmixer/skinmixer_data.h"

namespace skinmixer {
namespace internal {

template<class Mesh>
struct OpenVDBAdapter {

public:
    OpenVDBAdapter() {

    }

    OpenVDBAdapter(const Mesh& mesh) {
        this->setMesh(mesh, 1.0);
    }

    OpenVDBAdapter(const Mesh& mesh, const double& scaleFactor) {
        this->setMesh(mesh, scaleFactor);
    }

    void setMesh(const Mesh& mesh) {
        this->setMesh(mesh, 1.0);
    }

    void setMesh(const Mesh& mesh, const double& scaleFactor) {
        typedef typename Mesh::Face Face;
        typedef typename Mesh::Vertex Vertex;
        typedef typename Mesh::Point Point;

        for (const Vertex& vertex : mesh.vertices()) {
            const Point& p = vertex.point();
            this->vertices.push_back(openvdb::Vec3d(p.x() / scaleFactor, p.y() / scaleFactor, p.z() / scaleFactor));
        }
        for (const Face& face : mesh.faces()) {
            this->faces.push_back(face.vertexIds());
        }
    }


    size_t polygonCount() const { return this->faces.size(); } // Total number of polygons
    size_t pointCount() const { return this->vertices.size(); } // Total number of points
    size_t vertexCount(size_t n) const { return this->faces[n].size(); } // Vertex count for polygon n
    // Return position pos in local grid index space for polygon n and vertex v
    void getIndexSpacePoint(size_t n, size_t v, openvdb::Vec3d& pos) const {
        pos = this->vertices[this->faces[n][v]];
    }

private:
    std::vector<openvdb::Vec3R> vertices;
    std::vector<std::vector<nvl::Index>> faces;
};


template<class Mesh>
void getClosedGrid(
        const Mesh& inputMesh,
        const double& scaleFactor,
        const double& maxDistance,
        Mesh& closedMesh,
        openvdb::FloatGrid::Ptr& closedGrid,
        openvdb::Int32Grid::Ptr& polygonGrid);

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
        const double& scaleFactor,
        const double& maxDistance,
        openvdb::FloatGrid::Ptr& blendedGrid,
        openvdb::Int32Grid::Ptr& activeActionGrid);

template<class Mesh>
std::unordered_set<typename Mesh::FaceId> findFieldFaces(
        const Mesh& mesh,
        const std::vector<double>& vertexSelectValues,
        const double& scaleFactor);

template<class Model>
typename Model::Mesh getRemoveDetachMesh(
        const std::vector<const Model*>& models,
        const std::vector<const std::vector<double>*>& vertexSelectValue,
        const double& scaleFactor,
        const double& maxDistance,
        const std::vector<openvdb::FloatGrid::Ptr>& closedGrids,
        const std::vector<openvdb::Int32Grid::Ptr>& polygonGrids,
        const std::vector<std::vector<typename Model::Mesh::FaceId>>& fieldBirthFace,
        const openvdb::math::Coord& minCoord,
        const openvdb::math::Coord& maxCoord);

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
Mesh convertGridToMesh(const openvdb::FloatGrid::Ptr& gridPtr, bool transformQuadsToTriangles);


}
}

#include "skinmixer_field.cpp"

#endif // SKINMIXER_OPENVDB_BLENDING_H
