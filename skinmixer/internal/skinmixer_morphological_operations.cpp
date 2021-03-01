#include "skinmixer_morphological_operations.h"

#include <nvl/models/mesh_adjacencies.h>
#include <nvl/models/mesh_borders.h>

namespace skinmixer {

template<class Mesh, class Set>
void meshDilateFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces)
{
    typedef typename Mesh::FaceId FaceId;

    const std::vector<std::vector<FaceId>> ffAdj = meshFaceFaceAdjacencies(mesh);
    return meshDilateFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
}

template<class Mesh, class Set>
void meshDilateFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj)
{
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::VertexId VertexId;

    std::unordered_set<VertexId> verticesOnBorder;
    for (const FaceId& fId : selectedFaces) {
        assert(!mesh.isFaceDeleted(fId));

        const Face& face = mesh.face(fId);
        for (nvl::Index fePos = 0; fePos < face.vertexNumber(); ++fePos) {
            if (!meshIsBorderFaceEdge(mesh, fId, fePos, ffAdj) && selectedFaces.find(ffAdj[fId][fePos]) == selectedFaces.end()) {
                verticesOnBorder.insert(face.vertexId(fePos));
                verticesOnBorder.insert(face.nextVertexId(fePos));
            }
        }
    }

    for (FaceId fId = 0; fId < mesh.nextFaceId(); ++fId) {
        if (mesh.isFaceDeleted(fId))
            continue;

        const Face& face = mesh.face(fId);

        bool inserted = false;
        for (nvl::Index fePos = 0; fePos < face.vertexNumber() && !inserted; ++fePos) {
            const VertexId& vId = face.vertexId(fePos);

            if (verticesOnBorder.find(vId) != verticesOnBorder.end()) {
                selectedFaces.insert(fId);
                inserted = true;
            }
        }
    }
}

template<class Mesh, class Set>
void meshErodeFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces)
{
    typedef typename Mesh::FaceId FaceId;

    const std::vector<std::vector<FaceId>> ffAdj = meshFaceFaceAdjacencies(mesh);
    return meshErodeFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
}

template<class Mesh, class Set>
void meshErodeFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj)
{
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::VertexId VertexId;

    std::unordered_set<VertexId> verticesNonSurrounded;
    for (const FaceId& fId : selectedFaces) {
        assert(!mesh.isFaceDeleted(fId));

        const Face& face = mesh.face(fId);
        for (nvl::Index fePos = 0; fePos < face.vertexNumber(); ++fePos) {
            if (!meshIsBorderFaceEdge(mesh, fId, fePos, ffAdj) && selectedFaces.find(ffAdj[fId][fePos]) == selectedFaces.end()) {
                verticesNonSurrounded.insert(face.vertexId(fePos));
                verticesNonSurrounded.insert(face.nextVertexId(fePos));
            }
        }
    }

    typename Set::iterator it = selectedFaces.begin();
    while (it != selectedFaces.end()) {
        const FaceId& fId = *it;
        assert(!mesh.isFaceDeleted(fId));

        const Face& face = mesh.face(fId);

        bool toDelete = false;
        for (nvl::Index fePos = 0; fePos < face.vertexNumber() && !toDelete; ++fePos) {
            if (verticesNonSurrounded.find(face.vertexId(fePos)) != verticesNonSurrounded.end()) {
                toDelete = true;
            }
        }

        if (toDelete)
            it = selectedFaces.erase(it);
        else
            it++;
    }
}

template<class Mesh, class Set>
void meshOpenFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces)
{
    typedef typename Mesh::FaceId FaceId;

    const std::vector<std::vector<FaceId>> ffAdj = meshFaceFaceAdjacencies(mesh);
    return meshOpenFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
}

template<class Mesh, class Set>
void meshOpenFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj)
{
    meshErodeFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
    meshDilateFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
}

template<class Mesh, class Set>
void meshCloseFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces)
{
    typedef typename Mesh::FaceId FaceId;

    const std::vector<std::vector<FaceId>> ffAdj = meshFaceFaceAdjacencies(mesh);
    return meshCloseFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
}

template<class Mesh, class Set>
void meshCloseFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj)
{
    meshDilateFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
    meshErodeFaceSelectionNoBorders(mesh, selectedFaces, ffAdj);
}

}
