#ifndef SKINMIXER_MORPHOLOGICAL_OPERATORS_H
#define SKINMIXER_MORPHOLOGICAL_OPERATORS_H

#include <nvl/nuvolib.h>

#include <vector>

namespace skinmixer {

template<class Mesh, class Set>
void meshDilateFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces);

template<class Mesh, class Set>
void meshDilateFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj);

template<class Mesh, class Set>
void meshErodeFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces);

template<class Mesh, class Set>
void meshErodeFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj);

template<class Mesh, class Set>
void meshOpenFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces);

template<class Mesh, class Set>
void meshOpenFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj);

template<class Mesh, class Set>
void meshCloseFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces);

template<class Mesh, class Set>
void meshCloseFaceSelectionNoBorders(
        const Mesh& mesh,
        Set& selectedFaces,
        const std::vector<std::vector<typename Mesh::FaceId>>& ffAdj);

}

#include "skinmixer_morphological_operations.cpp"


#endif // SKINMIXER_MORPHOLOGICAL_OPERATORS_H
