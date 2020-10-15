#include "quad_steps.h"

#include "quad_convert.h"
#include "quad_patch_tracer.h"
#include "quad_utils.h"
#include "quad_patterns.h"
#include "quad_mapping.h"
#include "quad_patch_assembler.h"

#include <map>

#include <vcg/complex/algorithms/polygonal_algorithms.h>

#ifdef SAVE_MESHES_FOR_DEBUG
#include <igl/writeOBJ.h>
#endif

namespace QuadBoolean {
namespace internal {

template<class TriangleMeshType, class PolyMeshType>
std::vector<int> getPatchDecomposition(
        TriangleMeshType& newSurface,
        PolyMeshType& preservedSurface,
        std::vector<std::vector<size_t>>& partitions,
        std::vector<std::vector<size_t>>& corners,
        const bool initialRemeshing,
        const double edgeFactor,
        const bool reproject,
        const bool splitConcaves,
        const bool finalSmoothing)
{
    if (newSurface.face.size() <= 0)
        return std::vector<int>();

    std::vector<std::vector<std::vector<std::pair<size_t,size_t>>>> sides;
    PatchAssembler<TriangleMeshType, PolyMeshType> patchAssembler(newSurface, preservedSurface);
    typename PatchAssembler<TriangleMeshType, PolyMeshType>::Parameters parameters;
    parameters.InitialRemesh = initialRemeshing;
    parameters.EdgeSizeFactor = edgeFactor;
    parameters.FinalSmooth = finalSmoothing;
    parameters.SplitAllConcave = splitConcaves;
    parameters.Reproject = reproject;
    patchAssembler.BatchProcess(partitions, corners, sides, parameters);

    std::vector<int> newSurfaceLabel(newSurface.face.size(), -1);
    for (size_t pId = 0; pId < partitions.size(); pId++) {
        for (const size_t& fId : partitions[pId]) {
            assert(newSurfaceLabel[fId] == -1);
            newSurfaceLabel[fId] = static_cast<int>(pId);
        }
    }

    return newSurfaceLabel;
}

inline std::vector<int> findSubdivisions(
        ChartData& chartData,
        const double alpha,
        const ILPMethod& method)
{
    if (chartData.charts.size() <= 0)
        return std::vector<int>();

#ifdef SAVE_MESHES_FOR_DEBUG
    const double timeLimit = 30.0;
#else
    const double timeLimit = 5.0;
#endif
    const double gapLimit = 0.3;

    //Fix the chart on border
    for (ChartSubSide& subside : chartData.subSides) {
        subside.isFixed = subside.isOnBorder;
    }

    double gap;
    ILPStatus status;

    //Solve ILP to find the best patches
    std::vector<int> ilpResult = solveILP(chartData, alpha, method, true, timeLimit, gap, status);

    if (status == ILPStatus::SOLUTIONFOUND && gap <= gapLimit) {
        std::cout << "Solution found! Gap: " << gap << std::endl;
    }
    else {
        if (status == ILPStatus::INFEASIBLE) {
            std::cout << "Error! Model was infeasible or time limit exceeded!" << std::endl;
        }
        else {
            assert(gap > gapLimit);
            ilpResult = solveILP(chartData, alpha, ILPMethod::ABS, true, timeLimit*2, gap, status);

            if (status == ILPStatus::SOLUTIONFOUND) {
                std::cout << "Solution found (ABS)! Gap: " << gap << std::endl;
            }
            else {
                ilpResult = solveILP(chartData, alpha, ILPMethod::ABS, false, timeLimit*4, gap, status);
                std::cout << "Solution found? (ABS without regularity)! Gap: " << gap << std::endl;
            }
        }
    }

    return ilpResult;
}


template<class TriangleMeshType, class PolyMeshType>
void quadrangulate(
        TriangleMeshType& newSurface,
        ChartData& chartData,
        const std::vector<int>& ilpResult,
        const int chartSmoothingIterations,
        const int quadrangulationSmoothingIterations,
        PolyMeshType& quadrangulation,
        std::vector<int>& quadrangulatedNewSurfaceLabel)
{
    if (newSurface.face.size() <= 0)
        return;
    if (ilpResult.size() == 0)
        return;

    std::vector<std::vector<size_t>> subsideVertexMap(chartData.subSides.size());
    std::vector<int> cornerVertices(newSurface.vert.size(), -1);

    //Fill fixed vertices (subsides corners)
    for (const ChartSubSide& subside : chartData.subSides) {
        size_t vStart = subside.vertices[0];
        size_t vEnd = subside.vertices[subside.vertices.size() - 1];

        if (cornerVertices[vStart] == -1) {
            cornerVertices[vStart] = quadrangulation.vert.size();
            vcg::tri::Allocator<PolyMeshType>::AddVertex(
                        quadrangulation,
                        newSurface.vert[vStart].P());
        }

        if (cornerVertices[vEnd] == -1) {
            cornerVertices[vEnd] = quadrangulation.vert.size();
            vcg::tri::Allocator<PolyMeshType>::AddVertex(
                        quadrangulation,
                        newSurface.vert[vEnd].P());
        }
    }

    //Fill subside map for fixed borders
    std::set<size_t> finalMeshBorders;
    for (size_t subsideId = 0; subsideId < chartData.subSides.size(); subsideId++) {
        ChartSubSide& subside = chartData.subSides[subsideId];
        if (subside.isFixed) {
            for (size_t k = 0; k < subside.vertices.size(); k++) {
                const size_t& vId = subside.vertices[k];

                size_t newVertexId;

                if (cornerVertices[vId] == -1) {
                    assert(k > 0 && k < subside.vertices.size() - 1);

                    newVertexId = quadrangulation.vert.size();
                    vcg::tri::Allocator<PolyMeshType>::AddVertex(
                                quadrangulation,
                                newSurface.vert[vId].P());
                }
                else {
                    newVertexId = cornerVertices[vId];
                    assert(newVertexId >= 0);
                }

                finalMeshBorders.insert(newVertexId);
                subsideVertexMap[subsideId].push_back(newVertexId);
            }

            if (ilpResult[subsideId] > subside.size) {
                int vToSplit = -1;
                double maxLength = 0.0;
                for (size_t k = 0; k < subside.vertices.size() - 1; k++) {
                    const size_t& vId1 = subside.vertices[k];
                    const size_t& vId2 = subside.vertices[k + 1];
                    double length = (newSurface.vert[vId2].P() - newSurface.vert[vId1].P()).Norm();
                    if (length >= maxLength) {
                        vToSplit = k;
                    }
                }

                if (vToSplit >= 0) {
                    const size_t& vId1 = subside.vertices[vToSplit];
                    const size_t& vId2 = subside.vertices[vToSplit + 1];
                    size_t splitVertexId = quadrangulation.vert.size();
                    vcg::tri::Allocator<PolyMeshType>::AddVertex(
                                quadrangulation,
                                (newSurface.vert[vId1].P() + newSurface.vert[vId2].P()) / 2.0);
                    vcg::tri::Allocator<PolyMeshType>::AddFace(
                                quadrangulation, subsideVertexMap[subsideId][vToSplit], subsideVertexMap[subsideId][vToSplit + 1], splitVertexId);

                    subsideVertexMap[subsideId].insert(subsideVertexMap[subsideId].begin() + vToSplit + 1, splitVertexId);

                    std::cout << "Triangle added in subside " << subsideId << ": +1!" << std::endl;
                }
                else {
                    std::cout << "ERROR: impossible to augment the subside " << subsideId << ": +1! Target vertex not found." << std::endl;
                }
            }
            else if (ilpResult[subsideId] < subside.size) {
                if (subside.size >= 2) {
                    int vToSkip = -1;
                    double minLength = std::numeric_limits<double>::max();
                    for (size_t k = 0; k < subside.vertices.size() - 2; k++) {
                        const size_t& vId1 = subside.vertices[k];
                        const size_t& vId2 = subside.vertices[k + 1];
                        const size_t& vId3 = subside.vertices[k + 2];
                        double length = (newSurface.vert[vId2].P() - newSurface.vert[vId1].P()).Norm() + (newSurface.vert[vId3].P() - newSurface.vert[vId2].P()).Norm();
                        if (length <= minLength) {
                            vToSkip = k;
                        }
                    }

                    if (vToSkip >= 0) {
                        vcg::tri::Allocator<PolyMeshType>::AddFace(
                                    quadrangulation, subsideVertexMap[subsideId][vToSkip], subsideVertexMap[subsideId][vToSkip + 1], subsideVertexMap[subsideId][vToSkip + 2]);

                        subsideVertexMap[subsideId].erase(subsideVertexMap[subsideId].begin() + vToSkip + 1);

                        std::cout << "Triangle added in subside " << subsideId << ": -1!" << std::endl;
                    }
                    else {
                        std::cout << "ERROR: impossible to reduce the subside " << subsideId << ": -1! Target vertex not found." << std::endl;
                    }
                }
                else {
                    std::cout << "ERROR: impossible to reduce the subside " << subsideId << ": -1! Subside is less than 2." << std::endl;
                }
            }
        }
    }


    //For each chart
    for (size_t cId = 0; cId < chartData.charts.size(); cId++) {
        const Chart& chart = chartData.charts[cId];

        if (chart.faces.size() == 0)
            continue;

        const std::vector<ChartSide>& chartSides = chart.chartSides;
        if (chartSides.size() < 3 || chartSides.size() > 6) {
            std::cout << "Chart " << cId << " with corners less than 3 or greater than 6!" << std::endl;
            continue;
        }

        bool ilpSolvedForAll = true;
        for (size_t sId : chart.chartSubSides) {
            if (ilpResult[sId] < 0)
                ilpSolvedForAll = false;
        }

        if (!ilpSolvedForAll) {
            std::cout << "Chart " << cId << " not computed. ILP was not solved." << std::endl;
            continue;
        }

        //Input mesh
        Eigen::MatrixXd chartV;
        Eigen::MatrixXi chartF;
        vcg::tri::UpdateFlags<TriangleMeshType>::FaceClearS(newSurface);
        vcg::tri::UpdateFlags<TriangleMeshType>::VertexClearS(newSurface);
        for (const size_t& fId : chart.faces) {
            newSurface.face[fId].SetS();
            for (int k = 0; k < newSurface.face[fId].VN(); k++) {
                newSurface.face[fId].V(k)->SetS();
            }
        }
        std::vector<int> vMap, fMap;
        VCGToEigen(newSurface, chartV, chartF, vMap, fMap, true, 3);

        //Input subdivisions
        Eigen::VectorXi l(chartSides.size());

        std::vector<std::vector<double>> chartSideLength(chartSides.size());
        std::vector<std::vector<std::vector<size_t>>> chartSideVertices(chartSides.size());
        std::vector<std::vector<size_t>> chartSideSubdivision(chartSides.size());

        for (size_t i = 0; i < chartSides.size(); i++) {
            const ChartSide& chartSide = chartSides[i];

            chartSideLength[i].resize(chartSide.subsides.size());
            chartSideVertices[i].resize(chartSide.subsides.size());
            chartSideSubdivision[i].resize(chartSide.subsides.size());

            size_t targetSideSubdivision = 0;
            for (size_t j = 0; j < chartSide.subsides.size(); j++) {
                const size_t& subSideId = chartSides[i].subsides[j];
                const ChartSubSide& subSide = chartData.subSides[subSideId];

                targetSideSubdivision += ilpResult[subSideId];

                chartSideLength[i][j] = subSide.length;
                chartSideVertices[i][j] = subSide.vertices;
                chartSideSubdivision[i][j] = ilpResult[subSideId];

                if (chartSide.reversedSubside[j]) {
                    std::reverse(chartSideVertices[i][j].begin(), chartSideVertices[i][j].end());
                }

                for (size_t k = 0; k < chartSideVertices[i][j].size(); k++) {
                    size_t vId = chartSideVertices[i][j][k];
                    assert(vMap[vId] >= 0);
                    chartSideVertices[i][j][k] = vMap[vId];
                }

                if (ilpResult[subSideId] < 0) {
                    std::cout << "Error: ILP not valid" << std::endl;
                    return;
                }
            }

            l(static_cast<int>(i)) = targetSideSubdivision;
        }

        //Pattern quadrangulation
        Eigen::MatrixXd patchV;
        Eigen::MatrixXi patchF;
        std::vector<size_t> patchBorders;
        std::vector<size_t> patchCorners;
        QuadBoolean::internal::computePattern(l, patchV, patchF, patchBorders, patchCorners);


#ifdef SAVE_MESHES_FOR_DEBUG
        igl::writeOBJ(std::string("results/") + std::to_string(cId) + std::string("_patch.obj"), patchV, patchF);
#endif

        std::vector<std::vector<size_t>> patchEigenSides = getPatchSides(patchV, patchF, patchBorders, patchCorners, l);

        assert(chartSides.size() == patchCorners.size());
        assert(chartSides.size() == patchEigenSides.size());

#ifdef SAVE_MESHES_FOR_DEBUG
        igl::writeOBJ(std::string("results/") + std::to_string(cId) + std::string("_chart.obj"), chartV, chartF);
#endif

        //Compute quadrangulation
        Eigen::MatrixXd uvMapV;
        Eigen::MatrixXi uvMapF;
        Eigen::MatrixXd quadrangulationV;
        Eigen::MatrixXi quadrangulationF;
        QuadBoolean::internal::computeQuadrangulation(chartV, chartF, patchV, patchF, chartSideVertices, chartSideLength, chartSideSubdivision, patchEigenSides, uvMapV, uvMapF, quadrangulationV, quadrangulationF);

#ifdef SAVE_MESHES_FOR_DEBUG
        Eigen::MatrixXd uvMesh(uvMapV.rows(), 3);
        for (int i = 0; i < uvMapV.rows(); i++) {
            uvMesh(i, 0) = uvMapV(i, 0);
            uvMesh(i, 1) = uvMapV(i, 1);
            uvMesh(i, 2) = 0;
        }

        std::string uvFile = std::string("results/") + std::to_string(cId) + std::string("_uv.obj");
        igl::writeOBJ(uvFile, uvMesh, uvMapF);
#endif
        assert(chartV.rows() == uvMapV.rows());

        //Get polymesh
        PolyMeshType quadrangulatedChartMesh;
        eigenToVCG(quadrangulationV, quadrangulationF, quadrangulatedChartMesh, 4);

#ifdef SAVE_MESHES_FOR_DEBUG
        igl::writeOBJ(std::string("results/") + std::to_string(cId) + std::string("_quadrangulation.obj"), quadrangulationV, quadrangulationF);
#endif

        //Smoothing
        if (chartSmoothingIterations > 0) {
            vcg::tri::UpdateSelection<PolyMeshType>::VertexAll(quadrangulatedChartMesh);
            for (size_t vId : patchBorders) {
                quadrangulatedChartMesh.vert[vId].ClearS();
            }
            vcg::PolygonalAlgorithm<PolyMeshType>::LaplacianReproject(quadrangulatedChartMesh, chartSmoothingIterations, 0.5, true);
        }

        std::vector<int> currentVertexMap(quadrangulatedChartMesh.vert.size(), -1);

        //Map subsides on the vertices of the current mesh (create if necessary)
        for (size_t i = 0; i < chartSides.size(); i++) {
            const ChartSide& side = chartSides[i];
            const std::vector<size_t>& patchSide = patchEigenSides[i];

            size_t currentPatchSideVertex = 0;

            for (size_t j = 0; j < side.subsides.size(); j++) {
                const size_t& subSideId = side.subsides[j];
                const bool& reversed = side.reversedSubside[j];
                const ChartSubSide& subside = chartData.subSides[subSideId];

                //Create new vertices of the subsides
                if (subsideVertexMap[subSideId].empty()) {
                    assert(!subside.isFixed);

                    //Get fixed corners of the subside
                    size_t vStart = subside.vertices[0];
                    size_t vEnd = subside.vertices[subside.vertices.size() - 1];
                    assert(cornerVertices[vStart] >= 0 && cornerVertices[vEnd] >= 0);

                    currentVertexMap[patchSide[currentPatchSideVertex]] = cornerVertices[vStart];
                    currentVertexMap[patchSide[currentPatchSideVertex + ilpResult[subSideId]]] = cornerVertices[vEnd];

                    for (int k = 0; k <= ilpResult[subSideId]; k++) {
                        size_t patchSideVId = patchSide[currentPatchSideVertex];

                        if (currentVertexMap[patchSideVId] == -1) {
                            assert(k > 0 && k < ilpResult[subSideId]);

                            //Add new vertex
                            size_t newVertexId = quadrangulation.vert.size();

                            const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[patchSideVId].P();
                            vcg::tri::Allocator<PolyMeshType>::AddVertex(quadrangulation, coord);

                            currentVertexMap[patchSideVId] = newVertexId;

                            subsideVertexMap[subSideId].push_back(newVertexId);
                        }
                        else {
                            //Use the existing vertex
                            int existingVertexId = currentVertexMap[patchSideVId];
                            assert(existingVertexId >= 0);
                            subsideVertexMap[subSideId].push_back(existingVertexId);
                        }

                        currentPatchSideVertex++;
                    }

                    if (reversed) {
                        std::reverse(subsideVertexMap[subSideId].begin(), subsideVertexMap[subSideId].end());
                    }
                }
                //Set the existing vertices
                else {
                    assert(subsideVertexMap[subSideId].size() == ilpResult[subSideId] + 1);

                    for (int k = 0; k <= ilpResult[subSideId]; k++) {
                        int patchSideVId = patchSide[currentPatchSideVertex];

                        size_t subSideVertexIndex = reversed ? ilpResult[subSideId] - k : k;

                        currentVertexMap[patchSideVId] = subsideVertexMap[subSideId][subSideVertexIndex];

                        size_t existingVertexId = currentVertexMap[patchSideVId];

                        //If it is not a corner or if it is not on border
                        if (!subside.isFixed && k > 0 && k < ilpResult[subSideId]) {
                            //Average
                            const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[patchSideVId].P();
                            quadrangulation.vert[existingVertexId].P() =
                                    (coord + quadrangulation.vert[existingVertexId].P())/2;
                        }

                        currentPatchSideVertex++;
                    }
                }

                currentPatchSideVertex--;
            }

            assert(currentPatchSideVertex+1 == patchSide.size());
        }

        //Internal vertices
        for (size_t i = 0; i < quadrangulatedChartMesh.vert.size(); i++) {
            if (currentVertexMap[i] == -1) {
                size_t newId = quadrangulation.vert.size();

                const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[i].P();
                vcg::tri::Allocator<PolyMeshType>::AddVertex(quadrangulation, coord);

                currentVertexMap[i] = newId;
            }
        }

        //Set faces
        for (size_t i = 0; i < quadrangulatedChartMesh.face.size(); i++) {
            assert(quadrangulatedChartMesh.face[i].VN() == 4);

            size_t newFaceId = quadrangulation.face.size();

            vcg::tri::Allocator<PolyMeshType>::AddFaces(quadrangulation, 1);

            quadrangulation.face[newFaceId].Alloc(quadrangulatedChartMesh.face[i].VN());
            for (int j = 0; j < quadrangulatedChartMesh.face[i].VN(); j++) {
                int vId = currentVertexMap[vcg::tri::Index(quadrangulatedChartMesh, quadrangulatedChartMesh.face[i].V(j))];
                assert(vId >= 0);

                quadrangulation.face[newFaceId].V(j) = &quadrangulation.vert[vId];
            }
            quadrangulatedNewSurfaceLabel.push_back(chart.label);
        }
    }

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(quadrangulation);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(quadrangulation);
    OrientFaces<PolyMeshType>::AutoOrientFaces(quadrangulation);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(quadrangulation);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(quadrangulation);

    vcg::tri::UpdateNormal<TriangleMeshType>::PerFaceNormalized(newSurface);
    vcg::tri::UpdateNormal<TriangleMeshType>::PerVertexNormalized(newSurface);
    vcg::tri::UpdateBounding<TriangleMeshType>::Box(newSurface);

    vcg::GridStaticPtr<typename TriangleMeshType::FaceType,typename TriangleMeshType::FaceType::ScalarType> Grid;
    Grid.Set(newSurface.face.begin(),newSurface.face.end());

    //Reproject
    vcg::tri::UpdateBounding<PolyMeshType>::Box(quadrangulation);
    typename TriangleMeshType::ScalarType maxD=quadrangulation.bbox.Diag();
    typename TriangleMeshType::ScalarType minD=0;

    for (size_t i=0;i<quadrangulation.vert.size();i++)
    {
        typename TriangleMeshType::CoordType closestPT;
        typename TriangleMeshType::FaceType *f=
                vcg::tri::GetClosestFaceBase<TriangleMeshType>(
                    newSurface,
                    Grid,
                    quadrangulation.vert[i].P(),
                    maxD,minD,
                    closestPT);

        quadrangulation.vert[i].P()=closestPT;
    }

    if (quadrangulationSmoothingIterations > 0) {
        vcg::tri::UpdateSelection<PolyMeshType>::VertexAll(quadrangulation);
        for (const size_t& borderVertexId : finalMeshBorders) {
            quadrangulation.vert[borderVertexId].ClearS();
        }
        vcg::PolygonalAlgorithm<PolyMeshType>::template LaplacianReproject<TriangleMeshType>(quadrangulation, newSurface, quadrangulationSmoothingIterations, 0.7, 0.7, true);
    }

    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(quadrangulation);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(quadrangulation);
    vcg::tri::UpdateBounding<PolyMeshType>::Box(quadrangulation);
}


template<class PolyMeshType, class TriangleMeshType>
void getResult(
        PolyMeshType& preservedSurface,
        PolyMeshType& quadrangulatedNewSurface,
        PolyMeshType& result,
        TriangleMeshType& targetBoolean,
        const int resultSmoothingIterations,
        const double resultSmoothingNRing,
        const int resultSmoothingLaplacianIterations,
        const double resultSmoothingLaplacianNRing,
        std::unordered_map<size_t, size_t>& preservedFacesMap,
        std::unordered_map<size_t, size_t>& preservedVerticesMap)
{
    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearV(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(quadrangulatedNewSurface);
    for (size_t i = 0; i < quadrangulatedNewSurface.face.size(); i++) {
        if (!quadrangulatedNewSurface.face[i].IsD()) {
            quadrangulatedNewSurface.face[i].Q() = -1;
        }
    }

    for (size_t i = 0; i < preservedSurface.vert.size(); i++) {
        if (!preservedSurface.vert[i].IsD()) {
            preservedSurface.vert[i].Q() = i;
        }
    }
    for (size_t i = 0; i < quadrangulatedNewSurface.vert.size(); i++) {
        if (!quadrangulatedNewSurface.vert[i].IsD()) {
            quadrangulatedNewSurface.vert[i].Q() = -1;
        }
    }

    //Create result
    PolyMeshType tmpMesh;
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpMesh, preservedSurface);
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpMesh, quadrangulatedNewSurface);

#ifdef SAVE_MESHES_FOR_DEBUG
    vcg::tri::io::ExporterOBJ<PolyMeshType>::Save(tmpMesh, "results/tmpResult.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

//    vcg::tri::Clean<PolyMeshType>::MergeCloseVertex(tmpMesh, 0.0000001);

    int numDuplicateVertices = vcg::tri::Clean<PolyMeshType>::RemoveDuplicateVertex(tmpMesh);
    if (numDuplicateVertices > 0) {
        std::cout << "Removed " << numDuplicateVertices << " duplicate vertices." << std::endl;
    }
    int numUnreferencedVertices = vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(tmpMesh);
    if (numUnreferencedVertices > 0) {
        std::cout << "Removed " << numUnreferencedVertices << " unreferenced vertices." << std::endl;
    }

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(tmpMesh);

    int numNonManifoldFaces = vcg::tri::Clean<PolyMeshType>::RemoveNonManifoldFace(tmpMesh);
    if (numNonManifoldFaces > 0) {
        std::cout << "Removed " << numNonManifoldFaces << " non-manifold faces." << std::endl;
    }

//    int numHoles = vcg::tri::Hole<PolyMeshType>::template EarCuttingFill<vcg::tri::TrivialEar<PolyMeshType>>(result, result.face.size(), false);
//    if (numHoles > 0) {
//        std::cout << "Removed " << numHoles << " holes." << std::endl;
//    }

    int numDuplicateFaces = vcg::tri::Clean<PolyMeshType>::RemoveDuplicateFace(tmpMesh);
    if (numDuplicateFaces > 0) {
        std::cout << "Removed " << numDuplicateFaces << " duplicate faces." << std::endl;
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    vcg::tri::io::ExporterOBJ<PolyMeshType>::Save(tmpMesh, "results/tmpResultAfterMerge.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    result.Clear();
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, tmpMesh);

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(result);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    OrientFaces<PolyMeshType>::AutoOrientFaces(result);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(result);

    //Fill maps
    for (size_t i = 0; i < result.vert.size(); i++) {
        if (result.vert[i].IsD())
            continue;

        if (result.vert[i].Q() > -1) {
            preservedVerticesMap.insert(std::make_pair(i, static_cast<size_t>(result.vert[i].Q())));
        }
    }
    for (size_t i = 0; i < result.face.size(); i++) {
        if (result.face[i].IsD())
            continue;

        if (result.face[i].Q() > -1) {
            preservedFacesMap.insert(std::make_pair(i, static_cast<size_t>(result.face[i].Q())));
        }
    }

    //Get smoothing vertices
    std::vector<size_t> smoothingVertices;
    for (size_t i = 0; i < result.vert.size(); i++) {
        if (result.vert[i].IsD())
            continue;

        if (result.vert[i].Q() == -1) {
            smoothingVertices.push_back(i);
        }
    }

#ifdef SAVE_MESHES_FOR_DEBUG
    vcg::tri::io::ExporterOBJ<PolyMeshType>::Save(result, "results/resultBeforeReprojection.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    vcg::tri::UpdateNormal<TriangleMeshType>::PerFaceNormalized(targetBoolean);
    vcg::tri::UpdateNormal<TriangleMeshType>::PerVertexNormalized(targetBoolean);
    vcg::tri::UpdateBounding<TriangleMeshType>::Box(targetBoolean);

    if (result.face.size() == 0)
        return;

    vcg::tri::UpdateSelection<PolyMeshType>::VertexClear(result);
    vcg::tri::UpdateSelection<PolyMeshType>::FaceClear(result);
    for (const size_t& vId : smoothingVertices) {
        result.vert[vId].SetS();
    }

    vcg::GridStaticPtr<typename TriangleMeshType::FaceType,typename TriangleMeshType::FaceType::ScalarType> Grid;
    Grid.Set(targetBoolean.face.begin(),targetBoolean.face.end());

    //Reproject
    vcg::tri::UpdateBounding<PolyMeshType>::Box(result);
    typename TriangleMeshType::ScalarType maxD=result.bbox.Diag();
    typename TriangleMeshType::ScalarType minD=0;

    for (const size_t& vId : smoothingVertices) {
        typename TriangleMeshType::CoordType closestPT;
        typename TriangleMeshType::FaceType *f=
                vcg::tri::GetClosestFaceBase<TriangleMeshType>(
                    targetBoolean,
                    Grid,
                    result.vert[vId].P(),
                    maxD,minD,
                    closestPT);

        result.vert[vId].P()=closestPT;
    }

    for (int it = 0; it < resultSmoothingIterations; it++) {
        typename PolyMeshType::ScalarType maxDistance = averageEdgeLength(result) * resultSmoothingNRing;

        LaplacianGeodesic(result, 1, maxDistance, 0.7);

        //Reproject
        vcg::tri::UpdateBounding<PolyMeshType>::Box(result);
        typename TriangleMeshType::ScalarType maxD=result.bbox.Diag();
        typename TriangleMeshType::ScalarType minD=0;

        for (const size_t& vId : smoothingVertices) {
            typename TriangleMeshType::CoordType closestPT;
            typename TriangleMeshType::FaceType *f=
                    vcg::tri::GetClosestFaceBase<TriangleMeshType>(
                        targetBoolean,
                        Grid,
                        result.vert[vId].P(),
                        maxD,minD,
                        closestPT);

            result.vert[vId].P()=closestPT;
        }
    }

    typename PolyMeshType::ScalarType maxDistance = averageEdgeLength(result) * resultSmoothingLaplacianNRing;

    LaplacianGeodesic(result, resultSmoothingLaplacianIterations, maxDistance, 0.8);

    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(result);
    vcg::tri::UpdateBounding<PolyMeshType>::Box(result);

#ifdef SAVE_MESHES_FOR_DEBUG
    vcg::tri::io::ExporterOBJ<PolyMeshType>::Save(result, "results/resultAfterReprojection.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}



}
}
