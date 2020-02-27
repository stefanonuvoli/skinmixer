#include "skeleton_segmentation.h"

#include <MultiLabelOptimization/GCoptimization.h>

#include <nvl/models/mesh_adjacencies.h>
#include <nvl/models/skeleton_adjacencies.h>
#include <nvl/models/mesh_geometric_information.h>

#include <nvl/math/comparisons.h>

#include <iostream>
#include <queue>

#define MAXCOST GCO_MAX_ENERGYTERM

namespace skinmixer {

template<class Model>
std::vector<int> skeletonSegmentationMax(
        const Model& model)
{
    typedef typename Model::Skeleton Skeleton;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Skeleton::Joint SkeletonJoint;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    std::vector<int> faceSegmentation(mesh.faceNumber(), -1);
    for (const Face& face : mesh.faces()) {
        std::vector<double> score(skeleton.jointNumber(), 0);

        for (const VertexId& vId : face.vertexIds()) {
            for (const SkeletonJoint& joint : skeleton.joints()) {
                score[joint.id()] += skinningWeights.weight(vId, joint.id());
            }
        }

        size_t bestIndex = 0;
        for (size_t i = 0; i < score.size(); ++i) {
            if (score[i] >= score[bestIndex]) {
                bestIndex = i;
            }
        }

        faceSegmentation[face.id()] = static_cast<int>(bestIndex);
    }

    return faceSegmentation;
}

template<class Model>
std::vector<int> skeletonSegmentationGraphcut(
        const Model& model,
        float compactness)
{
    typedef typename Model::Skeleton Skeleton;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::VertexId VertexId;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    const nvl::Size nFaces = mesh.nextFaceId();
    const nvl::Size nJoints = skeleton.jointNumber();

    //Initialize to -1 direction association for each face
    std::vector<int> faceSegmentation;
    faceSegmentation.clear();
    faceSegmentation.resize(nFaces, -1);

    //Get mesh adjacencies    
    std::vector<std::vector<typename Mesh::FaceId>> ffAdj = nvl::meshFaceFaceAdjacencies(mesh);

    //Data cost
    std::vector<float> dataCost(nFaces * nJoints);

    for (FaceId fId = 0; fId < nFaces; fId++) {
        const Face& face = mesh.face(fId);

        for (unsigned int jId = 0; jId < nJoints; ++jId) {
            float cost = 0;

            for (const VertexId& vId : face.vertexIds()) {
                double vertexWeight = skinningWeights.weight(vId, jId);

                cost += static_cast<float>(1.0 - vertexWeight);
            }

            cost /= face.vertexIds().size();

            assert(cost >= 0);

            if (cost >= 1.0f)
                cost = MAXCOST;

            dataCost[fId * nJoints + jId] = cost;
        }
    }

    //Smooth cost
    std::vector<float> smoothCost(nJoints * nJoints);

    for (unsigned int l1 = 0; l1 < nJoints; ++l1) {
        for (unsigned int l2 = 0; l2 < nJoints; ++l2) {
            float cost;

            if (l1 == l2) {
                cost = 0.f;
            }
            else {
                cost = compactness;
            }

            smoothCost[l1 * nJoints + l2] = cost;
        }
    }

    try {
        GCoptimizationGeneralGraph* gc = new GCoptimizationGeneralGraph(nFaces, nJoints);

        //Set costs
        gc->setDataCost(dataCost.data());
        gc->setSmoothCost(smoothCost.data());

        //Set adjacencies
        std::vector<bool> visited(nFaces, false);

        for (FaceId fId = 0; fId < nFaces; fId++) {
            visited[fId] = true;

            for (FaceId adjId : ffAdj[fId]) {
                if (!visited[adjId]) {
                    gc->setNeighbors(static_cast<int>(fId), static_cast<int>(adjId));
                }
            }
        }

        //Compute graph cut
        gc->swap(-1); // -1 => run until convergence [convergence is guaranteed]

        //Set associations
        for (FaceId fId = 0; fId < nFaces; fId++) {
            int label = gc->whatLabel(static_cast<int>(fId));

            faceSegmentation[fId] = label;
        }

        //Delete data
        delete gc;
    }
    catch (GCException e) {
        std::cerr << "\n\n!!!GRAPH-CUT EXCEPTION!!!\nCheck logfile\n\n" << std::endl;
        e.Report();
    }

    return faceSegmentation;
}


template<class Model>
std::vector<int> skeletonBinarySegmentationGraphcut(
        const Model& model,
        float compactness,
        typename Model::Skeleton::JointId targetJointId,
        std::vector<int>& jointSegmentation)
{
    typedef typename Model::Skeleton Skeleton;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::FaceId FaceId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Point Point;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Skeleton::JointId JointId;

    const Mesh& mesh = model.mesh;
    const Skeleton& skeleton = model.skeleton;
    const SkinningWeights& skinningWeights = model.skinningWeights;

    const nvl::Size nFaces = mesh.nextFaceId();
    const nvl::Size nJoints = skeleton.jointNumber();

    //Get a set of after joints starting from the target joint
    std::vector<JointId> descendants = nvl::skeletonJointDescendants(skeleton, targetJointId);
    std::unordered_set<JointId> descendantSet(descendants.begin(), descendants.end());

    //If it has no children (useless to compute, cannot be detached)
    if (descendantSet.empty()) {
        return std::vector<int>(nFaces, 0);
    }

    //Max association
    std::vector<int> maxAssociation = skeletonSegmentationMax(model);

    //Weight per face for the target joint
    std::vector<float> weightPerFaceTargetJoint(nFaces, 0);
    for (FaceId fId = 0; fId < nFaces; fId++) {
        const Face& face = mesh.face(fId);

        for (const VertexId& vId : face.vertexIds()) {
            double vertexWeight = skinningWeights.weight(vId, targetJointId);
            weightPerFaceTargetJoint[fId] += static_cast<float>(vertexWeight);
        }

        weightPerFaceTargetJoint[fId] /= face.vertexIds().size();
    }

    //Segmentations
    std::vector<int> faceSegmentation(nFaces);
    jointSegmentation.resize(nJoints);

    //Get joint segmentation
    for (JointId jId = 0; jId < nJoints; jId++) {
        if (descendantSet.find(jId) != descendantSet.end() || jId == targetJointId) {
            jointSegmentation[jId] = 1;
        }
        else {
            jointSegmentation[jId] = 0;
        }
    }

    //Get initial face segmentation
    unsigned int numberOfOnes = 0;
    for (FaceId fId = 0; fId < nFaces; fId++) {
        if (mesh.isFaceDeleted(fId)) {
            continue;
        }

        assert(maxAssociation[fId] >= 0);
        int maxLabel = maxAssociation[fId];

        if (jointSegmentation[maxLabel] == 1) {
            numberOfOnes++;
            faceSegmentation[fId] = 1;
        }
        else if (nvl::epsEqual(weightPerFaceTargetJoint[fId], 0.0f)) {
            faceSegmentation[fId] = 0;
        }
        else {
            faceSegmentation[fId] = -1;
        }
    }

    //Get at least one face
    double currentThreshold = 0.99;
    while (numberOfOnes == 0 && currentThreshold > 0) {
        numberOfOnes = 0;

        for (FaceId fId = 0; fId < nFaces; fId++) {
            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            if (weightPerFaceTargetJoint[fId] >= currentThreshold) {
                faceSegmentation[fId] = 1;
                numberOfOnes++;
            }
            else {
                for (const JointId& jId : descendantSet) {
                    const Face& face = mesh.face(fId);

                    double weight = 0;
                    for (const VertexId& vId : face.vertexIds()) {
                        double vertexWeight = skinningWeights.weight(vId, jId);
                        weight += static_cast<float>(vertexWeight);
                    }
                    weight /= face.vertexIds().size();

                    if (weight >= currentThreshold) {
                        faceSegmentation[fId] = 1;
                        numberOfOnes++;

                        break;
                    }
                }
            }
        }

        currentThreshold -= 0.01;
    }

    //Get mesh adjacencies
    std::vector<std::vector<nvl::Index>> ffEdges;
    std::vector<std::vector<typename Mesh::FaceId>> ffAdj = nvl::meshFaceFaceAdjacencies(mesh, ffEdges);
    std::vector<nvl::Index> faceComponentMap;
    std::vector<std::vector<typename Mesh::FaceId>> connectedComponents = nvl::meshConnectedComponents(mesh, ffAdj, faceComponentMap);
    std::vector<std::vector<nvl::Index>> borderEdges = nvl::meshFaceBorderEdges(mesh, ffEdges);

    //Connect borders to some components (just in the ffAdj vector, non geometrical changes)
    if (connectedComponents.size() > 1) {
        for (FaceId f1Id = 0; f1Id < nFaces; f1Id++) {
            if (mesh.isFaceDeleted(f1Id)) {
                continue;
            }

            const Face& face1 = mesh.face(f1Id);

            if (face1.vertexNumber() <= ffAdj[f1Id].size()) {
                continue;
            }

            for (nvl::Index edgePos1 : borderEdges[f1Id]) {
                Point p1 = nvl::meshFaceEdgeMidpoint(mesh, f1Id, edgePos1);

                FaceId bestFace = nvl::MAX_ID;
                Scalar bestDistance = std::numeric_limits<Scalar>::max();

                for (FaceId f2Id = 0; f2Id < nFaces; f2Id++) {
                    if (mesh.isFaceDeleted(f2Id)) {
                        continue;
                    }

                    if (faceComponentMap[f1Id] != faceComponentMap[f2Id]) {
                        const Face& face2 = mesh.face(f2Id);

                        for (nvl::Index edgePos2 = 0; edgePos2 < face2.vertexNumber(); edgePos2++) {
                            Point p2 = nvl::meshFaceEdgeMidpoint(mesh, f2Id, edgePos2);

                            Scalar distance = (p2 - p1).norm();
                            if (distance < bestDistance) {
                                bestFace = f2Id;
                                bestDistance = distance;
                            }
                        }
                    }
                }

                assert(bestFace < nvl::MAX_ID);
                assert(bestDistance < std::numeric_limits<Scalar>::max());

                ffAdj[bestFace].push_back(f1Id);
                ffAdj[f1Id].push_back(bestFace);
            }

            borderEdges[f1Id].clear();
        }
    }

    //Data cost
    std::vector<float> dataCost(nFaces * 2);
    for (FaceId fId = 0; fId < nFaces; fId++) {
        if (mesh.isFaceDeleted(fId)) {
            dataCost[fId * 2 + 0] = 0;
            dataCost[fId * 2 + 1] = MAXCOST;
            continue;
        }

        if (faceSegmentation[fId] == 0) {
            dataCost[fId * 2 + 0] = 0;
            dataCost[fId * 2 + 1] = MAXCOST;
        }
        else if (faceSegmentation[fId] == 1) {
            dataCost[fId * 2 + 0] = MAXCOST;
            dataCost[fId * 2 + 1] = 0;
        }
        else {
            assert(faceSegmentation[fId] == -1);

            float weight = weightPerFaceTargetJoint[fId];

            assert(weight >= 0 && weight <= 1);

            dataCost[fId * 2 + 0] = weight < 1 ? weight : MAXCOST;
            dataCost[fId * 2 + 1] = weight > 0 ? 1 - weight : MAXCOST;
        }
    }

    //Smooth cost
    std::vector<float> smoothCost(2 * 2);
    for (unsigned int l1 = 0; l1 < 2; ++l1) {
        for (unsigned int l2 = 0; l2 < 2; ++l2) {
            float cost;

            if (l1 == l2) {
                cost = 0.f;
            }
            else {
                cost = compactness;
            }

            smoothCost[l1 * 2 + l2] = cost;
        }
    }

    try {
        GCoptimizationGeneralGraph* gc = new GCoptimizationGeneralGraph(nFaces, 2);

        //Set costs
        gc->setDataCost(dataCost.data());
        gc->setSmoothCost(smoothCost.data());

        //Set adjacencies
        std::vector<bool> visited(nFaces, false);

        for (FaceId fId = 0; fId < nFaces; fId++) {
            visited[fId] = true;

            if (mesh.isFaceDeleted(fId)) {
                continue;
            }

            for (FaceId adjId : ffAdj[fId]) {
                if (!visited[adjId]) {
                    gc->setNeighbors(static_cast<int>(fId), static_cast<int>(adjId));
                }
            }
        }

        //Compute graph cut
        gc->swap(-1); // -1 => run until convergence [convergence is guaranteed]

        //Set associations
        for (FaceId fId = 0; fId < nFaces; fId++) {
            int label = gc->whatLabel(static_cast<int>(fId));

            faceSegmentation[fId] = label;
        }

        //Delete data
        delete gc;
    }
    catch (GCException e) {
        std::cerr << "\n\n!!!GRAPH-CUT EXCEPTION!!!\nCheck logfile\n\n" << std::endl;
        e.Report();
    }

    return faceSegmentation;
}

}
