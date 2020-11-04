#include "skinmixer_blend_skinningweights.h"

#include <nvl/models/model_normalization.h>

#include <nvl/math/constants.h>
#include <nvl/math/numeric_limits.h>
#include <nvl/math/closest_point.h>
#include <nvl/math/barycentric_interpolation.h>

namespace skinmixer {

namespace internal {

template<class Mesh, class SkinningWeights, class JointId>
double interpolateSkinningWeightOnFace(
        const Mesh& mesh,
        const SkinningWeights& skinningWeights,
        const JointId& jointId,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point);
}

template<class Model>
void blendSkinningWeights(
        const SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::VertexInfo VertexInfo;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Point Point;

    Model* targetModel = entry.model;
    SkinningWeights& targetSkinningWeights = targetModel->skinningWeights;
    Mesh& targetMesh = targetModel->mesh;
    Skeleton& targetSkeleton = targetModel->skeleton;

    targetModel->initializeSkinningWeights();

    for (VertexId vId = 0; vId < targetMesh.nextVertexId(); ++vId) {
        if (targetMesh.isVertexDeleted(vId))
            continue;

        const std::vector<VertexInfo>& vertexInfos = entry.birth.vertex[vId];

        const Point& point = targetMesh.vertex(vId).point();

        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            double weight = 0.0;
            double sumSelectValues = 0;

            for (VertexInfo vertexInfo : vertexInfos) {
                for (JointInfo jointInfo : jointInfos) {
                    assert (jointInfo.jId != nvl::MAX_INDEX);

                    if (jointInfo.eId != vertexInfo.eId) {
                        continue;
                    }

                    const Entry& currentEntry = data.entry(vertexInfo.eId);
                    const Model* currentModel = currentEntry.model;
                    const Mesh& currentMesh = currentModel->mesh;
                    const SkinningWeights& currentSkinningWeights = currentModel->skinningWeights;

                    if (vertexInfo.vId != nvl::MAX_INDEX) {
                        weight += vertexInfo.weight * currentSkinningWeights.weight(vertexInfo.vId, jointInfo.jId);
                        sumSelectValues += vertexInfo.weight;
                    }
                    else {
                        double interpolatedWeight = internal::interpolateSkinningWeightOnFace(currentMesh, currentSkinningWeights, jointInfo.jId, vertexInfo.closestFaceId, point);

//                        double interpolatedWeight = 0;
//                        const Face& face = currentMesh.face(vertexInfo.closestFaceId);
//                        for (VertexId j = 0; j < face.vertexNumber(); j++) {
//                            interpolatedWeight += currentSkinningWeights.weight(face.vertexId(j), jointInfo.jId);
//                        }
//                        interpolatedWeight /= face.vertexNumber();

                        weight += vertexInfo.weight * interpolatedWeight;
                        sumSelectValues += vertexInfo.weight;
                    }
                }
            }

            weight /= sumSelectValues;
            if (weight > nvl::EPSILON) {
                targetSkinningWeights.setWeight(vId, jId, weight);
            }
        }
    }

    targetSkinningWeights.updateNonZeros();
    nvl::modelNormalizeSkinningWeights(*targetModel);
}

namespace internal {
template<class Mesh, class SkinningWeights, class JointId>
double interpolateSkinningWeightOnFace(
        const Mesh& mesh,
        const SkinningWeights& skinningWeights,
        const JointId& jointId,
        const typename Mesh::FaceId& faceId,
        const typename Mesh::Point& point)
{
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Scalar Scalar;
    typedef typename Mesh::Point Point;

    const Face& face = mesh.face(faceId);

    VertexId bestV1, bestV2, bestV3;
    Point bestPoint;
    Scalar maxDistance = nvl::maxLimitValue<Scalar>();
    for (VertexId j = 0; j < face.vertexNumber() - 2; ++j) {
        const VertexId& v1 = face.vertexId(0);
        const VertexId& v2 = face.vertexId(j + 1);
        const VertexId& v3 = face.vertexId(j + 2);

        const Point& p1 = mesh.vertex(v1).point();
        const Point& p2 = mesh.vertex(v2).point();
        const Point& p3 = mesh.vertex(v3).point();

        Point closest = nvl::closestPointOnTriangle(p1, p2, p3, point);
        Scalar distance = (point - closest).norm();
        if (distance < maxDistance) {
            bestV1 = v1;
            bestV2 = v2;
            bestV3 = v3;
            bestPoint = closest;
        }
    }

    const Point& p1 = mesh.vertex(bestV1).point();
    const Point& p2 = mesh.vertex(bestV2).point();
    const Point& p3 = mesh.vertex(bestV3).point();
    std::vector<Scalar> barycentricCoordinates = nvl::barycentricCoordinates(p1, p2, p3, bestPoint);

    double weight = nvl::barycentricInterpolation(
        skinningWeights.weight(bestV1, jointId),
        skinningWeights.weight(bestV2, jointId),
        skinningWeights.weight(bestV3, jointId),
        barycentricCoordinates);

    return weight;
}

}

}
