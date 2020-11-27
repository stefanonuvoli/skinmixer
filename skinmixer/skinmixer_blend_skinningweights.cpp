#include "skinmixer_blend_skinningweights.h"

#include <nvl/models/model_normalization.h>

#include <nvl/math/constants.h>
#include <nvl/math/numeric_limits.h>
#include <nvl/math/closest_point.h>
#include <nvl/math/barycentric_interpolation.h>

namespace skinmixer {

namespace internal {

template<class Mesh, class SkinningWeights, class JointId>
typename SkinningWeights::Scalar interpolateSkinningWeightOnFace(
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

                        weight += vertexInfo.weight * interpolatedWeight;
                        sumSelectValues += vertexInfo.weight;
                    }
                }
            }

            weight /= sumSelectValues;
            if (weight > nvl::EPSILON) {
                if (targetSkeleton.childNumber(jId) > 0) {
                    targetSkinningWeights.setWeight(vId, jId, weight);
                }
                else {
                    targetSkinningWeights.setWeight(vId, targetSkeleton.parentId(jId), weight);
                }
            }
        }
    }

    targetSkinningWeights.updateNonZeros();
    nvl::modelNormalizeSkinningWeights(*targetModel);
}

namespace internal {

template<class Mesh, class SkinningWeights, class JointId>
typename SkinningWeights::Scalar interpolateSkinningWeightOnFace(
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
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    //Get polygon points and skinning weights
    const Face& face = mesh.face(faceId);

    std::vector<Point> polygon(face.vertexNumber());
    std::vector<SkinningWeightsScalar> values(face.vertexNumber());
    for (VertexId j = 0; j < face.vertexNumber(); ++j) {
        const VertexId& vId = face.vertexId(j);

        polygon[j] = mesh.vertex(vId).point();
        values[j] = skinningWeights.weight(vId, jointId);
    }

    //Interpolation on polygon using barycenter subdivision
    SkinningWeightsScalar value = nvl::barycentricInterpolationBarycenterSubdivision(
        polygon,
        point,
        values,
        true);

    return value;
}

}

}
