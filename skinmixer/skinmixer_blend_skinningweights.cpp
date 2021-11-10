#include "skinmixer_blend_skinningweights.h"

#include <nvl/models/model_normalization.h>

#include <nvl/math/constants.h>
#include <nvl/math/numeric_limits.h>
#include <nvl/math/closest_point.h>
#include <nvl/math/barycentric_interpolation.h>
#include <nvl/math/inverse_map.h>

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
        SkinMixerData<Model>& data,
        std::vector<nvl::Index> cluster,
        typename SkinMixerData<Model>::Entry& resultEntry)
{
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::VertexInfo VertexInfo;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;
    typedef typename Model::SkinningWeights SkinningWeights;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;
    typedef typename Model::Mesh Mesh;
    typedef typename Mesh::VertexId VertexId;
    typedef typename Mesh::Point Point;
    typedef nvl::Index Index;

    std::vector<Index> clusterMap = nvl::inverseMap(cluster);

    Model* targetModel = resultEntry.model;
    SkinningWeights& targetSkinningWeights = targetModel->skinningWeights;
    Mesh& targetMesh = targetModel->mesh;
    Skeleton& targetSkeleton = targetModel->skeleton;

    targetModel->initializeSkinningWeights();

    std::vector<std::vector<JointId>> preJointMap(cluster.size());
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Entry& birthEntry = data.entry(cluster[cId]);
        preJointMap[cId].resize(birthEntry.model->skeleton.jointNumber(), nvl::MAX_INDEX);
    }

    for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
        const std::vector<JointInfo>& jointInfos = resultEntry.birth.joint[jId];

        for (const JointInfo& jointInfo : jointInfos) {
            if (jointInfo.confidence == 1.0) {
                const Index& cId = clusterMap[jointInfo.eId];

                assert(jointInfo.jId != nvl::MAX_INDEX);
                assert(preJointMap[cId][jointInfo.jId] == nvl::MAX_INDEX);

                preJointMap[cId][jointInfo.jId] = jId;
            }
        }
    }

    for (VertexId vId = 0; vId < targetMesh.nextVertexId(); ++vId) {
        if (targetMesh.isVertexDeleted(vId))
            continue;

        const std::vector<VertexInfo>& vertexInfos = resultEntry.birth.vertex[vId];

        for (const VertexInfo& vertexInfo : vertexInfos) {
            const Entry& currentEntry = data.entry(vertexInfo.eId);
            const Model* currentModel = currentEntry.model;
            const Mesh& currentMesh = currentModel->mesh;
            const SkinningWeights& currentSkinningWeights = currentModel->skinningWeights;

            if (vertexInfo.vId != nvl::MAX_INDEX) {
                const Index& cId = clusterMap[vertexInfo.eId];

                assert(vertexInfo.weight == 1.0);

                const std::vector<Index>& nonZeros = currentSkinningWeights.nonZeroWeights(vertexInfo.vId);
                for (const Index& jId : nonZeros) {
                    if (preJointMap[cId][jId] != nvl::MAX_INDEX) {
                        const SkinningWeightsScalar& sw = currentSkinningWeights.weight(vertexInfo.vId, jId);

                        assert(preJointMap[cId][jId] != nvl::MAX_INDEX);
                        if (sw > nvl::EPSILON) {
                            targetSkinningWeights.weight(vId, preJointMap[cId][jId]) += sw;
                        }
                    }
                }
            }
            else {
                const Point& point = targetMesh.vertex(vId).point();

                for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {

                    const std::vector<JointInfo>& jointInfos = resultEntry.birth.joint[jId];

                    SkinningWeightsScalar sw = 0.0;
                    for (const JointInfo& jointInfo : jointInfos) {
                        assert (jointInfo.jId != nvl::MAX_INDEX);

                        if (jointInfo.eId != vertexInfo.eId) {
                            continue;
                        }

                        SkinningWeightsScalar interpolatedWeight = internal::interpolateSkinningWeightOnFace(currentMesh, currentSkinningWeights, jointInfo.jId, vertexInfo.closestFaceId, point);

                        sw += vertexInfo.weight * interpolatedWeight;
                    }

                    if (sw > nvl::EPSILON) {
                        targetSkinningWeights.weight(vId, jId) += sw;
                    }
                }
            }
        }
    }

    targetSkinningWeights.updateNonZeros();

    nvl::modelNormalizeSkinningWeights(*targetModel);

    for (VertexId vId = 0; vId < targetMesh.nextVertexId(); ++vId) {
        if (targetMesh.isVertexDeleted(vId))
            continue;

        const std::vector<Index>& nonZeros = targetSkinningWeights.nonZeroWeights(vId);
        for (const Index& jId : nonZeros) {
            SkinningWeightsScalar& weight = targetSkinningWeights.weight(vId, jId);
            assert(weight >= 0.0 && weight <= 1.0);

            if (nvl::epsEqual(weight, 0.0)) {
                weight = 0.0;
            }
            else if (nvl::epsEqual(weight, 1.0)) {
                weight = 1.0;
            }
        }
    }

    targetSkinningWeights.updateNonZeros();

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
    typedef typename Mesh::Point Point;
    typedef typename SkinningWeights::Scalar SkinningWeightsScalar;

    //Get polygon points and skinning weights
    const Face& face = mesh.face(faceId);

    std::vector<Point> polygon(face.vertexNumber());
    std::vector<SkinningWeightsScalar> values(face.vertexNumber());

    bool isZero = true;
    for (VertexId j = 0; j < face.vertexNumber(); ++j) {
        const VertexId& vId = face.vertexId(j);

        polygon[j] = mesh.vertex(vId).point();
        values[j] = skinningWeights.weight(vId, jointId);

        if (values[j] > nvl::EPSILON) {
            isZero = false;
        }
    }

    SkinningWeightsScalar value;
    if (isZero) {
        value = 0.0;
    }
    else {
        //Interpolation on polygon using barycenter subdivision
        value = nvl::barycentricInterpolationBarycenterSubdivision(
            polygon,
            point,
            values,
            true);
    }

    return value;
}

}

}
