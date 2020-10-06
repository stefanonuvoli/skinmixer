#include "skinmixer_blend_skinningweights.h"

#include <nvl/models/model_normalization.h>

namespace skinmixer {

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

    Model* targetModel = entry.model;
    SkinningWeights& targetSkinningWeights = targetModel->skinningWeights;
    Mesh& targetMesh = targetModel->mesh;
    Skeleton& targetSkeleton = targetModel->skeleton;

    targetSkinningWeights.initialize(targetMesh.nextVertexId(), targetSkeleton.jointNumber());

    for (VertexId vId = 0; vId < targetMesh.nextVertexId(); ++vId) {
        if (targetMesh.isVertexDeleted(vId))
            continue;

        const std::vector<VertexInfo>& vertexInfos = entry.birth.vertex[vId];

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
                        double interpolatedWeight = 0;
                        const Face& face = currentMesh.face(vertexInfo.closestFaceId);
                        for (VertexId j = 0; j < face.vertexNumber(); j++) {
                            interpolatedWeight += currentSkinningWeights.weight(face.vertexId(j), jointInfo.jId);
                        }
                        interpolatedWeight /= face.vertexNumber();

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

}
