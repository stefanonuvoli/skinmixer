#include "skinmixer_blend_skinningweights.h"

namespace skinmixer {

template<class Model>
void blendSkinningWeights(
        const SkinMixerData<Model>& data,
        const std::vector<nvl::Index>& cluster,
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
            float sumSelectValues = 0;

            for (VertexInfo vertexInfo : vertexInfos) {
                if (vId == nvl::MAX_ID)
                    continue;

                for (JointInfo jointInfo : jointInfos) {
                    if (jId == nvl::MAX_ID)
                        continue;

                    if (jointInfo.eId != vertexInfo.eId) {
                        continue;
                    }

                    const Entry& currentEntry = data.entry(vertexInfo.eId);
                    const Model* currentModel = currentEntry.model;
                    const SkinningWeights& currentSkinningWeights = currentModel->skinningWeights;

                    weight += vertexInfo.selectValue * currentSkinningWeights.weight(vertexInfo.vId, jointInfo.jId);
                    sumSelectValues += vertexInfo.selectValue;
                }
            }

            weight /= sumSelectValues;
            targetSkinningWeights.setWeight(vId, jId, weight);
        }
    }

    targetSkinningWeights.updateNonZeros();
}

}
