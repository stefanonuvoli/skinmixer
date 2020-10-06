#include "skinmixer_blend_animations.h"

#include <nvl/math/inverse_map.h>
#include <nvl/math/normalization.h>
#include <nvl/math/interpolation.h>

namespace skinmixer {

template<class Model>
void initializeAnimationWeights(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry)
{
    typedef typename nvl::Index Index;
    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;
    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;

    NVL_SUPPRESS_UNUSEDVARIABLE(data);

    Model* targetModel = entry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;
    std::vector<Index>& cluster = entry.birth.entries;

    entry.animationWeights.resize(targetSkeleton.jointNumber(), std::vector<double>(cluster.size(), 0.0));

    std::vector<Index> clusterMap = nvl::getInverseMap(cluster);
    for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
        const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

        JointId parentId = targetSkeleton.parentId(jId);

        for (JointInfo jointInfo : jointInfos) {
            assert(jointInfo.jId != nvl::MAX_INDEX);
            assert(jointInfo.eId != nvl::MAX_INDEX);
            assert(entriesMap[jointInfo.eId] != nvl::MAX_INDEX);

            const Index& cId = clusterMap[jointInfo.eId];



            if (jointInfo.confidence == 1.0) {
                entry.animationWeights[jId][cId] = 1.0;
            }
        }

        nvl::normalize(entry.animationWeights[jId]);
    }
}

template<class Model>
void blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
        const std::vector<nvl::Index>& animationIds,
        nvl::Index& targetAnimationId)
{
    typedef typename nvl::Index Index;

    typedef typename SkinMixerData<Model>::Entry Entry;
    typedef typename SkinMixerData<Model>::BirthInfo::JointInfo JointInfo;

    typedef typename Model::Skeleton Skeleton;
    typedef typename Skeleton::JointId JointId;

    typedef typename Model::Animation Animation;
    typedef typename Animation::Frame Frame;
    typedef typename Animation::Transformation Transformation;

    Model* targetModel = entry.model;
    Skeleton& targetSkeleton = targetModel->skeleton;
    std::vector<Index>& cluster = entry.birth.entries;

    std::vector<Index> clusterMap = nvl::getInverseMap(cluster);

    Animation targetAnimation;

    std::vector<double> times;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index eId = cluster[cId];
        const Entry& currentEntry = data.entry(eId);
        const Model* currentModel = currentEntry.model;

        Index aId = animationIds[cId];

        if (aId == nvl::MAX_INDEX)
            continue;

        const Animation& currentAnimation = currentModel->animation(aId);
        for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
            const Frame& frame = currentAnimation.keyframe(fId);
            times.push_back(frame.time());
        }
    }

    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());

    std::vector<Index> currentFrameId(cluster.size(), 0);
    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];
        std::vector<Transformation> blendedTransformations(targetSkeleton.jointNumber());

        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            std::vector<Transformation> transformations(cluster.size());
            std::vector<double> weights(cluster.size());

            bool transformationComputed = false;

            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::MAX_INDEX);
                assert(jointInfo.eId != nvl::MAX_INDEX);
                assert(entriesMap[jointInfo.eId] != nvl::MAX_INDEX);

                const Index& cId = clusterMap[jointInfo.eId];
                const Index& aId = animationIds[cId];

                if (aId != nvl::MAX_INDEX && !nvl::epsEqual(entry.animationWeights[jId][cId], 0.0)) {
                    transformationComputed = true;
                    weights[cId] = entry.animationWeights[jId][cId];

                    const Animation& currentAnimation = data.entry(jointInfo.eId).model->animation(aId);
                    if (currentFrameId[cId] < currentAnimation.keyframeNumber() - 1 && currentAnimation.keyframe(currentFrameId[cId]).time() < currentTime) {
                        ++currentFrameId[cId];
                        assert(currentAnimation.keyframe(currentFrameId[cId]).time() >= currentTime);
                    }

                    if (currentFrameId[cId] < currentAnimation.keyframeNumber() - 1) {
                        const Frame& frame1 = currentAnimation.keyframe(currentFrameId[cId]);
                        const Frame& frame2 = currentAnimation.keyframe(currentFrameId[cId] + 1);

                        const double& time1 = frame1.time();
                        const double& time2 = frame2.time();
                        const Transformation& transformation1 = frame1.transformation(jointInfo.jId);
                        const Transformation& transformation2 = frame2.transformation(jointInfo.jId);

                        double alpha = (currentTime - time1) / (time2 - time1);

                        Transformation currentTransformation = nvl::interpolateAffine(transformation1, transformation2, alpha);
                        transformations[cId] = currentTransformation;
                    }
                    else {
                        transformations[cId] =  currentAnimation.keyframe(currentAnimation.keyframeNumber() - 1).transformation(jointInfo.jId);
                    }
                }
                else {
                    weights[cId] = 0.0;
                }
            }

            if (transformationComputed) {
                nvl::normalize(weights);
                blendedTransformations[jId] = nvl::interpolateAffine(transformations, weights);
            }
            else {
                blendedTransformations[jId] = targetSkeleton.joint(jId).restTransform();

                JointId parentId = targetSkeleton.parentId(jId);
                if (parentId != nvl::MAX_INDEX) {
                    blendedTransformations[jId] = blendedTransformations[jId] * targetSkeleton.joint(parentId).restTransform().inverse();
                }
            }
        }

        targetAnimation.addKeyframe(currentTime, blendedTransformations);
    }

    targetAnimation.setName("Blended");
    if (targetAnimationId == nvl::MAX_INDEX) {
        targetAnimationId = targetModel->addAnimation(targetAnimation);
    }
    else {
        targetModel->setAnimation(targetAnimationId, targetAnimation);
    }
}

}
