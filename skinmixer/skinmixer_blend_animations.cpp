#include "skinmixer_blend_animations.h"

#include <nvl/math/inverse_map.h>
#include <nvl/math/normalization.h>
#include <nvl/math/interpolation.h>

#include <nvl/models/animation_algorithms.h>

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
    std::vector<Index>& animationsIds = entry.blendingAnimations;
    std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

    animationWeights.resize(targetSkeleton.jointNumber(), std::vector<double>(cluster.size(), 0.0));

    std::vector<Index> clusterMap = nvl::inverseMap(cluster);
    for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
        const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

        JointId parentId = targetSkeleton.parentId(jId);

        int numConfidence = 0;
        for (JointInfo jointInfo : jointInfos) {
            assert(jointInfo.jId != nvl::MAX_INDEX);
            assert(jointInfo.eId != nvl::MAX_INDEX);
            assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);

            const Index& cId = clusterMap[jointInfo.eId];

            if (jointInfo.confidence == 1.0) {
                animationWeights[jId][cId] = 1.0;
                numConfidence++;
            }
        }
        if (numConfidence > 1) {
            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::MAX_INDEX);
                assert(jointInfo.eId != nvl::MAX_INDEX);
                assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);
                assert(parentId != nvl::MAX_INDEX);

                const Index& cId = clusterMap[jointInfo.eId];

                const std::vector<JointInfo>& parentJointInfos = entry.birth.joint[parentId];

                for (JointInfo parentJointInfo : parentJointInfos) {
                    const Index& parentCId = clusterMap[parentJointInfo.eId];

                    if (jointInfo.confidence == 1.0 && parentJointInfo.confidence == 1 && parentCId != cId) {
                        animationWeights[jId][cId] = 0.0;
                    }
                }
            }
        }

        nvl::normalize(animationWeights[jId]);
    }

    animationsIds.resize(cluster.size());
    for (Index cId = 0; cId < cluster.size(); cId++) {
        Index eId = cluster[cId];
        if (data.entry(eId).model->animationNumber() > 0) {
            animationsIds[cId] = 0;
        }
        else {
            animationsIds[cId] = nvl::MAX_INDEX;
        }
    }
}

template<class Model>
void blendAnimations(
        SkinMixerData<Model>& data,
        typename SkinMixerData<Model>::Entry& entry,
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
    const std::vector<nvl::Index>& animationIds = entry.blendingAnimations;
    const std::vector<std::vector<double>>& animationWeights = entry.blendingAnimationWeights;

    std::vector<Index> clusterMap = nvl::inverseMap(cluster);

    Animation targetAnimation;

    std::vector<std::vector<Frame>> localFrames(cluster.size());
    std::vector<double> times;
    for (Index cId = 0; cId < cluster.size(); ++cId) {
        Index eId = cluster[cId];
        const Entry& currentEntry = data.entry(eId);
        const Model* currentModel = currentEntry.model;
        const Skeleton& currentSkeleton = currentModel->skeleton;

        Index aId = animationIds[cId];

        if (aId == nvl::MAX_INDEX)
            continue;

        const Animation& currentAnimation = currentModel->animation(aId);
        for (Index fId = 0; fId < currentAnimation.keyframeNumber(); ++fId) {
            Frame frame = currentAnimation.keyframe(fId);

            nvl::animationComputeLocalFrame(currentSkeleton, frame);
            localFrames[cId].push_back(frame);

            times.push_back(frame.time());
        }
    }

    if (times.empty())
        times.push_back(0.0);

    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());

    std::vector<Index> currentFrameId(cluster.size(), 0);
    for (Index i = 0; i < times.size(); ++i) {
        const double& currentTime = times[i];
        std::vector<Transformation> blendedTransformations(targetSkeleton.jointNumber());

        for (JointId jId = 0; jId < targetSkeleton.jointNumber(); ++jId) {
            const std::vector<JointInfo>& jointInfos = entry.birth.joint[jId];

            std::vector<Transformation> transformations(cluster.size(), Transformation::Identity());
            std::vector<double> weights(cluster.size(), 0.0);

            for (JointInfo jointInfo : jointInfos) {
                assert(jointInfo.jId != nvl::MAX_INDEX);
                assert(jointInfo.eId != nvl::MAX_INDEX);
                assert(clusterMap[jointInfo.eId] != nvl::MAX_INDEX);

                const Index& cId = clusterMap[jointInfo.eId];
                const Index& aId = animationIds[cId];

                //Avoid numerical errors
                if (!nvl::epsEqual(animationWeights[jId][cId], 0.0)) {
                    weights[cId] = animationWeights[jId][cId];
                    if (aId != nvl::MAX_INDEX) {
                        const std::vector<Frame>& currentLocalFrames = localFrames[cId];
                        const Skeleton& currentSkeleton = data.entry(jointInfo.eId).model->skeleton;

                        if (currentFrameId[cId] < currentLocalFrames.size() - 1 && currentLocalFrames[currentFrameId[cId]].time() < currentTime) {
                            ++currentFrameId[cId];
                            assert(currentLocalFrames[currentFrameId[cId]].time() >= currentTime);
                        }

                        if (currentFrameId[cId] < currentLocalFrames.size() - 1) {
                            const Frame& frame1 = currentLocalFrames[currentFrameId[cId]];
                            const Frame& frame2 = currentLocalFrames[currentFrameId[cId] + 1];

                            const double& time1 = frame1.time();
                            const double& time2 = frame2.time();
                            const Transformation& transformation1 = frame1.transformation(jointInfo.jId) * targetSkeleton.joint(jId).restPose().inverse();
                            const Transformation& transformation2 = frame2.transformation(jointInfo.jId) * targetSkeleton.joint(jId).restPose().inverse();

                            double alpha = (currentTime - time1) / (time2 - time1);

                            transformations[cId] = nvl::interpolateAffine(transformation1, transformation2, alpha);
                        }
                        else {
                            transformations[cId] = currentLocalFrames[currentLocalFrames.size() - 1].transformation(jointInfo.jId) * targetSkeleton.joint(jId).restPose().inverse();
                        }
                    }
                    else {
                        const Skeleton& currentSkeleton = data.entry(jointInfo.eId).model->skeleton;
                        if (!currentSkeleton.isRoot(jointInfo.jId)) {
                            transformations[cId] = targetSkeleton.parent(jId).restPose().inverse();
                        }
                        else {
                            transformations[cId] = Transformation::Identity();
                        }
                    }
                }
            }

            nvl::normalize(weights);

            double sum = 0.0;
            for (const double& value : weights) {
                sum += value;
            }
            if (nvl::epsEqual(sum, 0.0)) {
                for (double& value : weights) {
                    value = 1.0 / weights.size();
                }
            }

            blendedTransformations[jId] = nvl::interpolateAffine(transformations, weights) * targetSkeleton.joint(jId).restPose();
        }

        Frame newFrame(currentTime, blendedTransformations);
        nvl::animationComputeGlobalFrame(targetSkeleton, newFrame);
        targetAnimation.addKeyframe(newFrame);
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
